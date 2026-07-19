#include "serialCommand.h"
#include "control.h"
// Instantiate objects
serialCom serialCLI;
motorControl robot;
commands currentCommand;
unsigned long preMillis;

void operate();
bool ifspin();
void topicPrint();
char getStateID();
//interrupt service routines
void refA_ISR();
void refB_ISR();    
void refC_ISR();

//setup timer 
extern "C" {
  // TC2 Channel 0 (Timer 6)
  void setupTimer6(uint32_t uSeconds) {
    uint32_t frequency = 1000000.0 / uSeconds;
    pmc_set_writeprotect(false); //disable write protect
    pmc_enable_periph_clk(ID_TC6); // ID of TC2 Channel 0

    // configure TC2, Channel 0
    TC_Configure(TC2, 0, TC_CMR_TCCLKS_TIMER_CLOCK2 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);
    
    // MCK/8 = 10.5MHz. RC = 10,500,000 / frequency
    uint32_t rc = 10500000 / frequency;
    TC_SetRC(TC2, 0, rc);

    // turn interrupt on RC compare
    TC2->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
    TC2->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;
    
    NVIC_ClearPendingIRQ(TC6_IRQn);
    NVIC_EnableIRQ(TC6_IRQn);
    // Demote this 50 kHz stepper ISR below the Servo library's pulse-timing
    // interrupt (SAM3X: first servos use TC1 ch0 -> TC3_IRQn, left at the
    // default priority 0). A higher number = lower priority on Cortex-M3, so
    // the servo interrupt can now preempt robot.run() and its edges stay
    // accurate even while the stepper ISR is busy. A few microseconds of
    // stepper jitter is harmless; a starved servo interrupt is not (it can
    // stop the servo pulse train entirely). See doc/bug-report.md.
    NVIC_SetPriority(TC6_IRQn, 8);

    TC_Start(TC2, 0);
  }

  // Handler of Timer 6
  void TC6_Handler() {
    // read SR to erase interrupt flag of TC2 Channel 0
    TC2->TC_CHANNEL[0].TC_SR;
    //put callback functions here
    robot.run();
  }
}

void setup() {
    ComPort.begin(115200); // Make sure this matches your monitor
    robot.init();
    //setup interrupt/timer
    setupTimer6(CALLBACK_TIME); //setup timer with CALLBACK_TIME microsecond interval
    pinMode(refA, INPUT_PULLUP);
    pinMode(refB, INPUT_PULLUP);
    pinMode(refC, INPUT_PULLUP);
    attachInterrupt(refA, refA_ISR, FALLING);
    attachInterrupt(refB, refB_ISR, FALLING);
    attachInterrupt(refC, refC_ISR, FALLING);
    preMillis = millis();
}

void loop() {
    if (!debugMode){
        robot.turnSW_A(digitalRead(refA));
        robot.turnSW_B(digitalRead(refB));
        robot.turnSW_C(digitalRead(refC));
    }
    operate();
    if (errorFlag == error_limitation_breaked){

    }
    if (ifspin()){
        topicPrint();
        if (errorFlag != error_none){
            serialCLI.sendingPackage('E', errorFlag + '0', robot.angles, robot.limitSwitchMask());
            if (robot.safety_check('a'))
            robot.jointBrake('a');
            if (robot.safety_check('b'))
            robot.jointBrake('b');
            if (robot.safety_check('c'))
            robot.jointBrake('c');

            errorFlag = error_none;
        }
    }
}


void operate() { // Read Serial and move motors
    // 1. Check for new commands
    commands cmd = serialCLI.readNode(); // Returns enum

    if (cmd != commands::cmd_none || getStateID() == 'D' || currentCommand == commands::cmd_moveref || currentCommand == commands::cmd_abort)
        currentCommand = cmd;
    // 2. Process Command
    robot.posWriting = 1;
    if (cmd == cmd_move) { // Relative move command
        // Get the parsed data
        serialCLI.getArgument();
        robot.resetProfile(); // relative moves run at nominal speed, not a leftover synced profile

        // Loop through the arguments (max 4)
        for(int i=0; i<maxArguments; i++) {
            char tag = serialCLI.Indexs[i];    // e.g., 'a'
            double val = serialCLI.Arguments[i]; // e.g., 90.0

            if(tag != ' ' && tag != 0) {
                robot.move(tag, (float)val);
            }
        }
    }
    else if (cmd == cmd_moveto) { // Absolute move command, steppers time-synchronized
        serialCLI.getArgument();
        bool useAxis[3] = {false, false, false};
        float targetDeg[3] = {0.0f, 0.0f, 0.0f};
        for(int i=0; i<maxArguments; i++) {
            char tag = serialCLI.Indexs[i];
            double val = serialCLI.Arguments[i];

            if(tag >= 'a' && tag <= 'c') { // steppers: collect for the synced profile
                useAxis[tag - 'a'] = true;
                targetDeg[tag - 'a'] = (float)val;
            }
            else if(tag != ' ' && tag != 0) { // servos (d/e): immediate, no speed API to sync
                robot.moveto(tag, (float)val);
            }
        }
        robot.movetoSync(useAxis, targetDeg);
    }
    else if (cmd == cmd_position) { // Report current position (machine interface: binary reply)
        robot.get_angles();
        serialCLI.sendingPackage((char)cmd, 'D', robot.angles, robot.limitSwitchMask());
    }
    else if (cmd == cmd_currentPos) { // Set current position without moving
        serialCLI.getArgument();
        for(int i=0; i<maxArguments; i++) {
            char tag = serialCLI.Indexs[i];
            double val = serialCLI.Arguments[i];

            if(tag != ' ' && tag != 0) {
                robot.setpos(tag, (float)val);
            }
        }

    }
    else if (cmd == cmd_moveref) {  // Calibrate to reference position
        for(int i=0; i<maxArguments; i++) {
            serialCLI.writeArgument(i, -1.0, serialCLI.indexsList[i]); // Clear arguments
        }
        robot.refCalibrate( serialCLI.readNode() != commands::cmd_abort);
    }
    else if (cmd == cmd_grip) {
        serialCLI.sendingPackage((char)cmd, 'P', robot.angles, robot.limitSwitchMask());
        robot.moveto('e', gripClose);
        serialCLI.sendingPackage((char)cmd, 'D', robot.angles, robot.limitSwitchMask());
    }
    else if (cmd == cmd_release) {
        serialCLI.sendingPackage((char)cmd, 'P', robot.angles, robot.limitSwitchMask());
        robot.moveto('e', gripOpen);
        serialCLI.sendingPackage((char)cmd, 'D', robot.angles, robot.limitSwitchMask());
    }
    else if (cmd == cmd_abort){

    }
    robot.posWriting = 0;
    serialCLI.clearArgument();
}

bool ifspin(){
    if (millis() - preMillis >= 1000.0 * (1/(float)TOPIC_FREQ)){
        preMillis = millis();
        return 1;
    }
    else return 0;
}

void topicPrint(){
    robot.get_angles();
    serialCLI.sendingPackage((char)currentCommand, getStateID(), robot.angles, robot.limitSwitchMask());
    //ComPort.print(robot.turnSW_A(digitalRead(refA)));
    //ComPort.print(robot.turnSW_B(digitalRead(refB)));;
    //ComPort.println(robot.turnSW_C(digitalRead(refC)));
    //robot.debug();
}
 
char getStateID(){
    robot.get_angles();
    serialCLI.getArgument(); 

    if (errorFlag != error_none) {
        return 'F';
    }
    else if (!robot.ifRun()) return 'D';
    else return 'P';    
}

//interrupt service routines

void refA_ISR(){
    if(!debugMode)
        robot.turnSW_A(0);
}   
void refB_ISR(){
    if(!debugMode)
        robot.turnSW_B(0);
}
void refC_ISR(){
    if(!debugMode)
        robot.turnSW_C(0);
}