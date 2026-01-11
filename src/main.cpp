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
char getCommandID();
//interrupt service routines
void runCallback();
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
    pinMode(refVolt, OUTPUT);
    digitalWrite(refVolt, 1);
    //setup interrupt/timer
    setupTimer6(CALLBACK_TIME); //setup timer with CALLBACK_TIME microsecond interval
    pinMode(refA, INPUT_PULLUP);
    pinMode(refB, INPUT_PULLUP);
    pinMode(refC, INPUT_PULLUP);
    attachInterrupt(refA, refA_ISR, FALLING);
    attachInterrupt(refB, refB_ISR, FALLING);
    attachInterrupt(refC, refC_ISR, FALLING);
    
    if (HumanInterface) {
        ComPort.println("System Initialized.");
    }
    preMillis = millis();
}

void loop() {
    operate();
    if (errorFlag == error_limitation_breaked){

    }
    if (ifspin()){
        topicPrint();
        if (errorFlag != error_none){
            serialCLI.sendingPackage('E', errorFlag + '0', robot.angles);
            if (robot.safety_check('a'))
            robot.jointBrake('a');
            if (robot.safety_check('b'))
            robot.jointBrake('b');
            if (robot.safety_check('c'))
            robot.jointBrake('c');

            ComPort.print(robot.safety_check('a'));
            ComPort.print(robot.safety_check('b'));
            ComPort.println(robot.safety_check('c'));
            errorFlag = error_none;
        }
    }
}


void operate() {
    // 1. Check for new commands
    commands cmd; // Returns enum
    if (HumanInterface){
        cmd = serialCLI.commandHandle();
    }
    else {
        cmd = serialCLI.readNode();
    }

    if (cmd != commands::cmd_none || getStateID() == 'D' || currentCommand == commands::cmd_moveref || currentCommand == commands::cmd_abort)
    currentCommand = cmd;
    // 2. Process Command
    robot.posWriting = 1;
    if (cmd == cmd_move) { // Relative move command
        // Get the parsed data
        serialCLI.getArgument(); 
        
        // Loop through the arguments (max 4)
        for(int i=0; i<maxArguments; i++) {
            char tag = serialCLI.Indexs[i];    // e.g., 'a'
            double val = serialCLI.Arguments[i]; // e.g., 90.0
            
            if(tag != ' ' && tag != 0) {
                robot.move(tag, (float)val);
                if (HumanInterface) {
                    ComPort.print("Relative Move: "); 
                    ComPort.print(tag); 
                    ComPort.println(val);
                }
            }
        }
    }
    else if (cmd == cmd_moveto) { // Absolute move command
        serialCLI.getArgument(); 
        for(int i=0; i<maxArguments; i++) {
            char tag = serialCLI.Indexs[i];
            double val = serialCLI.Arguments[i];
            
            if(tag != ' ' && tag != 0) {
                robot.moveto(tag, (float)val);
            }
        }
    }
    else if (cmd == cmd_position) { // Report current position
        robot.reportPosition();
    }
    else if (cmd == cmd_currentPos) { // Set current position without moving
        serialCLI.getArgument(); 
        for(int i=0; i<maxArguments; i++) {
            char tag = serialCLI.Indexs[i];
            double val = serialCLI.Arguments[i];
            
            if(tag != ' ' && tag != 0) {
                robot.setpos(tag, (float)val);
                if (HumanInterface) {
                    ComPort.print("Set Position: "); 
                    ComPort.print(tag); 
                    ComPort.println(val);
                }
            }
        }

    }
    else if (cmd == cmd_moveref) {  // Calibrate to reference position
        for(int i=0; i<maxArguments; i++) {
            serialCLI.writeArgument(i, -1.0, serialCLI.indexsList[i]); // Clear arguments
        }
        robot.refCalibrate( serialCLI.readNode() != commands::cmd_abort && serialCLI.commandHandle() != commands::cmd_abort);
    }
    else if (cmd == cmd_grip) {
        serialCLI.sendingPackage((char)cmd, 'P', robot.angles);
        robot.moveto('e', gripClose);
        serialCLI.sendingPackage((char)cmd, 'D', robot.angles);
    }
    else if (cmd == cmd_release) {
        serialCLI.sendingPackage((char)cmd, 'P', robot.angles);
        robot.moveto('e', gripOpen);    
        serialCLI.sendingPackage((char)cmd, 'D', robot.angles);
    }
    
    if (errorFlag == error_limitation_breaked) errorFlag = error_none;
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
    serialCLI.sendingPackage((char)currentCommand, getStateID(), robot.angles);
}
 
char getCommandID(){
    static char cmdID;
    if (cmdID == 0|| currentCommand != '~' || currentCommand == commands::cmd_moveref || getStateID() != 'P' || currentCommand != commands::cmd_abort) 
        cmdID = currentCommand;

    return cmdID;

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
    robot.jointBrake('a');
}   
void refB_ISR(){
    robot.jointBrake('b');
}
void refC_ISR(){
    robot.jointBrake('c');
}