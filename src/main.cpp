#include "serialCommand.h"
#include "control.h"

// Instantiate objects
serialCom serialCLI;
motorControl robot;
commands currentCommand;
long preMillis;

void operate();
bool ifspin();
void topicPrint();
char getStateID();
char getCommandID();

void setup() {
    ComPort.begin(115200); // Make sure this matches your monitor
    robot.init();
    pinMode(refVolt, OUTPUT);
    digitalWrite(refVolt, 1);
    if (HumanInterface) {
        ComPort.println("System Initialized.");
    }
    preMillis = millis();
}

void loop() {
    operate();
    topicPrint();
    //serialCLI.packageDebug();
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
    if (cmd != commands::cmd_none || getStateID() != 'P')
    currentCommand = cmd;
    // 2. Process Command
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
                if (HumanInterface) {
                    ComPort.print("Absolute Move: "); 
                    ComPort.print(tag); 
                    ComPort.println(val);
                }
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
        robot.moveto('e', gripClose);
    }
    else if (cmd == cmd_release) {
        robot.moveto('e', gripOpen);    
    }
    // 3. CRITICAL: Actually drive the motors
    robot.run();
}

bool ifspin(){
    if (millis() - preMillis >= SAMPLE_TIME){
        preMillis = millis();
        return 1;
    }
    else return 0;
}

void topicPrint(){
    if (HumanInterface ==0 && ifspin())
    robot.get_angles();
    serialCLI.sendingPackage((char)currentCommand, getStateID(), robot.angles);
    if (getStateID() != 'P'){
        serialCLI.clearArgument();;
    }

}
char getCommandID(){
    static char cmdID;
    if (cmdID == 0|| currentCommand != '~' || getStateID() != 'P' || currentCommand != commands::cmd_abort) 
        cmdID = currentCommand;

    return cmdID;

}
char getStateID(){
    bool cons[maxArguments];
    bool condition = 1;
    robot.get_angles();
    serialCLI.getArgument(); 

    for(int i=0; i<maxArguments; i++) {
        if (serialCLI.Indexs[i] != ' ' && serialCLI.Indexs[i] != 0){
            cons[i] = abs(robot.angles[i] - serialCLI.Arguments[i]) <= 1.0;
        }
        else {
            cons[i] = 1;
        }
        condition = condition && cons[i];
    }
    if (currentCommand == commands::cmd_moveref) return 'D';
    if (timeoutFlag) return 'F';
    else if (condition) return 'D';
    else return 'P';
    
}