#include "serialCommand.h"
#include "control.h"

// Instantiate objects
serialCom serialCLI;
motorControl robot;
long preMillis;

void operate();
bool ifspin();
void topicPrint();

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
        serialCLI.clearArgument(); // Cleanup
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
        serialCLI.clearArgument();
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
        serialCLI.clearArgument();
    }
    else if (cmd == cmd_moveref) {  // Calibrate to reference position
        robot.refCalibrate();
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
    robot.angleTopic();
}