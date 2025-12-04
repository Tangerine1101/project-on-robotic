#include "serialCommand.h"
#include "control.h"

// Instantiate objects
serialCom serialCLI;
motorControl robot;

void operate();

void setup() {
    Serial.begin(115200); // Make sure this matches your monitor
    robot.init();
    pinMode(28, OUTPUT);
    digitalWrite(28, 1);
    //robot.moveJoint(-1500);
    Serial.println("System Ready. Waiting for commands...");
}

void loop() {
    operate();
}


void operate() {
    // 1. Check for new commands
    commands cmd = serialCLI.commandHandle(); // Returns enum

    // 2. Process Command
    if (cmd == cmd_move) {
        // Get the parsed data
        serialCLI.getArgument(); 
        
        // Loop through the arguments (max 4)
        for(int i=0; i<maxArguments; i++) {
            char tag = serialCLI.Indexs[i];    // e.g., 'a'
            double val = serialCLI.Arguments[i]; // e.g., 90.0
            
            // Only move if tag is valid (not space/null)
            if(tag != ' ' && tag != 0) {
                robot.move(tag, (float)val);
                Serial.print("Relative Move: "); Serial.print(tag); Serial.println(val);
            }
        }
        serialCLI.clearArgument(); // Cleanup
    }
    else if (cmd == cmd_moveto) {
        serialCLI.getArgument(); 
        for(int i=0; i<maxArguments; i++) {
            char tag = serialCLI.Indexs[i];
            double val = serialCLI.Arguments[i];
            
            if(tag != ' ' && tag != 0) {
                robot.moveto(tag, (float)val);
                Serial.print("Absolute Move: "); Serial.print(tag); Serial.println(val);
            }
        }
        serialCLI.clearArgument();
    }
    else if (cmd == cmd_position) {
        robot.reportPosition();
    }

    // 3. CRITICAL: Actually drive the motors
    // This needs to run as fast as possible, every loop cycle.
    robot.run();
}
