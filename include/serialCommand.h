#ifndef SERIAL_COMMANDS
#define SERIAL_COMMANDS
#define POSITION    1
#define NONE    0
#include <Arduino.h>

#define maxArguments  4 //maximum arguments
    typedef enum {
        cmd_move =1, 
        cmd_moveto =2,
        cmd_position =3,
        cmd_setpos =4,
        cmd_refpos =5,
        cmd_invalid =0,
        cmd_none
            
    } commands;

class serialCom {
    public:

        commands commandHandle();
        serialCom();
        void readFrom(unsigned int pos, String Command);
        void clearArgument();
        void getArgument();
        double Arguments[maxArguments];
        char Indexs[maxArguments];

    private:
        String incomingCommand = "";
        double argument[maxArguments];
        char commandIndex[maxArguments];
};
/*
 commands serialCom::commandHandle(){
    if (Serial.available() > 0){
        //read character from Serial sequentially 
        char incomingChar = Serial.read();
        incomingCommand += incomingChar;
        //if hit Enter
        if (incomingChar == '\n' || incomingChar == '\r' ){
            String Command = incomingCommand;
            incomingCommand = "";
            Command.trim(); // Get rid of leading/trailing spaces and terminators
            Command.toLowerCase();
            //check commands
                if (Command.startsWith("position")) {
                    
                unsigned int startIndex = 9; 
                int spaceIndex = -1;
                
                while (startIndex < Command.length()) {
                    // Find the start of the next token (should be a tag like -a)
                    spaceIndex = Command.indexOf(' ', startIndex);
                    if (spaceIndex == -1) {
                        spaceIndex = Command.length(); // If it's the last token
                    }

                    // Extract the tag (e.g., "-a")
                    String tag = Command.substring(startIndex, spaceIndex);
                    tag.trim();
                    startIndex = spaceIndex + 1; // Start next search after this space

                    // Check if a tag was found and if we have space for a value
                    if (tag.length() > 0 && startIndex < Command.length()) {
                        
                        // Find the space after the value
                        int valueSpaceIndex = Command.indexOf(' ', startIndex);
                        if (valueSpaceIndex == -1) {
                            valueSpaceIndex = Command.length();
                        }
                        
                        // Extract the value (e.g., "10")
                        String valueStr = Command.substring(startIndex, valueSpaceIndex);
                        valueStr.trim();
                        startIndex = valueSpaceIndex + 1; // Move past the value

                        double motorValue = valueStr.toDouble(); // Convert to number!

                        // ----------------------------------------------------
                        // Assign Value based on Tag (Don't mess this up!)
                        if (tag.equalsIgnoreCase("-a")) {
                            argument[0] = motorValue;
                        } else if (tag.equalsIgnoreCase("-b")) {
                            argument[1] = motorValue;
                        } else if (tag.equalsIgnoreCase("-c")) {
                            argument[2] = motorValue;
                        } 
                        // You can add more motors here...
                        // ----------------------------------------------------
                        checkCommand();
                        clearArgument();
                    }

                }

                    return position;
                }
        }
    }
    else return cmd_move;
}
*/
#endif