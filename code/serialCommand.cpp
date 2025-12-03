#include"serialCommand.h"
serialCom::serialCom() {

}
// command format: 'command' -tag value -tag value ....
// ex: move -a 10 -b 20 -c 30
commands serialCom::commandHandle(){
    if (Serial.available() > 0){
        //read characters from Serial
        char incomingChar = Serial.read();
        incomingCommand += incomingChar;
        //when hit enter
        if (incomingChar == '\n' || incomingChar == '\r'){
            String Command = incomingCommand;
            incomingCommand = " ";
            Command.trim(); // get rid of spaces and terminators from begining/end
            Command.toLowerCase();
            
            //check for commands
            if (Command.startsWith("move ")){
                //detect argument and index
                readFrom(5, Command);
                return cmd_move;
            }   
            else if (Command.startsWith("moveto ")){
                readFrom(7, Command);
                return cmd_moveto;
            } 
            else if (Command.startsWith("position ")){
                
                return cmd_position;
            }
            else {
                return cmd_invalid;
            }
        }
    }
        return cmd_none;
}
void serialCom::readFrom(unsigned int pos, String Command){
    unsigned int startIndex = pos;
    unsigned int i = 0;
    int spaceIndex = -1;
    int hyphenIndex = -1;

    while(startIndex < Command.length() && i < maxArguments){ // Changed <= to < for safety
        // Find the hyphen
        hyphenIndex = Command.indexOf('-', startIndex);
        
        // Safety check: if no more hyphens, stop
        if(hyphenIndex == -1) break;

        // Find the space after the value (e.g., -a 10[space])
        spaceIndex = Command.indexOf(' ', hyphenIndex);
        
        // Look for the NEXT space to ensure we capture the full number if formatted like "-a 10 -b 20"
        // Actually, simpler logic: finding the space after the hyphen usually gives the tag, 
        // then finding the next space gives the value range.
        
        // Let's stick to your structure but fix the tag extraction:
        // Assume format: ... -a 10.0 ...
        
        // 1. Get the tag char (index after hyphen)
        if(hyphenIndex + 1 < Command.length()){
             Indexs[i] = Command.charAt(hyphenIndex + 1);
             commandIndex[i] = Indexs[i]; // Store internally too if needed
        }

        // 2. Find end of the number value
        // The number starts after the tag and a space usually? 
        // Or is it "-a10" or "-a 10"? 
        // Your parsing assumes "-a 10".
        
        int valueStartIndex = hyphenIndex + 2; // Skip '-' and 'a'
        spaceIndex = Command.indexOf(' ', valueStartIndex); // Find end of number
        
        if (spaceIndex == -1) {
            spaceIndex = Command.length(); // It's the last token
        }

        String valueStr = Command.substring(valueStartIndex, spaceIndex);
        valueStr.trim(); // Clean up just in case
        argument[i] = valueStr.toDouble(); 
        
        // Prepare for next loop
        startIndex = spaceIndex; 
        i++;
    }
}
void serialCom::clearArgument(){
    for (int i = 0; i < maxArguments; i++){
        argument[i] = 0.0;
        commandIndex[i] = ' ';
    }
}

void serialCom::getArgument(){
    for (int i =0; i < maxArguments; i++){
        Arguments[i] = argument [i];
        Indexs[i] = commandIndex[i];
    }
}