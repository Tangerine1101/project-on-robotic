#include"serialCommand.h"
char indexsList[maxArguments] = {'a','b','c','d','e'};
serialCom::serialCom() {

}
// command format: 'command' -tag value -tag value ....
// ex: move -a 10 -b 20 -c 30
commands serialCom::commandHandle(){
    if (ComPort.available() > 0){
        //read characters from Serial
        char incomingChar = ComPort.read();
        incomingCommand += incomingChar;
        //when hit enter
        if (incomingChar == '\n' || incomingChar == '\r'){
            String Command = incomingCommand;
            incomingCommand = "";
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
            else if (Command.startsWith("currentPos ")){
                readFrom(10, Command);
                return cmd_currentPos;
            }
            else if (Command.startsWith("grip ")){
                return cmd_grip;
            }
            else if (Command.startsWith("release ")){
                return cmd_release;
            }
            else if (Command.startsWith("moveref ")){
                return cmd_moveref;
            }
            else if (Command.startsWith("humanInterface ")){
                HumanInterface =1;
                return cmd_humanInterface;
            }
            else if (Command.startsWith("ros2Interface ")){
                HumanInterface =0;
                return cmd_ros2Interface;
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

    // Safety: Clear old arguments first so we don't get ghost values
    clearArgument(); 

    while(startIndex < Command.length() && i < maxArguments){ 
        // 1. Find the hyphen
        hyphenIndex = Command.indexOf('-', startIndex);
        if(hyphenIndex == -1) break; // No more tags

        // 2. Extract the Tag (e.g., 'a')
        if(hyphenIndex + 1 < Command.length()){
             Indexs[i] = Command.charAt(hyphenIndex + 1);
             privateIndex[i] = Indexs[i]; 
        }

        // 3. Find the Start of the Number
        // Start looking 2 chars after hyphen (skip '-' and tag)
        int valueStart = hyphenIndex + 2;
        
        // CRITICAL FIX: Skip any spaces between the tag and the number!
        while(valueStart < Command.length() && Command.charAt(valueStart) == ' ') {
            valueStart++;
        }

        // 4. Find the End of the Number (Next space)
        spaceIndex = Command.indexOf(' ', valueStart);
        
        // If no space found, the number goes to the end of the string
        if (spaceIndex == -1) {
            spaceIndex = Command.length(); 
        }

        // 5. Extract and Convert
        // Only try to convert if we actually have characters
        if (valueStart < spaceIndex) {
            String valueStr = Command.substring(valueStart, spaceIndex);
            privateArg[i] = valueStr.toDouble(); 
        }

        // Prepare for next loop
        startIndex = spaceIndex; 
        i++;
    }
}
void serialCom::clearArgument(){
    for (int i = 0; i < maxArguments; i++){
        privateArg[i] = 0.0;
        privateIndex[i] = ' ';
    }
}
void serialCom::getArgument(){
    for (int i =0; i < maxArguments; i++){
        Arguments[i] = privateArg [i];
        Indexs[i] = privateIndex[i];
    }
}

bool serialCom::verifyChecksum(const serialPackage& pkg) {
    uint8_t calcSum = 0;
    
    const uint8_t* ptr = (const uint8_t*)&pkg;

    for (int i = 0; i < sizeof(pkg) - 1; i++) {
        calcSum ^= ptr[i];
    }

    return (calcSum == pkg.checksum);
}

commands serialCom::readNode(){
    const uint8_t START_BYTE = NODE_STARTBYTE;
    if (ComPort.available() >= sizeof(serialPackage)) {
        // Read bytes into a buffer
        serialPackage pkg;
        if (ComPort.peek() != START_BYTE) {
            // Discard invalid byte
            ComPort.read();
            return cmd_invalid;
        }

        ComPort.readBytes((char*)&pkg, sizeof(serialPackage));
        
        // Verify checksum
        if (!verifyChecksum(pkg)) {
            return cmd_invalid;
        }

        // Copy arguments and indexes
        for (int i = 0; i < maxArguments; i++) {
            privateArg[i] = pkg.Arguments[i];
            // Assuming commandIndex is derived from bitmask or other means
            privateIndex[i] = indexsList[i]; // Example mapping
        }

        // Determine command type based on commandID
        switch (pkg.commandID) {
            case 'M':
                return cmd_move;
            case 'A':
                return cmd_moveto;
            case 'P':
                return cmd_position;
            case 'C':
                return cmd_currentPos;
            case 'G':
                return cmd_grip;
            case 'R':
                return cmd_release;
            case 'F':
                return cmd_moveref;
            default:
                return cmd_invalid;
        }
    }
    return cmd_none;
}