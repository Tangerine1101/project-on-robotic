#include "serialCommand.h"

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
            else if (Command.startsWith("position")){
                
                return cmd_position;
            }
            else if (Command.startsWith("currentPos ")){
                readFrom(10, Command);
                return cmd_currentPos;
            }
            else if (Command.startsWith("grip")){
                return cmd_grip;
            }
            else if (Command.startsWith("release")){
                return cmd_release;
            }
            else if (Command.startsWith("moveref")){
                return cmd_moveref;
            }
            else if (Command.startsWith("humanInterface")){
                HumanInterface =1;
                return cmd_humanInterface;
            }
            else if (Command.startsWith("ros2Interface")){
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
        unsigned int valueStart = hyphenIndex + 2;
        
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

    for (unsigned int i = 0; i < sizeof(pkg) - 1; i++) {
        calcSum ^= ptr[i];
    }

    return (calcSum == pkg.checksum);
}

uint8_t serialCom::checksumXOR(uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
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
        pkgDeg = pkg; // Store for debugging
        // Verify checksum
        if (!verifyChecksum(pkg)) {
            return cmd_invalid;
        }

        // Copy arguments and indexes
        for (int i = 0; i < maxArguments; i++) {
            privateArg[i] = pkg.Arguments[i];
            // Assuming commandIndex is derived from bitmask or other means
            if(pkg.bitmask & (0x01 << i))
            privateIndex[i] = indexsList[i]; // Example mapping
        }

        // Determine command type based on commandID
        switch (pkg.commandID) {
            case commands::cmd_move:
                return cmd_move;
            case commands::cmd_moveto:
                return cmd_moveto;
            case commands::cmd_position:
                return cmd_position;
            case commands::cmd_currentPos:
                return cmd_currentPos;
            case commands::cmd_grip:
                return cmd_grip;
            case commands::cmd_release:
                return cmd_release;
            case commands::cmd_moveref:
                return cmd_moveref;
            case commands::cmd_humanInterface:
                HumanInterface = 1;
                return cmd_humanInterface;
            case commands::cmd_ros2Interface:
                HumanInterface = 0;
                return cmd_ros2Interface;
            case commands::cmd_abort:
                return cmd_abort;
            default:
                return cmd_invalid;
        }
    }
    return cmd_none;
}

void serialCom::packageDebug() { //standalone debug function to print the last received package
    if (ComPort.available() >= sizeof(serialPackage)) {
        // Read bytes into a buffer
        serialPackage pkg;
        if (ComPort.peek() != NODE_STARTBYTE) {
            // Discard invalid byte
            ComPort.read();
            return;
        }

        ComPort.readBytes((char*)&pkg, sizeof(serialPackage));
        ComPort.print(pkg.startByte, HEX); ComPort.print(", ");
        ComPort.print(pkg.commandID); ComPort.print(", ");
        ComPort.print(pkg.bitmask, BIN); ComPort.print(", ");
        for (int i = 0; i < maxArguments; i++) {
            ComPort.print(pkg.Arguments[i]);
            if (i < maxArguments - 1) ComPort.print(", ");      
        }
        ComPort.print(", "); ComPort.print(pkg.checksum, HEX);
        ComPort.println();
    }

}

void serialCom::sendingPackage(char processingID, char statusID, float args[maxArguments]){
    sendPackage pkgToSend;
    pkgToSend.startByte = NODE_SENDBYTE;
    pkgToSend.processingID = processingID;
    pkgToSend.statusID = statusID;
    for (int i = 0; i < maxArguments; i++) {
        pkgToSend.Arguments[i] = args[i];
    }
    // Calculate checksum
    pkgToSend.checksum = checksumXOR((uint8_t*)&pkgToSend, sizeof(sendPackage) - 1);
    // Send package
    if (HumanInterface){
    ComPort.print((char)NODE_SENDBYTE); ComPort.print(",");
    ComPort.print((char)processingID); ComPort.print(",");
    ComPort.print((char)statusID); ComPort.print(",");
    for (int i = 0; i < maxArguments; i++) {
        ComPort.print(args[i]);
        if (i < maxArguments - 1) ComPort.print(",");      
    }
    ComPort.print(","); ComPort.println(pkgToSend.checksum, HEX);
    }
    else {
        ComPort.write((uint8_t*)&pkgToSend, sizeof(sendPackage));
    }
}

void serialCom::writeArgument(int index, float value, char tag){
    if (index >=0 && index < maxArguments){
        privateArg[index] = value;
        privateIndex[index] = tag;
    }
}