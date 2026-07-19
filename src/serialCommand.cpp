#include "serialCommand.h"

serialCom::serialCom() {

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
            case commands::cmd_machineInterface:
                return cmd_machineInterface;
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

void serialCom::sendingPackage(char processingID, char statusID, float args[maxArguments], uint8_t limitSwitches){
    sendPackage pkgToSend;
    pkgToSend.startByte = NODE_SENDBYTE;
    pkgToSend.processingID = processingID;
    pkgToSend.statusID = statusID;
    for (int i = 0; i < maxArguments; i++) {
        pkgToSend.Arguments[i] = args[i];
    }
    pkgToSend.limitSwitches = limitSwitches;
    // Calculate checksum (sizeof - 1 already covers the new limitSwitches byte)
    pkgToSend.checksum = checksumXOR((uint8_t*)&pkgToSend, sizeof(sendPackage) - 1);
    // Send package
    ComPort.write((uint8_t*)&pkgToSend, sizeof(sendPackage));
}

void serialCom::writeArgument(int index, float value, char tag){
    if (index >=0 && index < maxArguments){
        privateArg[index] = value;
        privateIndex[index] = tag;
    }
}