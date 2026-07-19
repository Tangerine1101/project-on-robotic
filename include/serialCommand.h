#ifndef SERIAL_COMMANDS
#define SERIAL_COMMANDS
#define POSITION    1
#define NONE    0

#include <Arduino.h>
#include "config.h"
extern char indexsList[maxArguments];
struct __attribute__((packed)) serialPackage //remote command package
{
    uint8_t startByte; //check if the first byte is correct
    char commandID; //the command character
    uint8_t bitmask; //5 bits to indicate which joints to move
    float Arguments[maxArguments]; //arguments for each joint
    uint8_t checksum; 
};

struct __attribute__((packed)) sendPackage //package that will be send to PC
{
    uint8_t startByte; //check if the first byte is correct
    char processingID; //the processing command character, refer to characters of enum commands(~, M, A, P, C, G, R, F, H, S, X, &, etc)
    char statusID; //status of the command: P(processing), D(done), F(fail)
    float Arguments[maxArguments]; //arguments for each joint
    uint8_t limitSwitches; //bitmask of the 3 reference switches: bit0=A(joint1), bit1=B(joint2), bit2=C(joint3). Raw polarity: 1=open(not touched), 0=at switch (matches limitSw_* / INPUT_PULLUP active-low).
    uint8_t checksum;

    sendPackage(){
        startByte = NODE_SENDBYTE;
        processingID = '~';
        statusID = '~';
        for (int i = 0; i < maxArguments; i++) {
            Arguments[i] = 0.0;
        }
        limitSwitches = 0;
        checksum = 0;
    }
};

typedef enum {
    cmd_none = '~',
    cmd_move = 'M', // Relative move
    cmd_moveto = 'A', // Absolute move
    cmd_position = 'P', // Report current position
    cmd_currentPos = 'C', // Set current position without moving
    cmd_grip = 'G',// Close the grip
    cmd_release = 'R', // Open the grip
    cmd_moveref = 'F', // Calibrate to reference position
    cmd_machineInterface = 'S', // machine interface: stream joints and grip angles constantly (binary packets)
    cmd_abort = 'X', // Emergency stop
    cmd_invalid = '&'
    // Add more commands as needed
} commands;
// note: status packets may also carry processingID 'E' = error report
// (statusID holds the error code as an ASCII digit, see errors enum in config.h)

class serialCom {
    public:

        commands readNode();
        serialCom();
        void clearArgument();
        void getArgument();
        void writeArgument(int index, float value, char tag);
        float Arguments[maxArguments];
        char Indexs[maxArguments];
        void packageDebug();
        static void sendingPackage(char processingID, char statusID, float args[maxArguments], uint8_t limitSwitches);
        static uint8_t checksumXOR(uint8_t* data, size_t length);
        const char indexsList[maxArguments] = {'a', 'b', 'c', 'd', 'e'};
    private:
        serialPackage pkgDeg;
        float privateArg[maxArguments];
        char privateIndex[maxArguments];
        bool verifyChecksum(const serialPackage& pkg);
    };

#endif