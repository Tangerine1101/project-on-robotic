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
    uint8_t checksum;

    sendPackage(){
        startByte = NODE_SENDBYTE;
        processingID = '~';
        statusID = '~';
        for (int i = 0; i < maxArguments; i++) {
            Arguments[i] = 0.0; 
        }
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
    cmd_humanInterface = 'H', // For human convinience
    cmd_ros2Interface = 'S', // if subscribed, stream joints and grip angles constantly
    cmd_abort = 'X', // Emergency stop
    cmd_invalid = '&'            
} commands;

class serialCom {
    public:
        
        commands commandHandle();
        commands readNode();
        serialCom();
        void readFrom(unsigned int pos, String Command);
        void clearArgument();
        void getArgument();
        void writeArgument(int index, float value, char tag);
        float Arguments[maxArguments];
        char Indexs[maxArguments];
        void packageDebug();
        void sendingPackage(char processingID, char statusID, float args[maxArguments]);
        uint8_t checksumXOR(uint8_t* data, size_t length);
        const char indexsList[maxArguments] = {'a', 'b', 'c', 'd', 'e'};
    private:
        serialPackage pkgDeg;
        String incomingCommand = "";
        float privateArg[maxArguments];
        char privateIndex[maxArguments];
        bool verifyChecksum(const serialPackage& pkg);
    };

#endif