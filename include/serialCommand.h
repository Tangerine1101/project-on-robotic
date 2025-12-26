#ifndef SERIAL_COMMANDS
#define SERIAL_COMMANDS
#define POSITION    1
#define NONE    0
#define maxArguments  5 //maximum arguments
#define NODE_STARTBYTE  0x23 // '#'
#define NODE_SENDBYTE   0x40 // '@'
#include <Arduino.h>
#include "config.h"
extern char indexsList[maxArguments];
struct __attribute__((packed)) serialPackage
{
    uint8_t startByte; //check if the first byte is correct
    char commandID; //the command character
    int8_t bitmask; //5 bits to indicate which joints to move
    float Arguments[maxArguments]; //arguments for each joint
    int8_t checksum; 
};

typedef enum {
    cmd_none,
    cmd_move, // Relative move
    cmd_moveto, // Absolute move
    cmd_position, // Report current position
    cmd_currentPos, // Set current position without moving
    cmd_grip,// Close the grip
    cmd_release, // Open the grip
    cmd_moveref, // Calibrate to reference position
    cmd_humanInterface, // For human convinience
    cmd_ros2Interface, // if subscribed, stream joints and grip angles constantly
    cmd_invalid            
} commands;

class serialCom {
    public:
        
        commands commandHandle();
        commands readNode();
        serialCom();
        void readFrom(unsigned int pos, String Command);
        void clearArgument();
        void getArgument();
        float Arguments[maxArguments];
        char Indexs[maxArguments];

    private:
        String incomingCommand = "";
        float privateArg[maxArguments];
        char privateIndex[maxArguments];
        bool verifyChecksum(const serialPackage& pkg);
    };

#endif