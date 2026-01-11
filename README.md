
# upload code
This project originally write on platformIO, if you use arduino IDE:\
copy all files from "src" and "include" folders into a folder name "main", rename main.cpp to main.ino and open it with arduino ide then compile/upload

# list of commands and errors:
|Define|CommandID|CLI command (available in human interface mode)| Function
|------|---------|---------|----
|cmd_none            |~                       |
|cmd_move            |M                       |move [option1] angle [option2] angle ... [options5] angle| move joints that list as "option" a **relative** angle 
|cmd_moveto          |A                       |moveto [option1] angle [option2] angle ... [options5] angle | move joints that list as "option" to the **Absolute** angle
|cmd_position        |P                       |position| report joints angle(only available in humanInterface mode) 
|cmd_currentPos      |C                       |currentPos [option1] angle [option2] angle ... [options5] angle| set joints current angle to the desired angle 
|cmd_grip            |G| |close grip                   
|cmd_release         |R| |open grip         
|cmd_moveref         |F                       |moveref | calibrate by move until the arm reach limit switches(reference point)
|cmd_humanInterface  |H                       |humanInterface| turn on humanInterface mode
|cmd_ros2Interface   |S                       |ros2Interface| turn on ros2Interface mode
|cmd_abort           |S                       |abort| abort task
|cmd_invalid         |&
|error_none         | 0
|error_invalid_axis|1||the motors or joints that called in the command are invalid
|error_limitation_breaked|2|| the robot arm's limitation breaked
|error_timeout  |3|| task timeout

Difference between humanInterface and ros2Interface: 
- ros2Interface is a interface that tweaked for M2M(machine to machine) communication.
- humanInterface is for human manually monitor and debug. \
Note: package_receive.py is for convert M2M message into readable text

# M2M: package send and receive mechanism
## package that send from MCU(Arduino Due in this case) to PC is define as:
```cpp
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
```
- therefor, a sending package from MCU to PC is a string of [{start byte}, {processID}, {statusID}, {array of angles}, {checksum byte}]
## package that received by MCU that send from PC:
```cpp
struct __attribute__((packed)) serialPackage //remote command package
{
    uint8_t startByte; //check if the first byte is correct
    char commandID; //the command character
    uint8_t bitmask; //5 bits to indicate which joints to move
    float Arguments[maxArguments]; //arguments for each joint
    uint8_t checksum; 
};
```
- therefor, ros2 driver node(PC) will send a package to MCU as string of [{start byte}, {commandID}, {bitmask}, {array of angles}, {checksum byte}]

