#ifndef CONFIG_H
#define CONFIG_H  
#include <Arduino.h>

//system default config
#define SAMPLE_TIME 100 //millisecond
#define GEAR_RATIO 13.7 
#define STEP_PER_REV 200
#define STEPPER_SPEED 60 //rpm
#define MICRO_STEP  2
#define maxArguments  5 //maximum arguments
//Serial communicate
#define BAUDRATE    115200
#define NODE_STARTBYTE  0x23 // '#'
#define NODE_SENDBYTE   '@' // '@'
//Reference angles
#define REF_A   0
#define REF_B   -20
#define REF_C   80
#define REF_D   0
#define REF_E   0

//define how the drivers are connected
#define COMMON_CATHODE  1
#define JOINT1_DIR   0
#define JOINT2_DIR   0
#define JOINT3_DIR   0
#define ProgramPort SerialUSB
#define ComPort Serial
inline bool timeoutFlag = 0;

//config tb6600
//pinout
inline const int dir1 = 22; //stepper 1 pinout
inline const int pul1= 23;
inline const int dir2 = 24; //stepper 2 pinout
inline const int pul2= 25;
inline const int dir3 = 26; //stepper 3 pinout
inline const int pul3= 27;
inline const int servo4 = 28;//servo 4 - joint 4
inline const int servo5 = 29;//servo 5 - the grip
inline const int refA= 30; //reference switch for joint 1
inline const int refB = 32; //reference switch for joint 2
inline const int refC = 34; //reference switch for joint 3
//const int refD = 36; //reference switch for joint 4
inline const int refVolt = 40; //Reference high signal that plug in driver

//stepper's parameters
inline const int stepsPerRevolution = 200; // Typical steps for a 1.8 degree motor in full step
inline const float maxSpeed = 2000;       // Steps per second 
inline const float acceleration = 1000;   // Steps per second squared 
//physical limited of each joint (IN STEP UNIT, NOT DEGREE)
inline const double stepConvert = STEP_PER_REV*GEAR_RATIO*MICRO_STEP/360;
inline const long joint1Min = 0 * stepConvert;
inline const long joint1Max = 360 * stepConvert;
inline const long  joint2Min = 0 * stepConvert;
inline const long  joint2Max= 360 * stepConvert;
inline const long  joint3Min =0 * stepConvert;
inline const long  joint3Max = 360 * stepConvert;
inline const long  joint4Min =0;
inline const long  joint4Max = 180;
inline const long  gripMin =0;
inline const long  gripMax =180;
inline bool HumanInterface = 0;
#endif
