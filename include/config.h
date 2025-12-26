#ifndef CONFIG
#include <Arduino.h>

//#include <AccelStepper.h>
//system default config
#define SAMPLE_TIME 10 //millisecond
#define GEAR_RATIO 13.7 
#define STEP_PER_REV 200
#define STEPPER_SPEED 60 //rpm
#define MICRO_STEP  2
//Serial communicate
#define BAUDRATE    115200
#define NODE_STARTBYTE  0x23 // '#'
#define NODE_SENDBYTE   0x40 // '@'
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
extern Serial_& ComPort;
extern UARTClass& ProgramPort;

//config tb6600
//pinout
extern const int dir1; //stepper 1 pinout
extern const int pul1;
extern const int dir2; //stepper 2 pinout
extern const int pul2;
extern const int dir3; //stepper 3 pinout
extern const int pul3;
extern const int servo4;//servo 4 - joint 4
extern const int servo5;//servo 5 - the grip
extern const int refA; //reference switch for joint 1
extern const int refB; //reference switch for joint 2
extern const int refC; //reference switch for joint 3
//extern const int refD; //reference switch for joint 4
extern const int refVolt; //Reference high signal that plug in driver

//stepper's parameters
extern const int stepsPerRevolution; // Typical steps for a 1.8 degree motor in full step
extern const float maxSpeed;       // Steps per second 
extern const float acceleration;   // Steps per second squared 
//physical limited of each joint (IN STEP UNIT, NOT DEGREE)
extern const double stepConvert;
extern const long joint1Min;
extern const long joint1Max;
extern const long  joint2Min;
extern const long  joint2Max;
extern const long  joint3Min;
extern const long  joint3Max;
extern const long  joint4Min;
extern const long  joint4Max; 
extern const long  gripMin;
extern const long  gripMax;
extern bool HumanInterface;
#endif
