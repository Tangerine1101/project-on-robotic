#ifndef CONFIG
#include <serialCommand.h>
//#include <Arduino.h>
//#include <stepper_control.h>
#include <AccelStepper.h>
//system default config
#define SAMPLE_TIME 10 //millisecond
#define GEAR_RATIO 13.7 
#define STEP_PER_REV 200
#define STEPPER_SPEED 60 //rpm
#define MICRO_STEP  2
//Serial communicate
#define BAUDRATE    115200
//Reference angles
#define REF_A   20
#define REF_B   80
#define REF_C   0
#define REF_D   0


//config tb6600
//pinout
const int dir1 = 22; //stepper 1 pinout
const int pul1= 23;
const int dir2 = 24; //stepper 2 pinout
const int pul2= 25;
const int dir3 = 26; //stepper 3 pinout
const int pul3= 27;
const int dir4 = 28;//stepper 4 pinout
const int pul4= 29;
const int refA= 30; //reference switch for joint 1
const int refB = 32; //reference switch for joint 2
const int refC = 34; //reference switch for joint 3
const int refD = 36; //reference switch for joint 4
const int refVolt = 40; //Reference high signal that plug in driver

//stepper's parameters
const int stepsPerRevolution = 200; // Typical steps for a 1.8 degree motor in full step
const float maxSpeed = 2000;       // Steps per second 
const float acceleration = 1000;   // Steps per second squared 
//physical limited of each joint (IN STEP UNIT, NOT DEGREE)
const double stepConvert = STEP_PER_REV*GEAR_RATIO*MICRO_STEP/360;
const long joint1Min = 0 * stepConvert;
const long joint1Max = 360 * stepConvert;
const long  joint2Min = 0 * stepConvert;
const long  joint2Max= 360 * stepConvert;
const long  joint3Min =0 * stepConvert;
const long  joint3Max = 360 * stepConvert;
const long  joint4Min =0 * stepConvert;
const long  joint4Max = 360 * stepConvert;
#endif
