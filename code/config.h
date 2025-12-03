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
#define MICRO_STEP  1
//Serial communicate
#define BAUDRATE    115200

//config tb6600
//pinout
const int dir1 = 22;
const int pul1= 23;
const int dir2 = 24;
const int pul2= 25;
const int dir3 = 26;
const int pul3= 27;
const int dir4 = 28;
const int pul4= 29;
//stepper's parameters
const int stepsPerRevolution = 200; // Typical steps for a 1.8 degree motor in full step
const float maxSpeed = 2000;       // Steps per second 
const float acceleration = 100;   // Steps per second squared 
//physical limited of each joint
const float joint1Min = 0;
const float joint1Max = 360;
const float joint2Min =0;
const float joint2Max= 360;
const float joint3Min =0;
const float joint3Max = 360;
const float joint4Min =0;
const float joint4Max = 360;
#endif
