#include "config.h"
#include <Arduino.h>
Serial_& ComPort = SerialUSB;
UARTClass& ProgramPort = Serial;

//config tb6600
//pinout
const int dir1 = 22; //stepper 1 pinout
const int pul1= 23;
const int dir2 = 24; //stepper 2 pinout
const int pul2= 25;
const int dir3 = 26; //stepper 3 pinout
const int pul3= 27;
const int servo4 = 28;//servo 4 - joint 4
const int servo5 = 29;//servo 5 - the grip
const int refA= 30; //reference switch for joint 1
const int refB = 32; //reference switch for joint 2
const int refC = 34; //reference switch for joint 3
//const int refD = 36; //reference switch for joint 4
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
const long  joint4Min =0;
const long  joint4Max = 180;
const long  gripMin =0;
const long  gripMax =180;
bool HumanInterface = 0;