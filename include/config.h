#ifndef CONFIG_H
#define CONFIG_H  
#include <Arduino.h>

inline bool HumanInterface = 1; // 1: human interface, 0: ROS2 interface
inline bool debugMode = 1; // 1: enable debug mode, 0: disable debug mode, currently only used in refCalibrate function to disable movement

//system default config
#define SAMPLE_TIME 1000 //millisecond
#define TOPIC_FREQ 5 //Hz
#define CALLBACK_TIME 20 //microsecond
#define TIMEOUT_LIMIT 10000 //millisecond

//robot physical config
#define GEAR_RATIO 13.7 
#define STEP_PER_REV 200.0
#define MICRO_STEP  8.0
#define maxArguments  5 //maximum arguments
#define JOINT_SPEEDDOWN 0.5 //speed down factor
#define INTERFERENCE_OFFSET 5.0 //degree, to avoid interference between joint 2 and joint 3, sum of joint2 and joint3 must < 60 - offset 
//Serial communicate
#define BAUDRATE    115200
#define NODE_STARTBYTE  0xAA 
#define NODE_SENDBYTE   0xFE 
//Reference angles
#define REF_A   0
#define REF_B   80
#define REF_C   -20
#define REF_D   0
#define REF_E   0

//define how the drivers are connected
#define COMMON_CATHODE  1
inline const int jointsDir[3] = {1,1,-1}; // Define direction for each joint, using in refCalibrate function
inline const int jointsRevDir[3] = {0,0,0}; // Define reverse direction for each joint
#define ProgramPort SerialUSB
#define ComPort Serial
//errors define
typedef enum{
    error_none = 0,
    error_invalid_axis = 1,
    error_limitation_breaked = 2,
    error_timeout = 3
} errors;
inline errors errorFlag = error_none; 
//stepper's parameters
inline const int stepsPerRevolution = 200; // Typical steps for a 1.8 degree motor in full step
inline const float maxSpeed = 13.7*200*8;       // Steps per second 
inline const float acceleration = 1200;   // Steps per second squared 

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
inline const int refA = 30; //reference switch for joint 1
inline const int refB = 32; //reference switch for joint 2
inline const int refC = 34; //reference switch for joint 3
inline const int refVolt = 40; //Reference high signal that plug in driver

//physical limited of each joint (IN STEP UNIT, NOT DEGREE)
#define CONST_LROUND(x) ((long)x +0.5)
inline constexpr double stepConvert = STEP_PER_REV*GEAR_RATIO*MICRO_STEP/360;
inline constexpr long joint1Min = CONST_LROUND(-175.0 * stepConvert);
inline constexpr long joint1Max = CONST_LROUND(175.0 * stepConvert);
inline constexpr long  joint2Min = CONST_LROUND(-70.0 * stepConvert);
inline constexpr long  joint2Max= CONST_LROUND(REF_B * stepConvert);
inline constexpr long  joint3Min = CONST_LROUND(REF_C * stepConvert);
inline constexpr long  joint3Max = CONST_LROUND(50 * stepConvert);
inline constexpr long  joint4Min =0;
inline constexpr long  joint4Max = 180;
inline constexpr long  gripMin =0;
inline constexpr long  gripMax =180;

#endif
