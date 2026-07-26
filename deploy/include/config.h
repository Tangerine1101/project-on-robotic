#ifndef CONFIG_H
#define CONFIG_H  
#include <Arduino.h>

inline bool debugMode = 0; // 1: enable debug mode, 0: disable debug mode, currently only used in refCalibrate function to disable movement
//system default config
#define SAMPLE_TIME 1000 //millisecond
#define TOPIC_FREQ 20 //Hz
#define CALLBACK_TIME 20 //microsecond, shouldn't be too small, otherwise the program will be stuck in the callback function
#define TIMEOUT_LIMIT 10000 //millisecond

//robot physical config
#define GEAR_RATIO 13.7 
#define STEP_PER_REV 200.0
#define MICRO_STEP  8.0
#define maxArguments  5 //maximum arguments
#define JOINT_SPEEDDOWN 0.5 //speed down factor, NO EFFECT
#define INTERFERENCE_OFFSET 5.0 //degree, to avoid interference between joint 2 and joint 3, sum of joint2 and joint3 must < 60 - offset 
// TODO: gripOpen == gripClose (148.0) means the gripper never physically
// opens or closes — measure the real open/close servo angles on hardware
// and update these two values (see doc/bug-report.md 1.4).
inline const float gripOpen = 60.0;
inline const float gripClose = 120.0;
// EXPERIMENTAL: when defined, joint4/grip servos are detached after each move
// (motorControl::moveto), so they stop drawing holding current and stop
// emitting their continuous refresh pulses when idle. The next move re-attaches
// them automatically. Leave undefined for the default always-attached behavior.
//#define DETACH_SERVO_AFTER_TASK
#define SERVO_SETTLE_MS 500 // open-loop settle time before detaching (ms)
//Serial communicate
#define BAUDRATE    115200
#define NODE_STARTBYTE  0xAA 
#define NODE_SENDBYTE   0xFE 
//Reference angles
#define REF_A   15.0
#define REF_B   60.0
#define REF_C   -20.0
#define REF_D   90.0
#define REF_E   gripOpen
#define HOME_A  0.0 
#define HOME_B  0.0
#define HOME_C  0.0
#define HOME_D  90.0
#define HOME_E  gripClose

//define how the drivers are connected
//trục a: chiều dương rời xa limit switch refA, xoay base CW
//trục b: chiều dương đẩy cánh tay về phía trước, đưa limit switch refB rời xa cơ cấu đòn bẩy nâng hạ cẳng tay.
//trục c: chiều dương nâng cơ cấu lên và tiến gần limit switch refC
//mô tả vị trí 3 switch: 
// switch A: nằm trên base, khi base xoay CCW sẽ đụng trúng nó
// switch B: nằm trên khâu bắp tay, khi bắp tay lùi về sau hoặc đòn bẩy của cánh tay tiến về trước sẽ có khả năng chạm vào. 
// switch C: nằm trên base, khi đòn bẩy cánh tay lùi về sau sẽ chạm.
#define COMMON_CATHODE  1
inline const int jointsDir[3] = {1,1,-1}; // Define direction for each joint, using in refCalibrate function
inline const int jointsRevDir[3] = {0,1,1}; // Define reverse direction for each joint
//define serial port, the program mainly use ComPort which in fact is programming port(Serial) on board, change it to serialUSB(native usb port) for faster Serial communication 
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
inline const float maxSpeed = 2*13.7*200*8;       // Steps per second, limit of this parameters is unclear due to the heaviness of the program, reduce callback time might improve this
inline const float acceleration = 2400;   // Steps per second squared 

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

//physical limited of each joint (IN STEP UNIT, NOT DEGREE)
#define CONST_LROUND(x) (x >= 0) ? (long)(x + 0.5) : (long)(x - 0.5);
inline constexpr float stepConvert = STEP_PER_REV*GEAR_RATIO*MICRO_STEP/360;
inline constexpr long joint1Min = CONST_LROUND( -90 * stepConvert);
inline constexpr long joint1Max = CONST_LROUND(REF_A * stepConvert);
inline constexpr long  joint2Min = CONST_LROUND(-80.0 * stepConvert);
inline constexpr long  joint2Max= CONST_LROUND(REF_B * stepConvert);
inline constexpr long  joint3Min = CONST_LROUND(REF_C * stepConvert);
inline constexpr long  joint3Max = CONST_LROUND(80 * stepConvert);
inline constexpr float  joint4Min = -0.001;
inline constexpr float  joint4Max = 180.001;
inline const float  gripMin = 50;
inline const float  gripMax = 130;


#endif
