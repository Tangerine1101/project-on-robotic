#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>
#include "config.h"
#include "serialCommand.h"

#define gripOpen    REF_E
#define gripClose   90
#define refStep   2  //steps to move when searching for reference switch
// Sum of joint 1 & 2 shouldn't exceed 60 degree to avoid interference
inline constexpr long  collisionLimit = CONST_LROUND((60 - INTERFERENCE_OFFSET)*stepConvert);
#define JOINT_2_3_LIMIT_targetpos (joint2.targetPosition() + joint3.targetPosition() < collisionLimit)
#define JOINT_2_3_LIMIT_currentpos (joint2.currentPosition() + joint3.currentPosition() < collisionLimit)
#define JOINT1_LIMITATION   (joint1.targetPosition() <= joint1Max && joint1.targetPosition() >= joint1Min)
#define JOINT2_LIMITATION    (joint2.targetPosition() <= joint2Max && joint2.targetPosition() >= joint2Min)
#define JOINT3_LIMITATION   (joint3.targetPosition() <= joint3Max && joint3.targetPosition() >= joint3Min)
// Mapping logic: 
// assume 'a' = Joint 1, 'b' = Joint 2, 'c' = Joint 3, 'd' = Joint 4, 'e' = Grip

class motorControl {
    public:
        bool posWriting = 0;
        motorControl();
        void init();
        // The core functions 
        void move(char axis, float angle);   // Relative move
        void moveto(char axis, float angle); // Absolute move
        void setpos(char axis, float angle);
        void gripPos(int angle);
        void movetoRef();
        void refCalibrate(bool interrupt);
        void angleTopic();
        bool ifRun();
        // Essential system functions
        bool run(); // Must be called in the main loop constantly!
        void reportPosition(); // Prints current angles to Serial
        void get_angles();
        float angles[maxArguments]; // Store current angles of joints and grip
        float servoAngle(Servo joint);
        bool jointBrake(char axis);
        volatile bool safety_check(char axis);
    private:
        // Helper to convert degrees to steps
        volatile bool avoidCollision(char axis);
        long angleToSteps(float angle);
        float stepsToAngle(long steps);
        bool calibrating;
        // Define the 4 steppers using the interface type 1 (Driver)
        AccelStepper joint1;
        AccelStepper joint2;
        AccelStepper joint3;
        Servo joint4, grip;
        
};



#endif