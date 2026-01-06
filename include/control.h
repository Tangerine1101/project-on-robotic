#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>
#include "config.h"

#define gripOpen    180
#define gripClose   90
#define refStep   2  //steps to move when searching for reference switch
// Mapping logic: 
// assume 'a' = Joint 1, 'b' = Joint 2, 'c' = Joint 3, 'd' = Joint 4, 'e' = Grip

class motorControl {
    public:
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
        // Essential system functions
        void run(); // Must be called in the main loop constantly!
        void reportPosition(); // Prints current angles to Serial
        bool safety_check(char axis, float angle);
        void get_angles();
        float angles[maxArguments]; // Store current angles of joints and grip
    private:
        // Helper to convert degrees to steps
        long angleToSteps(float angle);
        float stepsToAngle(long steps);
        // Define the 4 steppers using the interface type 1 (Driver)
        AccelStepper joint1;
        AccelStepper joint2;
        AccelStepper joint3;
        Servo joint4, grip;
        
};



#endif