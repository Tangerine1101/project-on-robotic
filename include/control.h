#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <AccelStepper.h>
#include "config.h"

// Mapping logic: 
// We assume 'a' = Joint 1, 'b' = Joint 2, 'c' = Joint 3, 'd' = Joint 4
// If you use different tags, update the switch cases in cpp.

class motorControl {
    public:
        motorControl();
        void init();
        void moveJoint(long step);
        // The core functions 
        void move(char axis, float angle);   // Relative move
        void moveto(char axis, float angle); // Absolute move
        void setpos(char axis, float angle);
        void refpos();
        // Essential system functions
        void run(); // Must be called in the main loop constantly!
        void reportPosition(); // Prints current angles to Serial
        bool safety_check(char axis, float angle);
    private:
        // Helper to convert degrees to steps
        long angleToSteps(float angle);
        float stepsToAngle(long steps);

        // Define the 4 steppers using the interface type 1 (Driver)
        AccelStepper joint1;
        AccelStepper joint2;
        AccelStepper joint3;
        AccelStepper joint4;
};

#endif