#include "control.h"

// Constructor: Connect pins from config.h
// Note: AccelStepper(interface, stepPin, dirPin)
motorControl::motorControl() :
    joint1(AccelStepper::DRIVER, pul1, dir1),
    joint2(AccelStepper::DRIVER, pul2, dir2),
    joint3(AccelStepper::DRIVER, pul3, dir3),
    joint4(AccelStepper::DRIVER, pul4, dir4)
{
}

void motorControl::init() {
    // Joint 1
    joint1.setMaxSpeed(maxSpeed);
    joint1.setAcceleration(acceleration);
    
    // Joint 2
    joint2.setMaxSpeed(maxSpeed);
    joint2.setAcceleration(acceleration);

    // Joint 3
    joint3.setMaxSpeed(maxSpeed);
    joint3.setAcceleration(acceleration);

    // Joint 4
    joint4.setMaxSpeed(maxSpeed);
    joint4.setAcceleration(acceleration);
}

// Helper: Convert degrees to steps
long motorControl::angleToSteps(float angle) {
    // Formula: (Angle / 360) * StepsPerRev * GearRatio * MicroStep
    return (long)((angle / 360.0) * STEP_PER_REV * GEAR_RATIO * MICRO_STEP);
}

// Helper: Convert steps back to degrees for reporting
float motorControl::stepsToAngle(long steps) {
    return (float)steps * 360.0 / (STEP_PER_REV * GEAR_RATIO * MICRO_STEP);
}

// RELATIVE MOVE: Move X degrees from current spot
void motorControl::move(char axis, float angle) {
    long steps = angleToSteps(angle);
    
    switch(axis) {
        case 'a': joint1.move(steps); break;
        case 'b': joint2.move(steps); break;
        case 'c': joint3.move(steps); break;
        case 'd': joint4.move(steps); break;
        default: Serial.println("Error: Invalid Axis"); break;
    }
}

// ABSOLUTE MOVE: Move to specific angle X
void motorControl::moveto(char axis, float angle) {
    long steps = angleToSteps(angle);

    switch(axis) {
        case 'a': joint1.moveTo(steps); break;
        case 'b': joint2.moveTo(steps); break;
        case 'c': joint3.moveTo(steps); break;
        case 'd': joint4.moveTo(steps); break;
        default: Serial.println("Error: Invalid Axis"); break;
    }
}

// Checks if motors need to step. CALL THIS OFTEN.
void motorControl::run() {
    joint1.run();
    joint2.run();
    joint3.run();
    joint4.run();
}

void motorControl::reportPosition() {
    Serial.print("Pos A: "); Serial.print(stepsToAngle(joint1.currentPosition()));
    Serial.print(" | Pos B: "); Serial.print(stepsToAngle(joint2.currentPosition()));
    Serial.print(" | Pos C: "); Serial.print(stepsToAngle(joint3.currentPosition()));
    Serial.print(" | Pos D: "); Serial.println(stepsToAngle(joint4.currentPosition()));
}