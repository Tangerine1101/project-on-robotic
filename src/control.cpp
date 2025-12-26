#include "control.h"

// Constructor: Connect pins from config.h
// Note: AccelStepper(interface, stepPin, dirPin)
motorControl::motorControl() :
    joint1(AccelStepper::DRIVER, pul1, dir1),
    joint2(AccelStepper::DRIVER, pul2, dir2),
    joint3(AccelStepper::DRIVER, pul3, dir3),
    joint4(),
    grip()
{
    joint4.attach(servo4);
    grip.attach(servo5);
}

void motorControl::init() {
    // Joint 1
    joint1.setMaxSpeed(maxSpeed);
    joint1.setAcceleration(acceleration);
    joint1.setMinPulseWidth(20);
    joint1.setPinsInverted(JOINT1_DIR,COMMON_CATHODE,0);
    // Joint 2
    joint2.setMaxSpeed(maxSpeed);
    joint2.setAcceleration(acceleration);
    joint2.setMinPulseWidth(20);
    joint2.setPinsInverted(JOINT2_DIR,COMMON_CATHODE,0);
    // Joint 3
    joint3.setMaxSpeed(maxSpeed);
    joint3.setAcceleration(acceleration);
    joint3.setMinPulseWidth(20);
    joint3.setPinsInverted(JOINT3_DIR,COMMON_CATHODE,0);
    // Joint 4
    
}


void motorControl::moveJoint(long step){
    joint1.move(step);
    joint1.run();
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
        case 'd': break;
        default: ComPort.println("[ERROR] Invalid Axis"); break;
    }
}

// ABSOLUTE MOVE: Move to specific angle X
void motorControl::moveto(char axis, float angle) {
    long steps = angleToSteps(angle);
    if (!safety_check(axis, angle)){
        return;
    }
    switch(axis) {
        case 'a': joint1.moveTo(steps); break;
        case 'b': joint2.moveTo(steps); break;
        case 'c': joint3.moveTo(steps); break;
        case 'd': 
            if (angle <= joint4Max && angle >= joint4Min) joint4.write(angle); 
            delay(15); 
            break;
        case 'e': 
            if (angle <= gripMax && angle >= gripMin) grip.write(angle); 
            delay(15); 
            break;
        default: ComPort.println("[ERROR] Invalid Axis"); break;
    }
}

void motorControl::setpos(char axis, float angle){
    long steps = angleToSteps(angle);
    switch(axis) {
        case 'a': joint1.setCurrentPosition(steps); break;
        case 'b': joint2.setCurrentPosition(steps); break;
        case 'c': joint3.setCurrentPosition(steps); break;
        case 'd': break;
        default: ComPort.println("[ERROR] Invalid Axis"); break;

    }
}
void motorControl::refCalibrate(){

    ComPort.println("[PROCESSING] Calibrating...");
    while (!digitalRead(refA) && !digitalRead(refB) && !digitalRead(refC) && ComPort.available() ==0){
        joint1.move(jointsDir[0]*refStep);
        joint2.move(jointsDir[1]*refStep);
        joint3.move(jointsDir[2]*refStep);
        run();
    }

    ComPort.println("[DONE] Calibration done.");

}
// Checks if motors need to step. CALL THIS OFTEN.
void motorControl::run() {
    joint1.run();
    joint2.run();
    joint3.run();
}

void motorControl::reportPosition() { //print joint & grip position to Serial, for human 
    ComPort.print("joint 1: "); ComPort.print(stepsToAngle(joint1.currentPosition()));
    ComPort.print(" | joint 2: "); ComPort.print(stepsToAngle(joint2.currentPosition()));
    ComPort.print(" | joint 3: "); ComPort.print(stepsToAngle(joint3.currentPosition()));
    ComPort.print(" | joint 4: "); ComPort.println(joint4.read());
    ComPort.print(" | grip: "); ComPort.println(grip.read());
}
void motorControl::angleTopic() { //print joint & grip position to Serial, for ros2 node specificfly
    ComPort.print(NODE_SENDBYTE); 
    ComPort.print(stepsToAngle(joint1.currentPosition()));ComPort.print(",");
    ComPort.print(stepsToAngle(joint2.currentPosition()));ComPort.print(",");
    ComPort.print(stepsToAngle(joint3.currentPosition()));ComPort.print(",");
    ComPort.print(joint4.read());ComPort.print(",");
    ComPort.println(grip.read());
}
bool motorControl::safety_check(char axis, float angle){
    if(0){
        ComPort.println("Error: Safety check fail");
    }
    else{
        return 1;
    }
}
