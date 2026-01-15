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
    // Attach servos to their pins
    joint4.attach(servo4);
    grip.attach(servo5);
    motorControl::init();
    grip.write(gripOpen);
    joint4.write(HOME_D);
}

void motorControl::init() { // Initialize motor parameters2
    // Joint 1
    joint1.setPinsInverted(jointsRevDir[0],COMMON_CATHODE,0);
    joint1.setMaxSpeed(maxSpeed);
    joint1.setAcceleration(acceleration);
    joint1.setMinPulseWidth(10);
    // Joint 2
    joint2.setPinsInverted(jointsRevDir[1],COMMON_CATHODE,0);
    joint2.setMaxSpeed(maxSpeed);
    joint2.setAcceleration(acceleration);
    joint2.setMinPulseWidth(10);
    // Joint 3
    joint3.setPinsInverted(jointsRevDir[2],COMMON_CATHODE,0);
    joint3.setMaxSpeed(maxSpeed);
    joint3.setAcceleration(acceleration);
    joint3.setMinPulseWidth(10);
    // Joint 4    
}


bool motorControl::run() {  // motor run callback function. 
    if(calibrating){
        if (limitSw_A || joint1.distanceToGo() < 0)
            joint1.run();
        if (limitSw_B || joint2.distanceToGo() < 0)
            joint2.run();
        if ((limitSw_C || joint3.distanceToGo() > 0) && (limitSw_B || joint3.distanceToGo() < 0))
            joint3.run();
        return 1;
    }
    if (posWriting){
        return 1;
    }
    if (!JOINT_2_3_LIMIT_targetpos){
        errorFlag = error_limitation_breaked;
        joint1.stop();
        joint2.stop();
        joint3.stop();
        return 0;
        
    }
        if (JOINT1_LIMITATION){ // joint1: must not move counter clockwise when switch A is pressed
            bool ifMoveCW = joint1.distanceToGo() < 0;
            if (ifMoveCW || limitSw_A) 
                joint1.run();
        }
        if (JOINT2_LIMITATION){ // joint2: must not move counter clockwise when switch B is pressed
            bool ifMoveCW = joint2.distanceToGo() < 0;
            bool safeToMoveCCW = JOINT_2_3_LIMIT_currentpos && limitSw_B;
            if (ifMoveCW || safeToMoveCCW)
                joint2.run();
        }  
        if (JOINT3_LIMITATION) {// joint3: must not move clockwise when switch B is pressed, and must not move counter clockwise when switch C is pressed
            bool ifMoveCW = joint3.distanceToGo() > 0;
            bool safeToMove_CW = ifMoveCW && (limitSw_B && JOINT_2_3_LIMIT_currentpos);
            bool safeToMove_CCW = !ifMoveCW && (limitSw_C && JOINT_2_3_LIMIT_currentpos);
            if (safeToMove_CCW || safeToMove_CW)
                joint3.run();    
        }
    return 1;
}
// Helper: Convert steps back to degrees for reporting
float motorControl::servoAngle(Servo joint){
    float pulse = joint.readMicroseconds();
    float angle = (pulse - 544.0) * (180.0 - 0.0) / (2400.0 - 544.0);
    return angle;
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

volatile bool motorControl::safety_check(char axis){
    switch(axis) {
        case 'a':
            return JOINT1_LIMITATION;
            break;
        case 'b':
            return JOINT2_LIMITATION && JOINT_2_3_LIMIT_targetpos;
            break;
        case 'c':
            return JOINT3_LIMITATION && JOINT_2_3_LIMIT_targetpos;
            break;
        case 'd':
            if (servoAngle(joint4) <= joint4Max && servoAngle(joint4) >= joint4Min) {
                return true;
            }
            break;
        case 'e':
            if (servoAngle(grip) <= gripMax && servoAngle(grip) >= gripMin) {
                return true;
            }
            break;
        default:
            errorFlag = error_invalid_axis;
            break;
    }
    return false;
}
// RELATIVE MOVE: Move X degrees from current spot
void motorControl::move(char axis, float angle) {
    long steps = angleToSteps(angle);
    
    switch(axis) {
        case 'a': joint1.move(steps); break;
        case 'b': joint2.move(steps); break;
        case 'c': joint3.move(steps); break;
        case 'd': break;
        case 'e': break;
        default: errorFlag= error_invalid_axis; break;
    }
}

// ABSOLUTE MOVE: Move to specific angle X
void motorControl::moveto(char axis, float angle) {
    long steps = angleToSteps(angle);

    switch(axis) {
        case 'a': joint1.moveTo(steps); break;
        case 'b': joint2.moveTo(steps); break;
        case 'c': joint3.moveTo(steps); break;
        case 'd': 
            if (angle <= joint4Max && angle>= joint4Min) joint4.write(angle); 
            delay(15); 
            break;
        case 'e': 
            if ((angle <= gripMax) && (angle >= gripMin)) grip.write(angle); 
            delay(15); 
            break;
        default: errorFlag= error_invalid_axis; break;
    }
}

void motorControl::setpos(char axis, float angle){
    long steps = angleToSteps(angle);
    switch(axis) {
        case 'a': joint1.setCurrentPosition(steps); break;
        case 'b': joint2.setCurrentPosition(steps); break;
        case 'c': joint3.setCurrentPosition(steps); break;
        case 'd': break;
        case 'e': break;
        default: ComPort.println("[ERROR] Invalid Axis"); break;

    }
}

void motorControl::refCalibrate(bool interrupt){
    calibrating = true;  
    float ref[maxArguments]={REF_A, REF_B, REF_C, REF_D, REF_E}; 
    if (HumanInterface){
        ComPort.println("[PROCESSING] Calibrating...");
    }
    else {
        serialCom::sendingPackage('F','P', ref);
    }
    // phase 1: calibrate joint 2 and 3
    unsigned long timeout_check = millis();
    joint2.move(angleToSteps(jointsDir[1]*360));
    joint3.move(angleToSteps(jointsDir[2]*360));
    while ((digitalRead(refB) || digitalRead(refC)) && interrupt && !debugMode){
        turnSW_A(digitalRead(refA));
        turnSW_B(digitalRead(refB));
        turnSW_C(digitalRead(refC));
        if (millis() - timeout_check >= TIMEOUT_LIMIT){
            serialCom::sendingPackage('F','F', ref);
            errorFlag = error_timeout;
            break;
        }

    }
    // phase 2: calibrate joint 1
    timeout_check = millis();
    joint1.move(angleToSteps(jointsDir[0]*360));
    while(digitalRead(refA) && !debugMode){
        turnSW_A(digitalRead(refA));
        turnSW_B(digitalRead(refB));
        turnSW_C(digitalRead(refC));
        if (millis() - timeout_check >= TIMEOUT_LIMIT){
            serialCom::sendingPackage('F','F', ref);
            errorFlag = error_timeout;
            break;
        }
    }

    noInterrupts(); 
    joint1.setCurrentPosition(angleToSteps(REF_A));
    joint2.setCurrentPosition(angleToSteps(REF_B));
    joint3.setCurrentPosition(angleToSteps(REF_C));
    
    joint1.moveTo(HOME_A);
    joint2.moveTo(angleToSteps(HOME_B));
    joint3.moveTo(angleToSteps(HOME_C));
    interrupts();
    joint4.write(HOME_D);
    grip.write(HOME_E);

    //phase 3: move to home position
    timeout_check = millis();
    while((joint1.currentPosition() != joint1.targetPosition() || joint2.currentPosition() != joint2.targetPosition() || joint3.currentPosition() != joint3.targetPosition()) && interrupt){
        turnSW_A(digitalRead(refA));
        turnSW_B(digitalRead(refB));
        turnSW_C(digitalRead(refC));
        if (millis() - timeout_check >= TIMEOUT_LIMIT){
            serialCom::sendingPackage('F','F', ref);
            errorFlag = error_timeout;
            break;
        }
    }
        
    calibrating = false;
    delay(1000);
    ComPort.flush();
    if(HumanInterface)
        ComPort.println("[DONE] Calibration done.");
    else 
        serialCom::sendingPackage('F','D', ref);

}

void motorControl::reportPosition() { //print joint & grip position to Serial, for human 
    ComPort.print("joint 1: "); ComPort.print(stepsToAngle(joint1.targetPosition()));
    ComPort.print(" | joint 2: "); ComPort.print(stepsToAngle(joint2.targetPosition()));
    ComPort.print(" | joint 3: "); ComPort.print(stepsToAngle(joint3.targetPosition()));
    ComPort.print(" | joint 4: "); ComPort.println(servoAngle(joint4));
    ComPort.print(" | grip: "); ComPort.println(servoAngle(grip));
}
void motorControl::angleTopic() { //print joint & grip position to Serial, for ros2 node specificfly
    ComPort.print(NODE_SENDBYTE); 
    ComPort.print(stepsToAngle(joint1.currentPosition()));ComPort.print(",");
    ComPort.print(stepsToAngle(joint2.currentPosition()));ComPort.print(",");
    ComPort.print(stepsToAngle(joint3.currentPosition()));ComPort.print(",");
    ComPort.print(servoAngle(joint4));ComPort.print(",");
    ComPort.println(servoAngle(grip));
}


void motorControl::get_angles(){
    angles[0] = stepsToAngle(joint1.currentPosition());
    angles[1] = stepsToAngle(joint2.currentPosition());
    angles[2] = stepsToAngle(joint3.currentPosition());
    angles[3] = joint4_callback;
    angles[4] = grip_callback;
}

bool motorControl::ifRun(){
    return (joint1.distanceToGo() != 0 || joint2.distanceToGo() != 0 || joint3.distanceToGo() != 0);
}

bool motorControl::jointBrake(char axis){
    if (axis == '0'){
        joint1.setCurrentPosition(joint1.currentPosition());
        joint2.setCurrentPosition(joint2.currentPosition());
        joint3.setCurrentPosition(joint3.currentPosition());
    }
    else{
        switch(axis) {
            case 'a': joint1.setCurrentPosition(joint1.currentPosition()); break;
            case 'b': joint2.setCurrentPosition(joint2.currentPosition()); break;
            case 'c': joint3.setCurrentPosition(joint3.currentPosition()); break;
            default: return 0; break;
        }
    }
    return 1;
}
void motorControl::debug(){
    ComPort.println( JOINT1_LIMITATION && (joint1.distanceToGo() > 0 || digitalRead(refA)));
}
volatile bool motorControl::avoidCollision(char axis){
    switch(axis) {
        case 'b': 
            if (joint2.distanceToGo() > 0 && !JOINT_2_3_LIMIT_currentpos) 
                return 0;
            break;
        case 'c': 
            if(joint3.distanceToGo() > 0 && !JOINT_2_3_LIMIT_currentpos) 
                return 0;
            break;
        default: return 1; break;
    }
    return 1;
}
bool motorControl::turnSW_A(bool val){
    return limitSw_A = val;
}
bool motorControl::turnSW_B(bool val){
    return limitSw_B = val;
}
bool motorControl::turnSW_C(bool val){
    return limitSw_C = val;
}
void motorControl::moveServo(){
    joint4.write(joint4_callback);
    grip.write(grip_callback);
}
//