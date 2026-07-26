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
    // NOTE: this constructor runs during C++ static init, BEFORE the Arduino
    // core's init() (global constructors run from the reset handler, before
    // main()). Servo pulse generation set up this early can fail to actually
    // run. init() is called again from setup() (post core-init) and re-attaches
    // the servos there -- that is the attach that matters. See init().
    motorControl::init();
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
    // Joint 4 / grip: (re)attach the servos here. init() is called from setup()
    // after the Arduino core is up, so this is the attach that reliably starts
    // servo pulse generation (the constructor's earlier call runs pre-core-init).
    if (!joint4.attached()) joint4.attach(servo4);
    if (!grip.attached())   grip.attach(servo5);
    joint4.write(HOME_D);
    grip.write(gripOpen);
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
// Helper: Convert steps back to degrees for reporting.
// NOTE: this is OPEN-LOOP. readMicroseconds() returns the last COMMANDED pulse,
// not the servo's real physical position (hobby servos give no feedback). So the
// reported joint4/grip angle -- and any PC-side "move done" check that compares
// it to the target -- confirms only that the command was issued, never that the
// horn actually got there. Keep that in mind when debugging servo motion.
// When DETACH_SERVO_AFTER_TASK is enabled and the servo is detached, this still
// returns the last commanded angle.
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
            if (angle <= joint4Max && angle >= joint4Min) {
            #ifdef DETACH_SERVO_AFTER_TASK
                if (!joint4.attached()) joint4.attach(servo4); // re-attach for this move
            #endif
                joint4.write(angle);
                delay(15);
            #ifdef DETACH_SERVO_AFTER_TASK
                delay(SERVO_SETTLE_MS); // let it reach position (open-loop) before cutting pulses
                joint4.detach();
            #endif
            }
            break;
        case 'e':
            if ((angle <= gripMax) && (angle >= gripMin)) {
            #ifdef DETACH_SERVO_AFTER_TASK
                if (!grip.attached()) grip.attach(servo5);
            #endif
                grip.write(angle);
                delay(15);
            #ifdef DETACH_SERVO_AFTER_TASK
                delay(SERVO_SETTLE_MS);
                grip.detach();
            #endif
            }
            break;
        default: errorFlag= error_invalid_axis; break;
    }
}

// Duration of a trapezoidal move of `distSteps` steps starting from rest with
// max speed V (steps/s) and acceleration A (steps/s^2).
float motorControl::profileTime(long distSteps, float V, float A) {
    float d = (float)labs(distSteps);
    if (d <= 0.0f)
        return 0.0f;
    if (d >= V * V / A)               // reaches cruise speed: trapezoid
        return d / V + V / A;
    return 2.0f * sqrtf(d / A);       // never reaches cruise: triangle
}

void motorControl::resetProfile() {
    AccelStepper* joints[3] = {&joint1, &joint2, &joint3};
    for (int i = 0; i < 3; i++) {
        joints[i]->setMaxSpeed(maxSpeed);
        joints[i]->setAcceleration(acceleration);
    }
}

// SYNCHRONIZED ABSOLUTE MOVE (joints 1-3): the slowest axis runs at the nominal
// profile; every other axis is stretched in time (V' = V/s, A' = A/s^2, which
// keeps the trapezoid shape and regime boundary V^2/A intact) so all commanded
// axes arrive at the same moment.
// Note: profileTime assumes the axes start from rest, so retargeting a moving
// arm gives only approximate sync -- the PC side's single-flight command queue
// never does that in normal operation.
void motorControl::movetoSync(const bool useAxis[3], const float targetDeg[3]) {
    AccelStepper* joints[3] = {&joint1, &joint2, &joint3};
    long targetSteps[3];
    float t[3];
    float T = 0.0f;
    for (int i = 0; i < 3; i++) {
        if (!useAxis[i])
            continue;
        targetSteps[i] = angleToSteps(targetDeg[i]);
        long dist = targetSteps[i] - joints[i]->currentPosition();
        t[i] = profileTime(dist, maxSpeed, acceleration);
        if (t[i] > T)
            T = t[i];
    }
    for (int i = 0; i < 3; i++) {
        if (!useAxis[i])
            continue;
        if (t[i] > 0.0f) {
            float s = T / t[i];       // >= 1; slowest axis gets s == 1 (nominal)
            joints[i]->setMaxSpeed(maxSpeed / s);
            joints[i]->setAcceleration(acceleration / (s * s));
        } else {                      // zero-distance axis: nominal profile
            joints[i]->setMaxSpeed(maxSpeed);
            joints[i]->setAcceleration(acceleration);
        }
        joints[i]->moveTo(targetSteps[i]);
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
        default: errorFlag = error_invalid_axis; break;
    }
}

void motorControl::refCalibrate(bool interrupt){
    resetProfile(); // a previous synced moveto may have left scaled-down speeds
    calibrating = true;
    float ref[maxArguments]={REF_A, REF_B, REF_C, REF_D, REF_E};
    serialCom::sendingPackage('F','P', ref, limitSwitchMask());
    // phase 1: calibrate joint 2 and 3
    unsigned long timeout_check = millis();
    joint2.move(angleToSteps(jointsDir[1]*360));
    joint3.move(angleToSteps(jointsDir[2]*360));
    while ((digitalRead(refB) || digitalRead(refC)) && interrupt && !debugMode){
        turnSW_A(digitalRead(refA));
        turnSW_B(digitalRead(refB));
        turnSW_C(digitalRead(refC));
        if (millis() - timeout_check >= TIMEOUT_LIMIT){
            serialCom::sendingPackage('F','F', ref, limitSwitchMask());
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
            serialCom::sendingPackage('F','F', ref, limitSwitchMask());
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
            serialCom::sendingPackage('F','F', ref, limitSwitchMask());
            errorFlag = error_timeout;
            break;
        }
    }
        
    calibrating = false;
    delay(1000);
    ComPort.flush();
    serialCom::sendingPackage('F','D', ref, limitSwitchMask());
}

void motorControl::get_angles(){
    angles[0] = stepsToAngle(joint1.currentPosition());
    angles[1] = stepsToAngle(joint2.currentPosition());
    angles[2] = stepsToAngle(joint3.currentPosition());
    angles[3] = servoAngle(joint4);
    angles[4] = servoAngle(grip);
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
// Compose the 3 switch states into one byte for the MCU->PC packet. Raw
// polarity (1=open/not-touched, 0=at switch): each limitSw_* bool is written
// atomically by its own turnSW_*/ISR, so reading them here needs no locking.
uint8_t motorControl::limitSwitchMask() const {
    return (uint8_t)(limitSw_A | (limitSw_B << 1) | (limitSw_C << 2));
}
//