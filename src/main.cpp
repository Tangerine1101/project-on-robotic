#include<config.h>


void setup(){
  Serial.begin(9600);

  //stepper setup
  AccelStepper joint1(AccelStepper::DRIVER, pul1, dir1);
  AccelStepper joint2(AccelStepper::DRIVER, pul2, dir2);
  AccelStepper joint3(AccelStepper::DRIVER, pul3, dir3);
  AccelStepper joint4(AccelStepper::DRIVER, pul4, dir4);
  joint1.setAcceleration(acceleration);
  joint1.setMaxSpeed(maxSpeed);
  joint2.setAcceleration(acceleration);
  joint2.setMaxSpeed(maxSpeed);
  joint3.setAcceleration(acceleration);
  joint3.setMaxSpeed(maxSpeed);
  joint4.setAcceleration(acceleration);
  joint4.setMaxSpeed(maxSpeed);

}

void loop(){
 if(command() == POSITION) Serial.println("success");

}
