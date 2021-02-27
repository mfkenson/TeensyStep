Example using MotionPlanner2D

```
#include "TeensyStep.h"


Stepper motor1(2, 3);   //STEP pin =  2, DIR pin = 3
Stepper motor2(9,10);   //STEP pin =  9, DIR pin = 10

StepControl controller;
MotionPlanner2D mp;

void setup() {
  // setup the motors 
   motor1
    .setMaxSpeed(1000)       // steps/s
    .setAcceleration(2000); // steps/s^2 
  
    
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  mp.addPoint(1000,1000);
  mp.addPoint(2000,2000);
  mp.addPoint(3000,1000);
  mp.addPoint(4000,2000);
  mp.addPoint(5000,5000);
  mp.calculate(motor1, motor2);
  delay(100);
  Serial.println("-----start?-----");
  //motor1.setTargetRel(1000);  // Set target position to 1000 steps from current position
  //motor2.setTargetRel(1000);  // Set target position to 1000 steps from current position
  controller.move(motor1, motor2);    // Do the move
  Serial.println("-----end-----");
}

void loop() {}

```
