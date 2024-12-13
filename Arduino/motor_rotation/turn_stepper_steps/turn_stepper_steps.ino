#include <Stepper.h>

const int stepsPerRev = 2048;
const int steps5deg = stepsPerRev/72;
const int speed = 10;

Stepper myStepper = Stepper(stepsPerRev, 8, 10, 9, 11);

void setup() {
}

void loop() {
	myStepper.setSpeed(speed);
  for (int i=0; i<90/5; i++){
    myStepper.step(steps5deg);
    delay(500);
  }
	
  for (int i=0; i<90/5; i++){
    myStepper.step(-steps5deg);
    delay(500);
  }
	delay(10000);
}