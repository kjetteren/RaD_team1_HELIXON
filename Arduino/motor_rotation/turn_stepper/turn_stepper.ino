#include <Stepper.h>

const int stepsPerRev = 2048;
const int speed = 10;

Stepper myStepper = Stepper(stepsPerRev, 8, 10, 9, 11);

void setup() {
}

void loop() {
	myStepper.setSpeed(speed);
	myStepper.step(stepsPerRev/4);
	delay(1000);
	
	myStepper.setSpeed(speed);
	myStepper.step(-stepsPerRev/4);
	delay(15000);
}