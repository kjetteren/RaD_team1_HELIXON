#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(9);
  myServo.write(0);  
  delay(500);        
}

void loop() {
  for (int angle = 0; angle <= 90; angle += 5) { 
    myServo.write(angle); 
    delay(500);           
  }
  for (int angle = 90; angle >= 0; angle -= 5) { 
    myServo.write(angle); 
    delay(500);           
  }
  delay(10000); 
}
