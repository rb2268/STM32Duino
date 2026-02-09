#include <Arduino.h>
#include <Servo.h>

// Pin Definitions
const int SERVO_30_PIN = A2; 
const int SERVO_10_PIN = D9;

//Default angle is set to 90 so it can be adjusted in the positive and negative directions
const int angle = 90;

Servo servo30;
Servo servo10;

void positionServos(int xServo, int yServo, int delay);

void setup() {
  Serial.begin(115200);
  Serial.println("--- Nano Servo Initializing ---");

  // Attach servos to the pins
  servo30.attach(SERVO_30_PIN);
  servo10.attach(SERVO_10_PIN);

  // Set initial position to 0
  servo30.write(angle);
  servo10.write(angle);
  
  delay(1000); 
  Serial.println("Setup Complete. Starting movement loop...");
}

void loop() {
  // Up Left
  positionServos(30, 8, 300);
  // Left Down
  positionServos(-30, 8, 300);
  // uP riGHT
  positionServos(30, -8, 300);
  // Down Right
  positionServos(-30, -8, 300);
}

/* 
 * Takes in an x and y angle input in degrees (range from -90 to 90 for proper values)

 * Input int pauseTime runs in ms (100 - 2000 is a good range)
 * 
 * Prints the servo positions to the serial monitor before moving
 * 
 * Outputs nothing (void)
*/
void positionServos(int xServo, int yServo, int pauseTime) {
  Serial.print("Moving x Servo: ");
  Serial.print(xServo);
  Serial.print(" degrees. Moving y Servo ");
  Serial.print(yServo);
  Serial.println(" degrees");
  servo30.write(angle + xServo);
  servo10.write(angle + yServo);
  delay(pauseTime);
}