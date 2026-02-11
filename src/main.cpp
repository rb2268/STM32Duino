#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>

// Pin Definitions
const int SERVO_30_PIN = A2; 
const int SERVO_10_PIN = D9;

//Default angle is set to 90 so it can be adjusted in the positive and negative directions
const int angle = 90;

Servo servo30;
Servo servo10;

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

void setIMUTest();
void positionServos(int xServo, int yServo, int delay);
float calcRoll(float accelX, float accelY, float accelZ);
float calcPitch(float accelX, float accelY, float accelZ);
float calcAngVel(char axis, sensors_event_t &g);

void setup() {
  Serial.begin(115200);
  Serial.println("--- Nano Servo Initializing ---");

  // Attach servos to the pins
  servo30.attach(SERVO_30_PIN);
  servo10.attach(SERVO_10_PIN);

  // Set initial position to 0
  servo30.write(angle);
  servo10.write(angle);

  setIMUTest();
  
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


  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  float x_rot_vel = calcAngVel( 'x', g);
  float y_rot_vel = calcAngVel( 'y', g);
  float z_rot_vel = calcAngVel( 'z', g);

  float pitch = calcPitch(a.acceleration.x, a.acceleration.y, a.acceleration.z);
  float roll = calcRoll(a.acceleration.x, a.acceleration.y, a.acceleration.z);

  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print(" Roll: "); Serial.println(roll);

  delay(pauseTime);
}

/*

*/
void setIMUTest() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // Define I2C on PB6/PB7
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();

  delay(2000); 
  Serial.println("\n--- Starting LSM9DS1 Test ---");

  // Try to initialize
  if (!lsm.begin()) {
    Serial.println("CRITICAL: LSM9DS1 not found at PB6/PB7!");
    Serial.println("Check: 1. SCL is PB6, SDA is PB7. 2. Sensor has 3.3V.");
  } else {
    Serial.println("SUCCESS: Sensor Connected!");
  }
}

float calcRoll(float accelX, float accelY, float accelZ) {
  return atan2(-accelX, sqrt(sq(accelY) + sq(accelZ))) * 180.0 / PI;
}

float calcPitch(float accelX, float accelY, float accelZ) {
  return atan2(accelY, accelZ) * 180.0 / PI;
}

float calcAngVel(char axis, sensors_event_t &g) {
  if(axis == 'x' or axis == 'X') {
    return (g.gyro.x);
  }
  else if (axis == 'y' or axis == 'Y') {
    return (g.gyro.y);
  }
  else if (axis == 'Z' or axis == 'z') {
    return (g.gyro.z);
  }
  return -1.0;
}