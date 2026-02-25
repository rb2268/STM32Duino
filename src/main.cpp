#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>

// Pin Definitions
const int SERVO_30_PIN = A2; 
const int SERVO_10_PIN = D9;

//Default angle is set to 90 so it can be adjusted in the positive and negative directions
const int angle = 90;

//For the PID constants
const float setpointAngle = 0.0;
struct PID {
  float kp, ki, kd, tau;
  float integral;
  float prevError;
  float derivative;
  unsigned long lastTime;
};
//Also for PID
float pitchOffset, rollOffset, pitchZero = 0, rollZero = 0;
float pitchFilt = 0.0f, rollFilt  = 0.0f;
unsigned long lastIMUTime = 0;

//Sets the kp, ki, kd, tau, ... in a more 'struct'ured way, get it!
PID pitchPID = {1.7, 0.05, 0.10, 0.3, 0, 0, 0, 0};
PID rollPID  = {1.2, 0.05, 0.10, 0.3, 0, 0, 0, 0};

Servo servo30;
Servo servo10;

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

void setIMUTest();
void positionServos(int xServo, int yServo, int delay);
float calcRoll(float accelX, float accelY, float accelZ);
float calcPitch(float accelX, float accelY, float accelZ);
float calcAngVel(char axis, sensors_event_t &g);
void calcOrientation(float wx, float wy, float wz);
// float outputPID(float imuAngle, float limMaxI);
// float calcPID(float error, float dt, float limMaxI);
float updatePID(PID &pid, float measurement, float setpoint, float limMaxI);

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

  delay(500);

  float pSum = 0, rSum = 0;
  for (int i = 0; i < 100; i++) {
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);

    pSum += calcPitch(a.acceleration.x, a.acceleration.y, a.acceleration.z);
    rSum += calcRoll (a.acceleration.x, a.acceleration.y, a.acceleration.z);
    delay(5);
  }

  pitchZero = pSum / 100.0f;
  rollZero  = rSum / 100.0f;

  Serial.print("Pitch zero: "); Serial.println(pitchZero);
  Serial.print("Roll zero: ");  Serial.println(rollZero);
  Serial.println("Setup Complete. Starting movement loop...");
}

void loop() {
  // Updates all the sensors
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  unsigned long now = millis();
  float dt = (now - lastIMUTime) / 1000.0f;
  lastIMUTime = now;

  //X-axis goes to pitch, Y-axis goes to roll
  float pitch = calcPitch(a.acceleration.x, a.acceleration.y, a.acceleration.z);
  float roll = calcRoll(a.acceleration.x, a.acceleration.y, a.acceleration.z);

  // Gyro rates (rad/s -> deg/s if needed)
  float gyroX = g.gyro.x * 180.0f / PI;
  float gyroY = g.gyro.y * 180.0f / PI;

  // Constant for filtering
  const float alpha = 0.98f;

  //Filters pitcha and roll (found online)
  pitchFilt = alpha * (pitchFilt + gyroX * dt) + (1.0f - alpha) * pitch;
  rollFilt  = alpha * (rollFilt  + gyroY * dt) + (1.0f - alpha) * roll;

  //Incase you want an offset value (not used)
  pitch -= pitchZero;
  roll  -= rollZero;
  //For pitch and roll axes
  pitch = 0.98f * (pitch + gyroX * dt) + 0.02f * pitch;
  roll  = 0.98f * (roll  + gyroY * dt) + 0.02f * roll;

  Serial.print("Pitch: "); Serial.print(pitch); 
  Serial.print(" Roll: "); Serial.println(roll);

  float pitchCorrection = updatePID(pitchPID, pitch, setpointAngle, 30);
  float rollCorrection  = updatePID(rollPID,  roll,  setpointAngle, 10);

  Serial.print("Pitch correction: "); Serial.println(pitchCorrection); 
  Serial.print("Roll Correction   "); Serial.println(rollCorrection); 
  positionServos(round(pitchCorrection), round(rollCorrection), 0);
  delay(8);  // control loop at ~100Hz
  // positionServos(0, 0, 100);
  Serial.print("ax: "); Serial.print(a.acceleration.x);
  Serial.print(" ay: "); Serial.print(a.acceleration.y);
  Serial.print(" az: "); Serial.println(a.acceleration.z);
  
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

  delay(pauseTime);
}

/*
 * 
 * Sets the IMU data to the two peripheral pins using I2C (SDA and SCL)
 * Will notify user in the Serial Monitor if pins aren't connected or the IMU isn't working
 * Operates at a Baud Rate of 115200
 * 
 * Used in the setup() function in main.cpp

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

//From IMU, outputs pitch and roll
float calcPitch(float ax, float ay, float az) {
  return atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
}

float calcRoll(float ax, float ay, float az) {
  return atan2(ay, az) * 180.0 / PI;
}

//Outputs angular velocity data
//The axis input variable must be either 'x' 'y' or 'z' for these three coordinate axes
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

//Convert to Quarternions to prevent Euler gimbal lock, then make the PID loop. Credit: https://www.nmas.org/wp-content/uploads/2023/03/2022_NMJS_Kim.pdf
//Inputs from the IMU
void calcOrientation(float wx, float wy, float wz) { //Deadline: Saturday 2/14 Night
  float w[4] = {0.0, wx, wy, wz};
  float q[4] = {1.0, 0.0, 0.0, 0.0};
  
  //Make the Hamilton Operator Calculations

  //Derivative and Integral Calculations

  //Normalize the Quarternion

  //Convert to Euler Angles
    
  //Account for Roll

  //Find the acceleration values to determine the PID gain (still a little confused on)
}

float updatePID(PID &pid, float measurement, float setpoint, float limMaxI) {
  unsigned long now = millis();
  float dt = (now - pid.lastTime) / 1000.0f;
  pid.lastTime = now;

  if (dt <= 0.001f) return 0;

  float error = setpoint - measurement;

  // Proportional term
  float P = pid.kp * error;

  // I (anti-windup)
  pid.integral += pid.ki * error * dt;
  pid.integral = constrain(pid.integral, -limMaxI, limMaxI);

  // D (filtered version to prevent noise I believe)
  float rawD = (error - pid.prevError) / dt;
  pid.derivative = (2.0f * pid.tau * pid.derivative + dt * rawD) /
                   (2.0f * pid.tau + dt);

  float D = pid.kd * pid.derivative;

  pid.prevError = error;

  float output = P + pid.integral + D;
  return constrain(output, -limMaxI, limMaxI);
}

//Do this first
//Calculate the PID equation that gives the necessary angle for stabilization
// float outputPID(float imuAngle, float limMaxI) {
//   float time = millis();
//   float dt = (time - lastTime) / 1000.0f;
//   lastTime = time;

//   if (dt <= 0) return 0;

//   float error = setpointAngle - imuAngle;
//   return calcPID(error, dt, limMaxI);
// }

// //Helper method that calculates the actual PID output necessary to stabilize
// float calcPID(float error, float dt, float limMaxI) {
//   float proportional = kp * error;

//   if(fabs(error) > 0.2f) {
//     integral += 0.5f * ki * dt * (error + previous);
//   }

//   integral = constrain(integral, -limMaxI, limMaxI);

//   float rawDerivative = (error - previous) / dt;

//   float filteredDerivative =
//       (2.0f * tau * derivative + dt * rawDerivative) /
//       (2.0f * tau + dt);

//   derivative = kd * filteredDerivative;

//   previous = error;

//   float output = proportional + integral + derivative;
//   output = constrain(output, -limMaxI, limMaxI);

//   Serial.print("error: ");
// Serial.print(error, 4);
// Serial.print("  P: ");
// Serial.print(proportional, 4);
// Serial.print("  D: ");
// Serial.print(derivative, 4);
// Serial.print("  I: ");
// Serial.println(integral, 4);
//   return output;
// }