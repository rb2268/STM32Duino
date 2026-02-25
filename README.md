Contributor(s): Riley Becker
2/25/26
Project: TVC Mount PID Loop

Board: STM32 Nucleo L432KC ---> Selected for accessibility, efficiency, and increased control over low-level capabilities
Framework: Arduino  --->  Selected due to simplicity of Code and relative ease of incorporating sensors

Constraints: Code Designed for a TVC gimbal (two Servos mounted to control pitch and roll axes)
 - Used two SG92R servos for extra robustness (SG90s also work)
 - Found the maximum rotation of the two servos to be 30 and 10 degrees, I included code to clamp the maximum rotation based on this
 - Requires an IMU with a Gyro and Accelerometer
 - I used an external 5V power source from a wall socket plug-in to prevent voltage sag in the motors

Code Resources:
 - Used Phil's Lab https://www.youtube.com/watch?v=zOByx3Izf5U and https://www.youtube.com/watch?v=RZW1PsfgVEI for programming the PID calculations (converts the PID equation into discrete timestep functions for programming)
 - Used ChatGPT for a smoothing equation that prevents movement in rest

Project Status:
 - Demonstrated working PID Control with slight jitter (working on fixing)
 - Currently one of the axes has a mechanical issue

Code Docmentation:
 - Used the Arduino Servo library
