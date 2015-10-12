/*
  Pathfinder Arduino
  v3 Tin Man

  Obstacle avoidance with brushed DC motors, encoders, and IR sensors.

  modified 12 Oct 2015
  by Arun Kurian
 */

// Include Libraries
#include <SoftwareSerial.h>
#include <PololuQik.h>
#include <Encoder.h>
#include <autotune.h>
#include <microsmooth.h>


// Initialization
PololuQik2s9v1 qik(10, 11, 12); // Constructor for qik motor controller

Encoder leftEncoder(2, 3);      // Initialize left encoder on pins 2 (A) and 3 (B)
Encoder rightEncoder(20, 21);   // Initialize right encoder on pins 20 (A) and 21 (B)

int incomingByte;               // Incoming serial byte
int robotMode;                  // Define autonomous(1) or remote control(0) mode

int irForwardLimit;             // Forward IR Limit
int irSideLimit;                // Side IR Limits
int irForwardAnalog;            // Forward IR analog reading
int irLeftAnalog;               // Left IR analog reading
int irRightAnalog;              // Right IR analog reading

uint16_t *irForwardPtr;         // Forward IR history pointer
uint16_t *irLeftPtr;            // Left IR history pointer
uint16_t *irRightPtr;           // Right IR history pointer


// Setup Function
void setup() {

  Serial1.begin(115200);          // Begin serial communication with bluetooth
  Serial.begin(9600);

  robotMode = 0;                  // Set to remote control mode initially

  irForwardLimit = 300;           // Set IR limits for analog reading
  irSideLimit = 400;              // Higher means closer

  irForwardPtr = ms_init(SMA);    // Initialize IR history pointers
  irLeftPtr = ms_init(SMA);
  irRightPtr = ms_init(SMA);

  qik.init();                     // Reset qik and initialize serial comms at 9600 bps

}


// Loop Function
void loop() {

  irForwardAnalog = analogRead(A0);                              // Read forward IR sensor reading
  irForwardAnalog = sma_filter(irForwardAnalog, irForwardPtr);   // Use simple moving average filter

  irLeftAnalog = analogRead(A1);                                 // Read left IR sensor reading
  irLeftAnalog = sma_filter(irLeftAnalog, irLeftPtr);            // Use simple moving average filter

  irRightAnalog = analogRead(A2);                                // Read right IR sensor reading
  irRightAnalog = sma_filter(irRightAnalog, irRightPtr);         // Use simple moving average filter

  // For autonomous control of robot
  if (robotMode == 1) {

    // Run IR-based obstacle avoidance routine
    runObstacleAvoidance();

  }

  // Run while serial communication with control is active
  if (Serial1.available() > 0) {

    incomingByte = Serial1.read();      // Read recieved serial data

    // Set control mode of robot
    if (incomingByte == '1') {
      robotMode = 1;                   // Set robot mode to autonomous
    }
    else if (incomingByte == '0') {
      robotMode = 0;                   // Set robot mode to remote control
    }

    // For remote control of robot
    if (robotMode == 0) {

      // Run Bluetooth-based remote control routine
      runRemoteControl(incomingByte);
      setDesiredTranslation(0); //Stop

    }

    // Send sensor data of serial
    if (incomingByte == 'r') {

      Serial1.print(irForwardAnalog);
      Serial1.print(',');
      Serial1.print(irLeftAnalog);
      Serial1.print(',');
      Serial1.println(irRightAnalog);

    } else {

      delay(100);

    }
  }

}

// Remote control routine (in remote control mode)
void runRemoteControl(int inByte) {

  int delTranslate = 30;                      // Set delta translate value
  int delRotate = 25;                         // Set delta rotate value

  if (inByte == 'w') {
    setDesiredTranslation(delTranslate);      // Move forward if recieved byte is 'w'
  }
  else if (inByte == 's') {
    setDesiredTranslation(-delTranslate);     // Move backward if recieved byte is 's'
  }
  else if (inByte == 'a') {
    setDesiredRotation(-delRotate);           // Turn left if recieved byte is 'a'
  }
  else if (inByte == 'd') {
    setDesiredRotation(delRotate);            // Turn right if recieved byte is 'd'
  }

}

// Obstacle avoidance routine (in autonomous mode)
void runObstacleAvoidance() {

  int irOverall;                                    // Weighted average of all IR readings

  irOverall = irForwardAnalog / 2;
  irOverall = irOverall + irLeftAnalog / 4;
  irOverall = irOverall + irRightAnalog / 4;
  irOverall = constrain(irOverall, 0, 350);

  int motorSpeed = map(irOverall, 0, 350, 100, 40);  // Determine motor speed from IR saturation

  // If completely surrounded
  if (irOverall > 325) {

    setDesiredTranslation(0);     // Stop
    delay(250);
    setDesiredTranslation(-50);   // Move back

    // Turn around
    if (irLeftAnalog < irRightAnalog) {
      setDesiredRotation(-180);
    } else {
      setDesiredRotation(180);
    }

  }

  // If forward IR detects obstacle
  if (irForwardAnalog > irForwardLimit) {

    setDesiredTranslation(0);              // Stop
    delay(100);
    setDesiredTranslation(-50);            // Move back

    // If there is more space on left side
    if (irLeftAnalog < irRightAnalog) {    // Turn left
      setDesiredRotation(-45);
    } else {                               // Turn right
      setDesiredRotation(45);
    }

  // If forward IR doesn't detect obstacle
  } else {
    setDesiredVelocity(motorSpeed, 1);     // Move forward
  }

  // If left IR detects an obstacle, turn right
  if (irLeftAnalog > irSideLimit) {
    setDesiredRotation(30);
  }

  // If right IR detects an obstacle, turn left
  if (irRightAnalog > irSideLimit) {
    setDesiredRotation(-30);
  }

}

// Function to set robot direction of motion
void setDesiredVelocity(int motorSpeed, int bodyDirection) {

  if (bodyDirection == 1) {                  // Move forward
    qik.setSpeeds(-motorSpeed, motorSpeed);
  } else {                                   // Move back
    qik.setSpeeds(motorSpeed, -motorSpeed);
  }

}

// Function to return actual motor angle positions when passed encoder values
float getMotorActual(float encoderValue) {

  float motorActual;
  float countsPerRev = 2248.8;                // Counts per revolution of geared motor shaft

  motorActual = encoderValue / countsPerRev;  // Determine true angle of motor shaft
  motorActual = motorActual * 360;

  return motorActual;

}

// Function to convert desired translations of the robot into rotations of the motors
void setDesiredTranslation(int desiredTranslation) {

  int radiusWheel = 40;                  // Define radius of wheel
  float arcRatio = 57.3 / radiusWheel;   // Define ratio for arc length of the wheels

  // Calculate the translation of the body
  float translation = desiredTranslation * arcRatio;

  // Run desired step
  setDesiredStep(translation, -translation);

}

// Function to convert desired rotations of robot into rotations of the motors
void setDesiredRotation(int desiredAngle) {

  int radiusWheel = 40;                    // Define radius of wheel
  int radiusBody = 85;                     // Define half distance between wheels

  float radiusRatio = 85 / 40;              // Find ratio of radii
  
  // Calculate angles from desired body rotation and ratio of radii
  float leftAngle = (float)desiredAngle * radiusRatio;
  float rightAngle = (float)desiredAngle * radiusRatio;

  // Run desired step
  setDesiredStep(leftAngle, rightAngle);

}

// Function to accomplish desired steps for motor movements using PID
void setDesiredStep(float leftDesiredDiff, float rightDesiredDiff) {

  float leftMotorDesired, rightMotorDesired;
  float leftMotorActual, rightMotorActual;
  float leftMotorCommand, rightMotorCommand;
  float leftErrorLast, rightErrorLast;
  float leftErrorDer, rightErrorDer;

  // Define control gains (Kp, Ki, Kd) for PID
  float KpLeft = 0.0092;
  float KpRight = 0.009;
  float Ki = 0.001;
  float Kd = 0.001;

  // Initialize error, desired, and actual values
  float leftError = leftDesiredDiff;
  float rightError = rightDesiredDiff;

  float leftErrorIntegral = 0;
  float rightErrorIntegral = 0;

  leftMotorActual = getMotorActual( (float)leftEncoder.read() );
  rightMotorActual = getMotorActual( (float)rightEncoder.read() );

  leftMotorDesired = leftMotorActual + leftDesiredDiff;
  rightMotorDesired = rightMotorActual + rightDesiredDiff;
  
  // Drive error to zero while errors are greater than 5 degrees
  while (abs(leftError) > 5 || abs(rightError) > 5) {

    // Get current motor angles
    leftMotorActual = getMotorActual( (float)leftEncoder.read() );
    rightMotorActual = getMotorActual( (float)rightEncoder.read() );

    // Save previous error for derivative error term
    leftErrorLast = leftError;
    rightErrorLast = rightError;

    // Calculate error (desired - actual)
    leftError = leftMotorDesired - leftMotorActual;
    rightError = rightMotorDesired - rightMotorActual;

    // Calculate sum of errors when errors begin to converge (to prevent windup)
    if (abs(leftError) < abs(leftDesiredDiff / 3)) {
      leftErrorIntegral = leftErrorIntegral + leftError;
    } else {
      leftErrorIntegral = 0;
    }

    if (abs(rightError) < abs(rightDesiredDiff / 3)) {
      rightErrorIntegral = rightErrorIntegral + rightError;
    } else {
      rightErrorIntegral = 0;
    }

    // Calculate difference of previous and current errors
    leftErrorDer = leftErrorLast - leftError;
    rightErrorDer = rightErrorLast - rightError;

    // Define PID motor command for left motor
    leftMotorCommand = KpLeft * leftError + Ki * leftErrorIntegral + Kd * leftErrorDer;
    leftMotorCommand = constrain(leftMotorCommand, -1, 1);
    leftMotorCommand = leftMotorCommand * 127.0;

    // Define PID motor command for right motor
    rightMotorCommand = KpRight * rightError + Ki * rightErrorIntegral + Kd * rightErrorDer;
    rightMotorCommand = constrain(rightMotorCommand, -1, 1);
    rightMotorCommand = rightMotorCommand * 127.0;

    // Set motor command speeds
    qik.setSpeeds(-(int)leftMotorCommand, -(int)rightMotorCommand);

    // Delay loop time for consistency
    delay(3);

  }

  // Stop motors when desired angles achieved
  qik.setSpeeds(0, 0);

}
