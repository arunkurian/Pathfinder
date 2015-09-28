/*
  Pathfinder Arduino
  v2 Scarecrow

  Obstacle avoidance with brushed DC motors, encoders, and IR sensors.

  modified 26 Sept 2015
  by Arun Kurian
 */

// Include Libraries
#include <SoftwareSerial.h>
#include <PololuQik.h>
#include <autotune.h>
#include <microsmooth.h>


// Initialization
const int leftEncoderA = 2;           // Encoder output A for left motor
const int leftEncoderB = 4;           // Encoder output B for left motor
const int rightEncoderA  = 3;         // Encoder output A for right motor
const int rightEncoderB = 5;          // Encoder output B for right motor

volatile unsigned int leftEncoderPos = 0;  // Encoder count for left motor
volatile unsigned int rightEncoderPos = 0; // Encoder count for right motor

PololuQik2s9v1 qik(10, 11, 12); // Constructor for qik motor controller

float leftMotorAngle;           // True angle of left motor
float rightMotorAngle;          // True angle of right motor
int motorSpeed;                 // Motor speed
int currentDirection;           // Current direction of travel (0 = stop, 1 = forward, 2 = back, 3 = left, 4 = right)

int incomingByte;               // Incoming serial byte
int robotMode;                  // Define autonomous(1) or remote control(0) mode

int irForwardLimit;             // Forward IR Limit
int irSideLimit;                // Side IR Limits
int irForwardAnalog;            // Forward IR analog reading
int irLeftAnalog;               // Left IR analog reading
int irRightAnalog;              // Right IR analog reading
int irOverall;                  // Weighted average of IR readings
int irAbsDiff;                  // Absolute difference between side IR sensors 
int backupDelay;                // Delay in moving backward when obstacle

uint16_t *irForwardPtr;         // Forward IR history pointer
uint16_t *irLeftPtr;            // Left IR history pointer
uint16_t *irRightPtr;           // Right IR history pointer


// Setup Function
void setup() {

  Serial1.begin(115200);          // Begin serial communication with bluetooth

  robotMode = 0;                  // Set to remote control mode initially
  currentDirection = 0;

  irForwardLimit = 300;           // Set IR limits for analog reading
  irSideLimit = 400;              // Higher means closer

  irForwardPtr = ms_init(SMA);    // Initialize IR history pointers
  irLeftPtr = ms_init(SMA);
  irRightPtr = ms_init(SMA);

  qik.init();                     // Reset qik and initialize serial comms at 9600 bps

  pinMode(leftEncoderA, INPUT);            // Set pin modes for left encoder outputs
  pinMode(leftEncoderB, INPUT);
  digitalWrite(leftEncoderA, HIGH);        // Turn on pullup resistors
  digitalWrite(leftEncoderB, HIGH);

  pinMode(rightEncoderA, INPUT);           // Set pin modes for right encoder outputs
  pinMode(rightEncoderB, INPUT);
  digitalWrite(rightEncoderA, HIGH);       // Turn on pullup resistor
  digitalWrite(rightEncoderB, HIGH);

  attachInterrupt(0, doLeftEncoder, CHANGE);   // Interrupt for Left Encoder Pin A (0 - Pin 2)
  attachInterrupt(1, doRightEncoder, CHANGE);  // Interrupt for Right Encoder Pin A (1 - Pin 3)

}


// Loop Function
void loop() {

  irForwardAnalog = analogRead(A0);                              // Read forward IR sensor reading
  irForwardAnalog = sma_filter(irForwardAnalog, irForwardPtr);   // Use simple moving average filter

  irLeftAnalog = analogRead(A1);                                 // Read left IR sensor reading
  irLeftAnalog = sma_filter(irLeftAnalog, irLeftPtr);

  irRightAnalog = analogRead(A2);                                // Read right IR sensor reading
  irRightAnalog = sma_filter(irRightAnalog, irRightPtr);

  irOverall = irForwardAnalog / 2;
  irOverall = irOverall + irLeftAnalog / 4;
  irOverall = irOverall + irRightAnalog / 4;
  irOverall = constrain(irOverall, 0, 350);
  
  motorSpeed = map(irOverall, 0, 350, 100, 40);

  // For autonomous control of robot
  if (robotMode == 1) {

    // If forward IR detects obstacle
    if (irForwardAnalog > irForwardLimit) {
      setDirection(0);      // Stop
      delay(100);
      
      irAbsDiff = irLeftAnalog - irRightAnalog;
      irAbsDiff = abs(irAbsDiff);
      irAbsDiff = constrain(irAbsDiff, 0, 100);
      
      backupDelay = map(irAbsDiff, 0, 100, 300, 100);

      setDirection(2);      // Move back
      delay(backupDelay);

      // If there is more space on left side
      if (irLeftAnalog <= irRightAnalog) {
        setDirection(3);    // Turn left
        // If there is more space on right side
      } else {
        setDirection(4);    // Turn right
      }

      delay(200);

      // If forward IR doesn't detect obstacle
    } else {
      setDirection(1);      // Move forward
    }

    // If left IR detects an obstacle
    if (irLeftAnalog > irSideLimit) {
      setDirection(4);      // Turn right
      delay(200);
    }

    // If right IR detects an obstacle
    if (irRightAnalog > irSideLimit) {
      setDirection(3);      // Turn left
      delay(200);
    }

  }

  // Run while serial communication with control is active
  if ( Serial1.available() > 0) {

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

      if (incomingByte == 'w') {
        setDirection(1);                  // Move forward if recieved byte is 'w'
      }
      else if (incomingByte == 's') {
        setDirection(2);                  // Move backward if recieved byte is 's'
      }
      else if (incomingByte == 'a') {
        setDirection(3);                  // Turn left if recieved byte is 'a'
      }
      else if (incomingByte == 'd') {
        setDirection(4);                  // Turn right if recieved byte is 'd'
      }

    }

    // Send sensor data of serial
    if (incomingByte == 'r') {

      Serial1.print(irForwardAnalog);
      Serial1.print(',');
      Serial1.print(irLeftAnalog);
      Serial1.print(',');
      Serial1.print(irRightAnalog);
      Serial1.print(',');
      Serial1.println(motorSpeed);

    } else {

      delay(100);

    }

  } else {

    if (robotMode == 0) {
      setDirection(0);                    // Stop between remote control readings
    }

  }

}


// Function to set robot direction of motion
void setDirection(int newDirection) {

  if (newDirection != currentDirection) {

    if (newDirection == 0) {          // Stop
      qik.setSpeeds(0, 0);
    } else if (newDirection == 1) {   // Move forward
      qik.setSpeeds(-motorSpeed, motorSpeed);
    } else if (newDirection == 2) {   // Move backward
      qik.setSpeeds(motorSpeed, -motorSpeed);
    } else if (newDirection == 3) {   // Turn left
      qik.setSpeeds(motorSpeed, motorSpeed);
    } else {                          // Turn right
      qik.setSpeeds(-motorSpeed, -motorSpeed);
    }

    // Set current direction to new direction
    currentDirection = newDirection;

  }

}

// Interrupt function for left motor encoder counts
void doLeftEncoder() {
  if (digitalRead(leftEncoderA) == digitalRead(leftEncoderB)) {
    leftEncoderPos++;
  } else {
    leftEncoderPos--;
  }
}

// Interrupt function for right motor encoder counts
void doRightEncoder() {
  if (digitalRead(rightEncoderA) == digitalRead(rightEncoderB)) {
    rightEncoderPos++;
  } else {
    rightEncoderPos--;
  }
}
