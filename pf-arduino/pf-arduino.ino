/*
  Pathfinder Arduino - Main
  v4 Wizard

  Obstacle avoidance with brushed DC motors, encoders, and IR sensors.

  modified 9 Nov 2015
  by Arun Kurian
 */

// Include Libraries
#include <SoftwareSerial.h>
#include <PololuQik.h>
#include <Encoder.h>
#include <autotune.h>
#include <microsmooth.h>
#include <QTRSensors.h>
#include <LiquidCrystal.h>
#include <Servo.h>


// Initialization
PololuQik2s9v1 qik(11, 12, 10); // Constructor for qik motor controller

Encoder leftEncoder(2, 3);      // Initialize left encoder on pins 2 (A) and 3 (B)
Encoder rightEncoder(21, 20);   // Initialize right encoder on pins 20 (A) and 21 (B)

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

QTRSensorsAnalog qtrForward((unsigned char[]) {  // Constructor for forward line sensor
  8, 9, 10
}, 3);

QTRSensorsAnalog qtrLeft((unsigned char[]) {     // Constructor for left line sensor
  11, 12, 13
}, 3);

QTRSensorsAnalog qtrRight((unsigned char[]) {    // Constructor for right line sensor
  5, 6, 7
}, 3);

int forwardReading;
int leftReading;
int rightReading;
int forwardPos;

uint16_t *lineForwardPtr;       // Forward IR history pointer
uint16_t *lineLeftPtr;          // Left IR history pointer
uint16_t *lineRightPtr;         // Right IR history pointer

int intCounter;                 // Intersection counter to wait and verify intersection points
int minLineValue = 350;         // Minimum average line readings to detect intersection
int steadySpeed = 50;           // Steady speed for robot
int sensorReadCount = 0;        // Initial sensor count
int lastError = 0;              // Initial error

LiquidCrystal lcd(49, 47, 48, 50, 51, 52, 53);   // LCD display setup with 4 data lines

Servo camServo;                 // Initialize tilt servo for camera


// Setup Function
void setup() {

  Serial1.begin(115200);          // Begin serial communication with bluetooth
  Serial.begin(9600);

  irForwardPtr = ms_init(SMA);    // Initialize obstacle IR history pointers
  irLeftPtr = ms_init(SMA);
  irRightPtr = ms_init(SMA);

  lineForwardPtr = ms_init(SMA);  // Initialize line IR history pointers
  lineLeftPtr = ms_init(SMA);
  lineRightPtr = ms_init(SMA);

  irForwardLimit = 300;           // Set IR limits for analog reading
  irSideLimit = 400;              // Higher means closer

  camServo.attach(8);             // Set up tilt servo for camera on Pin 8
  camServo.write(60);             // Set servo motor to 60 degrees - forward

  qik.init();                     // Reset qik and initialize serial comms at 9600 bps
  lcd.begin(16, 2);               // Set up LCD for 16x2 display

  lcd.setCursor(0, 0);            // Display introduction message
  lcd.print("PATHFINDER");
  lcd.setCursor(0, 1);
  lcd.print("Waiting on BT...");

  Serial1.flush();                // Flush Serial1 (BT) buffer
  while (!Serial1.available());   // Wait until Serial1 (BT) connection is made before proceeding

  lcd.clear();                    // Clear LCD and Display "BT Connected!" message
  lcd.setCursor(0, 0);
  lcd.print("BT Connected!");
  delay(1000);

  calibrateLineSensors();         // Run routine to calibrate line sensors

  delay(2000);                    // Pause in setup to verify before going to main loop

}


// Loop Function
void loop() {

  unsigned int forwardLineSensors[3];
  unsigned int forwardSensors[3];
  unsigned int rightSensors[3];
  unsigned int leftSensors[3];

  irForwardAnalog = analogRead(A0);                              // Read forward IR sensor reading
  irForwardAnalog = sma_filter(irForwardAnalog, irForwardPtr);   // Use simple moving average filter

  irLeftAnalog = analogRead(A1);                                 // Read left IR sensor reading
  irLeftAnalog = sma_filter(irLeftAnalog, irLeftPtr);            // Use simple moving average filter

  irRightAnalog = analogRead(A2);                                // Read right IR sensor reading
  irRightAnalog = sma_filter(irRightAnalog, irRightPtr);         // Use simple moving average filter

  int obstacle = obstacleDetected();                             // Determine if obstacle is detected

  forwardPos = qtrForward.readLine(forwardLineSensors);          // Calculate forward line position for PD error

  qtrForward.readCalibrated(forwardSensors);                     // Calculate average forward line readings
  forwardReading = averageLineReading(forwardSensors);
  forwardReading = sma_filter(forwardReading, lineForwardPtr);

  qtrLeft.readCalibrated(leftSensors);                           // Calculate left forward line readings
  leftReading = averageLineReading(leftSensors);
  leftReading = sma_filter(leftReading, lineLeftPtr);

  qtrRight.readCalibrated(rightSensors);                         // Calculate right forward line readings
  rightReading = averageLineReading(rightSensors);
  rightReading = sma_filter(rightReading, lineRightPtr);

  // Print readings on LCD
  lcd.clear();
  lcd.print(leftReading);
  lcd.setCursor(5, 0);
  lcd.print(forwardReading);
  lcd.setCursor(11, 0);
  lcd.print(rightReading);

  // Wait until atleast three reads have been completed
  if (sensorReadCount > 3) {

    // Run while not picked up (1000), nonzero, and no obstacle
    if (forwardReading < 1000 && forwardReading > 0 && obstacle == 0) {

      // Stop at intersection, if left or right detectors exceed values
      if (leftReading > minLineValue || rightReading > minLineValue) {
        
        // Wait until intersection is confirmed
        if (intCounter > 3) {

          // Stop
          qik.setSpeeds(0, 0);
          
          // Solve intersection
          arbitrateIntersection(forwardSensors, leftSensors, rightSensors);

        }
        
        // Iterate counter
        intCounter++;

      // Drive forward along line
      } else {
        
        int error;
        int errorDer;
        float motorSpeed;
        float leftMotorCommand;
        float rightMotorCommand;

        // Set PD gains
        float Kp = 0.06;
        float Kd = 0.07;

        error = 1000 - forwardPos;                                      // Calculate error

        errorDer = error - lastError;                                   // Calculate error derivative

        motorSpeed = Kp * error + Kd * errorDer;                        // Calculate PD command differential
        lastError = error;                                              // Set last error = current error

        leftMotorCommand = steadySpeed - motorSpeed;                    // Determine motor commands
        rightMotorCommand = steadySpeed + motorSpeed;

        leftMotorCommand = constrain(leftMotorCommand, -127, 127);      // Constrain motor commands
        rightMotorCommand = constrain(rightMotorCommand, -127, 127);

        qik.setSpeeds(leftMotorCommand, -rightMotorCommand);            // Set motor commands

      }

    } else {
      
      qik.setSpeeds(0, 0);  // Stop if obstacle is detected or no path
      
    }

  }

  // Iterate sensor count
  sensorReadCount++;

}

// Obstacle detection function
int obstacleDetected() {
  
  // If forward, left, or right IRs detect an obstacle, set flag
  if (irForwardAnalog > irForwardLimit || irLeftAnalog > irSideLimit || irRightAnalog > irSideLimit) {
    return 1;
  }
  else {
    return 0;
  }
  
}
