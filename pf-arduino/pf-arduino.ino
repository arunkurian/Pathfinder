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

int forwardPos;                // Forward 0 - 2000 position reading
int forwardReading;            // Forward 0 - 1000 averaged sensor readings
int leftReading;               // Left 0 - 1000 averaged sensor readings
int rightReading;              // Right 0 - 1000 averaged sensor readings

uint16_t *lineForwardPtr;       // Forward IR history pointer
uint16_t *lineLeftPtr;          // Left IR history pointer
uint16_t *lineRightPtr;         // Right IR history pointer

int intCounter;                 // Intersection counter to wait and verify intersection points
int minLineValue = 350;         // Minimum average line readings to detect intersection

int steadySpeed;                // Steady speed for robot
int driveDelay;                 // Drive delay for turns

float Kp;                       // PD gains 
float Kd;

int notFound = 0;

int sensorReadCount = 0;        // Initial sensor count
int lastError = 0;              // Initial error

LiquidCrystal lcd(49, 47, 48, 50, 51, 52, 53);   // LCD display setup with 4 data lines

Servo camServo;                 // Initialize tilt servo for camera


// Setup Function
void setup() {

  Serial.begin(9600);             // Begin serial communication with ODROID
  Serial1.begin(115200);          // Begin serial communication with Bluetooth

  irForwardPtr = ms_init(SMA);    // Initialize obstacle IR history pointers
  irLeftPtr = ms_init(SMA);
  irRightPtr = ms_init(SMA);

  lineForwardPtr = ms_init(SMA);  // Initialize line IR history pointers
  lineLeftPtr = ms_init(SMA);
  lineRightPtr = ms_init(SMA);

  irForwardLimit = 300;           // Set IR limits for analog reading
  irSideLimit = 500;              // Higher means closer

  camServo.attach(8);             // Set up tilt servo for camera on Pin 8
  camServo.write(50);             // Set servo motor to default 50 degrees

  qik.init();                     // Reset qik and initialize serial comms at 9600 bps
  lcd.begin(16, 2);               // Set up LCD for 16x2 display

  switchProfile(0);
  
  lcd.clear();
  lcd.setCursor(3, 0);            // Display introduction message
  lcd.print("Pathfinder");

  lcd.setCursor(0, 1);
  lcd.print("C1+: ");
  lcd.setCursor(9, 1);
  lcd.print("BT: ");

  // Pause in setup until C1+ and BT are connected

  // C1+ (USB-Serial)
  while (!Serial.available());

  incomingByte = Serial.read() - '0';
  Serial.print(incomingByte);    // Handshake
  lcd.setCursor(5, 1);
  lcd.print("Go!");

  // BT (Serial1)
  while (!Serial1.available());

  incomingByte = Serial1.read() - '0';
  Serial1.print(incomingByte);    // Handshake
  lcd.setCursor(13, 1);
  lcd.print("Go!");

  delay(1000);

  Serial.flush();                 // Flush serial buffer
  Serial1.flush();

  calibrateLineSensors();         // Run routine to calibrate line sensors

  delay(2000);                    // Pause in setup to verify cal before going to main loop
randomSeed(analogRead(0));
}


// Loop Function
void loop() {

  unsigned int forwardLineSensors[3];
  unsigned int forwardSensors[3];
  unsigned int rightSensors[3];
  unsigned int leftSensors[3];

  readLines(forwardLineSensors, forwardSensors, leftSensors, rightSensors);  // Read lines

  camServo.write(50);
  int obstacle = obstacleDetected();                                         // Determine if obstacle is detected

  // Wait until atleast three reads have been completed
  if (sensorReadCount > 3) {

    // Run while not picked up (1000), nonzero, and no obstacle
    if (forwardReading < 1000 && forwardReading > 0 && obstacle == 0) {

      // Stop at intersection, if left or right detectors exceed values
      if (leftReading > minLineValue || rightReading > minLineValue || notFound == 1) {

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

  irForwardAnalog = analogRead(A0);                              // Read forward IR sensor reading
  irForwardAnalog = sma_filter(irForwardAnalog, irForwardPtr);   // Use simple moving average filter

  irLeftAnalog = analogRead(A1);                                 // Read left IR sensor reading
  irLeftAnalog = sma_filter(irLeftAnalog, irLeftPtr);            // Use simple moving average filter

  irRightAnalog = analogRead(A2);                                // Read right IR sensor reading
  irRightAnalog = sma_filter(irRightAnalog, irRightPtr);         // Use simple moving average filter

  // If forward, left, or right IRs detect an obstacle, set flag
  if (irForwardAnalog > irForwardLimit) {
    return 2;
  }
  else if (irLeftAnalog > irSideLimit) {
    return 1;
  }
  else if (irRightAnalog > irSideLimit) {
    return 3;
  }
  else {
    return 0;
  }

}
