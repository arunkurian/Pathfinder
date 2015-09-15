/*
  Pathfinder Arduino
  v1.0 Dorothy
  
  Obstacle avoidance with continuous rotation servos and IR sensors.

  modified 14 Sept 2015
  by Arun Kurian
 */

#include <Servo.h>
#include <autotune.h>
#include <microsmooth.h>

// Initialization
Servo leftServo;                // Initialize servos
Servo rightServo;

int incomingByte;               // Incoming serial byte
int robotMode;                  // Define autonomous(1) or remote control(0) mode  

int irLimit;                    // Forward IR Limit
int irLED;                      // Forward IR LED
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
  
  irLED = 12;                     // Set IR pin number
  irLimit = 300;                  // Set IR limit analog reading
  
  pinMode(irLED, OUTPUT);         // Set pin for IR indicator LED
  
  irForwardPtr = ms_init(SMA);    // Initialize IR history pointers
  irLeftPtr = ms_init(SMA);
  irRightPtr = ms_init(SMA);
  
  leftServo.attach(8);            // Attach servos to pins and set to no movement
  rightServo.attach(9);
  moveStop();                     // Set movement to stop

}


// Loop Function
void loop() {
  
  irForwardAnalog = analogRead(A0);                              // Read forward IR sensor reading
  irForwardAnalog = sma_filter(irForwardAnalog, irForwardPtr);   // Use simple moving average filter
  
  irLeftAnalog = analogRead(A1);                                 // Read left IR sensor reading
  irLeftAnalog = sma_filter(irLeftAnalog, irLeftPtr);

  irRightAnalog = analogRead(A2);                                // Read right IR sensor reading
  irRightAnalog = sma_filter(irRightAnalog, irRightPtr);
  
  // Light forward obstacle LED if higher (closer) than limit
  if (irForwardAnalog > irLimit) {
    digitalWrite(irLED, HIGH);
  } else {
    digitalWrite(irLED, LOW);
  }
    
  // For autonomous control of robot
  if (robotMode == 1) {
    
    if (irForwardAnalog > irLimit) {
      // If forward IR detects obstacle
      moveStop();
      delay(100);
      
      moveBackward();
      delay(100);
      
      if (irLeftAnalog <= irRightAnalog) { 
        // If there is more space on left side
        turnLeft();
      } else {
        // If there is more space on right side
        turnRight(); 
      }
      
      delay(200);
      
    } else {
      // If forward IR doesn't detect obstacle
      moveForward();
    }
    
    // If left IR detects an obstacle
    if (irLeftAnalog > irLimit) {
       turnRight();
       delay(200); 
    }
    
    // If right IR detects an obstacle
    if (irRightAnalog > irLimit) {
       turnLeft();
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
        moveForward();                  // Move forward if recieved byte is 'w'
      }
      else if (incomingByte == 's') {
        moveBackward();                 // Move backward if recieved byte is 's'
      }
      else if (incomingByte == 'a') {
        turnLeft();                     // Turn left if recieved byte is 'a' 
      }
      else if (incomingByte == 'd') {
        turnRight();                    // Turn right if recieved byte is 'd' 
      }
    
    }

    // Send sensor data of serial
    if (incomingByte == 'r') {
      Serial1.print(irForwardAnalog);
      Serial1.print(',');
      Serial1.print(irLeftAnalog);
      Serial1.print(',');
      Serial1.println(irRightAnalog);
    }

    delay(150);        // Delay loop time for 150ms
    moveStop();
   
  } else {
    
    delay(100);
    
  }
  
}

// Function to move robot forward
void moveForward() {
  leftServo.write(180);
  rightServo.write(0);
}

// Function to move robot backward
void moveBackward() {
  leftServo.write(0);
  rightServo.write(180);
}

// Function to turn robot to left
void turnLeft() {
  leftServo.write(0);
  rightServo.write(0);
}

// Function to turn robot to right
void turnRight() {
  leftServo.write(180);
  rightServo.write(180);
}

// Function to stop robot motion
void moveStop() {
  leftServo.write(90);
  rightServo.write(90);
}
