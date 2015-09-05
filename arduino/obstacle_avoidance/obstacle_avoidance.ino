/*
  Pathfinder Arduino
  v1.0 Dorothy
  
  Obstacle avoidance with continuous rotation servos and IR sensors.

  modified 4 Sept 2015
  by Arun Kurian
 */

#include <Servo.h>

// Initialization

Servo leftServo;      // Initialize servos
Servo rightServo;

int incomingByte;          // Incoming serial byte
int irForwardLED = 12;          // Forward IR LED
int irForwardAnalog;       // Forward IR analog reading
float irForwardInch;         // Forward IR inch value

// Setup Function
void setup() {
  Serial1.begin(115200);  // Begin serial communication with bluetooth
  pinMode(irForwardLED, OUTPUT);
  leftServo.attach(8);    // Attach servos to pins and set to no movement
  rightServo.attach(9);
  servoMove(90,90);
}


// Loop Function
void loop() {
  
  irForwardAnalog = analogRead(A0);      // Read forward IR sensor reading
  irForwardAnalog = max(1, irForwardAnalog);
  irForwardInch = pow(irForwardAnalog,-0.867) * 539.98;  // Convert to inches
  
  Serial1.println(irForwardInch);    // Transmit IR reading
  
  // Light forward obstacle LED if less than 5 inches
  if (irForwardInch < 5) {
    digitalWrite(irForwardLED, HIGH);
  } else {
    digitalWrite(irForwardLED, LOW);
  }
  
  // Run while serial communication with control is active
  while ( Serial1.available() > 0) {
    
    incomingByte = Serial1.read();  // Read recieved serial data
    
    // Move forward if recieved byte is 'w'
    if (incomingByte == 'w') {
      Serial1.println("Move Forward");   // Transmit line over serial
      servoMove(180,0);                  // Move forward
    }
    
    // Move backward if recieved byte is 's'
    if (incomingByte == 's') {
      Serial1.println("Move Back");
      servoMove(0,180);
    }

    // Turn left if recieved byte is 's'    
    if (incomingByte == 'a') {
      Serial1.println("Turn Left");
      servoMove(0,0);
    }
    
    // Turn right if recieved byte is 's'    
    if (incomingByte == 'd') {
      Serial1.println("Turn Right");
      servoMove(180,180);
    }
     
    Serial1.flush();    // Flush serial data
   
  }
  
  delay(150);        // Delay loop time for 150ms for flush
  
  servoMove(90,90);  // Set servo stop between readings
  
}

// Servo movement function based on left and right control
void servoMove(int leftMove, int rightMove) {
 
   leftServo.write(leftMove);
   rightServo.write(rightMove);
 
}
