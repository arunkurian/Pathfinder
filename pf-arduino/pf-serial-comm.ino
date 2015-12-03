/*
  Pathfinder Arduino - Serial Communications
  v4 Wizard

  Functions for serial communications with ODROID and BT.

  modified 24 Nov 2015
  by Arun Kurian
 */

// ODROID servo commands for shape classification and servo movement
int odroidCommand() {

  int defaultServo = 50;
  
  int currentPos, newPos, difference;
  int sweepDiff = -1;
  int turnPos = 0;
  int speedProfile = 0;
  
  camServo.write(defaultServo);             // Set servo motor to default 50 degrees
  currentPos = defaultServo;
  newPos = defaultServo;

  bool done = false;

  while (!done) {

    char incomingByte = Serial.read();
    currentPos = camServo.read();

    switch (incomingByte) {

      case 'A': // Servo add command
        {
          difference = Serial.parseInt();
          
          // Rotate camera desired difference, or else sweep
          if (difference != 100) {
            newPos = currentPos + difference;
          } else {
            if (currentPos == 40) {
              sweepDiff = 1;
            } else if (currentPos == 60) {
              sweepDiff = -1;
            }
            newPos = currentPos + sweepDiff; 
          }

          break;
        }
      case 'S': // Servo subtract command
        {
          difference = Serial.parseInt();

          // Rotate camera desired difference, or else sweep
          if (difference != 100) {
            newPos = currentPos - difference;
          } else {
            if (currentPos == 40) {
              sweepDiff = 1;
            } else if (currentPos == 60) {
              sweepDiff = -1;
            }
            newPos = currentPos + sweepDiff; 
          }

          break;
        }
      case 'T': // Turn direction value (1 = Left, 2 = Straight, 3 = Right)
        {
          turnPos = Serial.parseInt();
          break;
        }
      case 'C': // Speed value (1 = Fast profile, 0 = Slow profile)
        {
          speedProfile = Serial.parseInt();
          if (turnPos != 0) {
            switchProfile(speedProfile);
          }
          break;
        }
      case 'Q': // Shape classification complete, exit loop
        {
          done = true;
          break;
        }
    }

    newPos = constrain(newPos, 40, 60);
    camServo.write(newPos);

  }

  camServo.write(50);             // Set servo motor to default 50 degrees
  return turnPos;

}

void switchProfile(int profile) {

  if (profile == 1) {

    steadySpeed = 70;
    intCounterMin = 3;
    Kp = 0.095;
    Kd = 0.09;

    lcd.setCursor(11, 0);
    lcd.print("Green");
    lcd.setCursor(12, 1);
    lcd.print("Fast");
    
  } else if (profile == 0) {

    steadySpeed = 50;
    intCounterMin = 8;
    Kp = 0.085;
    Kd = 0.07;
    
    lcd.setCursor(12, 0);
    lcd.print("Pink");
    lcd.setCursor(12, 1);
    lcd.print("Slow");

  }

}
