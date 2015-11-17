/*
  Pathfinder Arduino - Line Following
  v4 Wizard

  Line following functions. Calibration and intersection resolution.

  modified 9 Nov 2015
  by Arun Kurian
 */

// Line calibration function
void calibrateLineSensors() {

  int numIterations = 100;
  double percent;

  // Clear LCD and display line calibration
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Pathfinder");
  lcd.setCursor(0, 1);
  lcd.print("Calibrating ");

  // Drive forward slowly open loop
  qik.setSpeeds(-30, -30);

  // Calibrate for 1.25 seconds
  for (int i = 0; i < numIterations; i++) {

    // Calibrate each of the three line sensors
    qtrForward.calibrate();
    qtrRight.calibrate();
    qtrLeft.calibrate();

    // Display percent complete
    lcd.setCursor(12, 1);
    percent = 100 * i /  (numIterations - 1);
    percent = percent / 2.0;
    lcd.print((int)percent);
    lcd.print("%");

    delay(10);
  }

  // Stop
  qik.setSpeeds(0, 0);

  // Drive backward slowly open loop
  qik.setSpeeds(30, 30);

  // Calibrate for 1.25 seconds
  for (int i = 0; i < numIterations; i++) {

    // Calibrate each of the three line sensors
    qtrForward.calibrate();
    qtrRight.calibrate();
    qtrLeft.calibrate();

    // Display percent complete
    lcd.setCursor(12, 1);
    percent = 100 * i / (numIterations - 1);
    percent = percent / 2.0 + 50.0;
    lcd.print((int)percent);
    lcd.print("%");

    delay(10);
  }

  // Stop
  qik.setSpeeds(0, 0);

  // Average calibrated minimum and maximum values for display
  int leftCalMin = averageLineReading(qtrLeft.calibratedMinimumOn);
  int leftCalMax = averageLineReading(qtrLeft.calibratedMaximumOn);

  int forwardCalMin = averageLineReading(qtrForward.calibratedMinimumOn);
  int forwardCalMax = averageLineReading(qtrForward.calibratedMaximumOn);

  int rightCalMin = averageLineReading(qtrRight.calibratedMinimumOn);
  int rightCalMax = averageLineReading(qtrRight.calibratedMaximumOn);

}

void readLines(unsigned int forwardLineSensors[], unsigned int forwardSensors[], unsigned int leftSensors[], unsigned int rightSensors[]) {

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

}

// Resolve intersections function
void arbitrateIntersection(unsigned int forwardSensors[], unsigned int leftSensors[], unsigned int rightSensors[]) {

  boolean leftInt = false;
  boolean forwardInt = false;
  boolean rightInt = false;

  int intersectionCounter = 0;
  int driveForwardDelay = 375;

  int forwardAverage = averageLineReading(forwardSensors);
  int leftAverage = averageLineReading(leftSensors);
  int rightAverage = averageLineReading(rightSensors);

  // If intersection on left side
  if (leftAverage > minLineValue) {
    intersectionCounter++;
    leftInt = true;
  }
  // If intersection on forward side
  if (forwardAverage > minLineValue + 100) {
    intersectionCounter++;
    forwardInt = true;
  }
  // If intersection on right side
  if (rightAverage > minLineValue) {
    intersectionCounter++;
    rightInt = true;
  }

  // Intersection cases
  switch (intersectionCounter) {

    // One line detected, turn in direction of line detected
    case 1:

      if (leftInt == true) {

        lcd.setCursor(0, 1);
        lcd.print("Left!");

        driveForward();
        delay(driveForwardDelay);
        turnLeft();

      } else {


        lcd.setCursor(10, 1);
        lcd.print("Right!");

        driveForward();
        delay(driveForwardDelay);
        turnRight();

      }

      break;

    // Two lines detected, turn in left direction if true, or right direction if true
    // Assumes no forward dead ends
    case 2:

      if (leftInt == true) {

        lcd.setCursor(0, 1);
        lcd.print("Left!");

        driveForward();
        delay(driveForwardDelay);
        turnLeft();

      } else {

        lcd.setCursor(10, 1);
        lcd.print("Right!");

        driveForward();
        delay(driveForwardDelay);
        turnRight();

      }

      break;

    // If three lines detected, wait for direction to turn
    case 3:

      lcd.setCursor(3, 0);
      lcd.print("Waiting...");
      lcd.setCursor(2, 1);
      lcd.print("Pick a shape");

      // Stall while serial data from BT is unavailable
      while (!Serial1.available());

      incomingByte = Serial1.read();

      if (incomingByte == 'c' || incomingByte == 's' || incomingByte == 't' || incomingByte  == 'x') {

        lcd.clear();
        if (incomingByte == 'c') {
          lcd.setCursor(5, 0);
          lcd.print("Circle");
        } else if (incomingByte == 's') {
          lcd.setCursor(5, 0);
          lcd.print("Square");
        } else if (incomingByte == 't') {
          lcd.setCursor(4, 0);
          lcd.print("Triangle");
        } else if (incomingByte == 'x') {
          lcd.setCursor(5, 0);
          lcd.print("Cross");
        }

        Serial.print("pfcv");
        Serial.println(incomingByte);

        int turnDirection  = odroidCommand();

        // Turn left
        if (turnDirection == 1) {

          lcd.setCursor(0, 1);
          lcd.print("Left!");
          delay(500);

          driveForward();
          delay(driveForwardDelay);
          turnLeft();

          // Drive forward
        } else if (turnDirection == 2) {

          lcd.setCursor(4, 1);
          lcd.print("Straight!");
          delay(500);

          driveForward();
          delay(200);

          // Turn right
        } else if (turnDirection == 3) {

          lcd.setCursor(10, 1);
          lcd.print("Right!");
          delay(500);

          driveForward();
          delay(driveForwardDelay);
          turnRight();

        } else {
          
          lcd.setCursor(3, 1);
          lcd.print("Not Found!");
          delay(1000);
          
        }

        Serial1.print("pfcv");
        Serial1.println(incomingByte);

      }

      break;

  }

  // Reset counters
  sensorReadCount = 0;
  intCounter = 0;

}

// Drive forward function
void driveForward() {

  // Drive forward at steady speed
  qik.setSpeeds(steadySpeed, -steadySpeed);
  delay(100);

}

// Turn left function
void turnLeft() {

  // Turn left for 400 ms
  qik.setSpeeds(-steadySpeed, -steadySpeed);
  delay(400);

  // Determine forward line average
  unsigned int sensors[3];
  qtrForward.readCalibrated(sensors);
  int forwardLine = averageLineReading(sensors);

  // Turn left until forward line average above some minimum value
  while (forwardLine < minLineValue + 100) {

    qik.setSpeeds(-steadySpeed, -steadySpeed);

    qtrForward.readCalibrated(sensors);
    forwardLine = averageLineReading(sensors);

  }

}

// Turn right function
void turnRight() {

  // Turn right for 400 ms
  qik.setSpeeds(steadySpeed, steadySpeed);
  delay(400);

  // Determine forward line average
  unsigned int sensors[3];
  qtrForward.readCalibrated(sensors);
  int forwardLine = averageLineReading(sensors);

  // Turn right until forward line average above some minimum value
  while (forwardLine < minLineValue + 100) {

    qik.setSpeeds(steadySpeed, steadySpeed);

    qtrForward.readCalibrated(sensors);
    forwardLine = averageLineReading(sensors);

  }

}

// Get average of three sensor readings
int averageLineReading(unsigned int sensorReadings[]) {

  int averageReading = sensorReadings[0] + sensorReadings[1] + sensorReadings[2];
  averageReading = averageReading / 3;

  return averageReading;

}

// Get maximum value of three sensor readings
int maxLineReading(unsigned int sensorReadings[]) {

  int maxReading;

  for (int i = 0; i < 2; i++) {
    maxReading = max(sensorReadings[i], sensorReadings[i + 1]);
  }

  return maxReading;

}

// Get minimum value of three sensor readings
int minLineReading(unsigned int sensorReadings[]) {

  int minReading;

  for (int i = 0; i < 2; i++) {
    minReading = min(sensorReadings[i], sensorReadings[i + 1]);
  }

  return minReading;

}
