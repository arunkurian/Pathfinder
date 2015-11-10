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
  lcd.setCursor(0, 0);
  lcd.print("Line Calibrating");

  // Drive forward slowly open loop
  qik.setSpeeds(-30, -30);

  // Calibrate for 1.25 seconds
  for (int i = 0; i < numIterations; i++) {

    // Calibrate each of the three line sensors
    qtrForward.calibrate();
    qtrRight.calibrate();
    qtrLeft.calibrate();

    // Display percent complete
    lcd.setCursor(0, 1);
    percent = 100 * i /  (numIterations - 1);
    percent = percent / 2.0;
    lcd.print((int)percent);
    lcd.print(" % ");

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
    lcd.setCursor(0, 1);
    percent = 100 * i / (numIterations - 1);
    percent = percent / 2.0 + 50.0;
    lcd.print((int)percent);
    lcd.print(" % ");

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

  // Clear LCD and print averaged min and max line IR values
  lcd.clear();
  lcd.print(leftCalMin);
  lcd.setCursor(5, 0);
  lcd.print(forwardCalMin);
  lcd.setCursor(11, 0);
  lcd.print(rightCalMin);
  lcd.setCursor(0, 1);
  lcd.print(leftCalMax);
  lcd.setCursor(5, 1);
  lcd.print(forwardCalMax);
  lcd.setCursor(11, 1);
  lcd.print(rightCalMax);

}

// Resolve intersections function
void arbitrateIntersection(unsigned int forwardSensors[], unsigned int leftSensors[], unsigned int rightSensors[]) {

  boolean leftInt = false;
  boolean forwardInt = false;
  boolean rightInt = false;

  int intersectionCounter = 0;
  int driveForwardDelay = 375;

  delay(500);

  int forwardAverage = averageLineReading(forwardSensors);
  int leftAverage = averageLineReading(leftSensors);
  int rightAverage = averageLineReading(rightSensors);

  lcd.clear();
  lcd.print(leftAverage);
  lcd.setCursor(5, 0);
  lcd.print(forwardAverage);
  lcd.setCursor(11, 0);
  lcd.print(rightAverage);

  // If intersection on left side
  if (leftAverage > minLineValue) {
    intersectionCounter++;
    leftInt = true;

    lcd.setCursor(0, 1);
    lcd.print("X");
  }
  // If intersection on forward side
  if (forwardAverage > minLineValue) {
    intersectionCounter++;
    forwardInt = true;

    lcd.setCursor(8, 1);
    lcd.print("X");
  }
  // If intersection on right side
  if (rightAverage > minLineValue) {
    intersectionCounter++;
    rightInt = true;

    lcd.setCursor(15, 1);
    lcd.print("X");
  }

  delay(500);

  // Intersection cases
  switch (intersectionCounter) {

    // One line detected, turn in direction of line detected
    case 1:

      if (leftInt == true) {

        lcd.setCursor(1, 1);
        lcd.print("<--");
        turnLeft();

      } else {

        lcd.setCursor(12, 1);
        lcd.print("-->");
        turnRight();

      }

      break;

    // Two lines detected, turn in left direction if true, or right direction if true
    // Assumes no forward dead ends
    case 2:

      if (leftInt == true) {

        lcd.setCursor(1, 1);
        lcd.print("<--");

        driveForward();
        delay(driveForwardDelay);
        turnLeft();

      } else {

        lcd.setCursor(12, 1);
        lcd.print("-->");

        driveForward();
        delay(driveForwardDelay);
        turnRight();

      }

      break;

    // If three lines detected, wait for direction to turn
    case 3:

      // Stall while serial data is unavailable
      while (!Serial1.available());

      // Read incoming byte
      incomingByte = Serial1.read();

      // Turn left
      if (incomingByte == 'a') {

        lcd.setCursor(1, 1);
        lcd.print("<--");
        delay(500);

        driveForward();
        delay(driveForwardDelay);
        turnLeft();

      // Drive forward
      } else if (incomingByte == 'w') {

        lcd.setCursor(6, 1);
        lcd.print("^^");
        delay(500);

        driveForward();
        delay(200);

      // Turn right
      } else if (incomingByte == 'd') {

        lcd.setCursor(12, 1);
        lcd.print("-->");
        delay(500);

        driveForward();
        delay(driveForwardDelay);
        turnRight();

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
