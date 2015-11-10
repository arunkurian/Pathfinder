/*
  Pathfinder Arduino - Motor Control
  v4 Wizard

  Motor control functions. PID closed loop position rotations and translations

  modified 9 Nov 2015
  by Arun Kurian
 */

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
  int radiusBody = 55;                     // Define half distance between wheels

  float radiusRatio = (float)radiusBody / (float)radiusWheel;              // Find ratio of radii
  
  // Calculate angles from desired body rotation and ratio of radii
  float leftAngle = -(float)desiredAngle * radiusRatio;
  float rightAngle = -(float)desiredAngle * radiusRatio;

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
  float KpLeft = 0.0075;
  float KpRight = 0.0073;
  float Ki = 0.0007;
  float Kd = 0.0055;

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
  while (abs(leftError) > 3 || abs(rightError) > 3) {

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
    qik.setSpeeds((int)leftMotorCommand, (int)rightMotorCommand);

    // Delay loop time for consistency
    delay(3);

  }

  // Stop motors when desired angles achieved
  qik.setSpeeds(0, 0);

}
