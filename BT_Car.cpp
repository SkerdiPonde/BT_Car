#include "BT_Car.h"
#include <math.h>
#include <algorithm>

// Constructor initializes the member variables
BT_Car::BT_Car(int triggerPin, int echoPin, int leftFwPin, int leftBwPin, int rightFwPin, int rightBwPin, int irLeft, int irRight)
  : distanceSensor(triggerPin, echoPin), motorDriver(leftFwPin, leftBwPin, rightFwPin, rightBwPin), ps3(Ps3), irLeft(irLeft), irRight(irRight){
    // for (int i = 0; i < numReadings; i++) {
    //   distanceReadings[i] = 0;
    // }
  }

// Calls the begin methods of each component to configure the pins
void BT_Car::begin(String macAddress, int player) {
  this->macAddress = macAddress; // Store the mac address in the member variable
  this->player = player; // Store the player number
  ps3.begin(macAddress.c_str()); // Begin the connection to the controller with the given mac address as a const char*
  ps3.setPlayer(player); // Set the player nubmer to the controller

  distanceSensor.begin(); // Initialize the distance sensor
  motorDriver.begin(); // Initialize the motor driver

  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
}

// Read the distance from the HC_SR04 distance sensor
float BT_Car::getDistance(const unsigned long callInterval) {
  unsigned long currentMillis = millis();
  if (currentMillis - prevCallDistance >= callInterval) {
    prevCallDistance = currentMillis;

    // Get new distance reading from the sensor
    cachedDistance = distanceSensor.getDistance();
    if (cachedDistance > 800) {
      cachedDistance = 2;
    }
  }
  // Return the new or cached value if not enough time has passed
  return cachedDistance;
}

// Check for the joystick position
void BT_Car::readJoystick(unsigned long callInterval) {
  unsigned long currentMillis = millis();
  if (currentMillis - prevCallJoy >= callInterval) {
    prevCallJoy = currentMillis;
    // Read values from the left analog joystick
    leftJoyX = ps3.data.analog.stick.lx;
    leftJoyY = ps3.data.analog.stick.ly;
  }
}

// Update the speeds of both motors
void BT_Car::updateMotorSpeeds(unsigned long callInterval) {
  unsigned long currentMillis = millis();
  if (currentMillis - prevCallSpeed >= callInterval) {
    prevCallSpeed = currentMillis;

    int X = map(leftJoyX, -127, 128, -256, 255);
    int Y = map(leftJoyY, 127, -128, -256, 255);
    
    // Removed for now because it makes controlling it feel weird
    // // Don't mind the numbers
    // // This is so that the motors disengage before actually reaching the safety distance
    // // based on the Y value of the joystick
    // if ((cachedDistance - 0.00000038*Y*Y*Y <= safetyDistance) && (Y > 0)) {
    //   motorDriver.stop();
    //   return;
    // }

    if (X <= deadzone && X >= -deadzone) {
      if (Y <= deadzone && Y >= -deadzone) {
        motorDriver.stop();
      }
      else if (Y > deadzone) {
        motorDriver.moveForward(Y);
      }
      else {
        motorDriver.moveBackward(-Y);
      }
    }
    else {
      int leftSpeed = constrain(Y + X, -255, 255);
      int rightSpeed = constrain(Y - X, -255, 255);

      int dirLeft = leftSpeed > 0 ? 1 : 0;
      int dirRight = rightSpeed > 0 ? 1 : 0;

      if (abs(leftSpeed - prevLeftSpeed) > significantChange ||
          abs(rightSpeed - prevRightSpeed) > significantChange) {
        motorDriver.setSpeeds(leftSpeed, dirLeft, rightSpeed, dirRight);
        prevLeftSpeed = leftSpeed;
        prevRightSpeed = rightSpeed;
      }
    }
  }
}

// Getter emthod to access the ps3 (Ps3) object
Ps3Controller& BT_Car::ps3Get() {
  return ps3;
}

// Function to retreive the current opreating mode of the robot car
int BT_Car::modeGet() {
  return mode;
}

// Function to change the operating mode to a new one
void BT_Car::modeSet(int newMode) {
  mode = newMode;
}

// Perform line following
void BT_Car::lineFollowing(unsigned long callInterval) {
  unsigned long currentMillis = millis();
  if (currentMillis - prevCallLine >= callInterval) {
    prevCallLine = currentMillis;

    int irLeftVal = digitalRead(irLeft);
    int irRightVal = digitalRead(irRight);

    if (!irLeftVal && !irRightVal) {
      motorDriver.moveForward(220);
    }
    else if (irLeftVal && !irRightVal) {
      // motorDriver.setSpeeds(0, 1, 150, 1);
      motorDriver.turnLeft(200);
    }
    else if (!irLeftVal && irRightVal) {
      // motorDriver.setSpeeds(150, 1, 0, 1);
      motorDriver.turnRight(200);
    }
    else {
      motorDriver.stop();
    }
  }
}

// Updates the desired distance when L1 or L2 are pressed and returns it
int BT_Car::updateDesiredDistance(unsigned long callInterval) {
  unsigned long currentMillis = millis();
  
  if (currentMillis - prevCallDesiredDistance >= callInterval) {
    prevCallDesiredDistance = currentMillis;
    if (ps3.data.button.r2) {
      desiredDistance++;
    }
    else if (ps3.data.button.r1) {
      desiredDistance--;
    }
    desiredDistance = constrain(desiredDistance, 10, 40);
  }
  return desiredDistance;
}

// Set a custom desired distance
void BT_Car::setDesiredDistance(int newDesiredDistance) {
  desiredDistance = newDesiredDistance;
}

// Keep a desired distance from the wall/obstacle in front of the robot
int BT_Car::keepDistance(unsigned long callInterval) {
  unsigned long currentMillis = millis();
  if (currentMillis - prevCallPID >= callInterval) {
    unsigned long deltaTime = currentMillis - prevCallPID;
    prevCallPID = currentMillis;

    // Convert deltaTime from milliseconds to seconds
    float dt = deltaTime / 1000.0;

    // Calculate the error
    float error = desiredDistance - cachedDistance;

    // Proportional Term
    float proportionalTerm = Kp * error;

    // Integral Term
    integral += error * dt;
    float integralTerm = Ki * integral;

    // Derivative Term
    float derivativeTerm = Kd * (error - previousError) / dt;
    previousError = error; // Update the previous error

    // The control signal is a combination of all components above
    controlOutput = -(int)(proportionalTerm + integralTerm + derivativeTerm);

    // Constrain the values within the -255 to 255 range
    controlOutput = constrain(controlOutput, -255, 255);

    if (controlOutput > 0) {
      motorDriver.moveForward(controlOutput);
    }
    else {
      motorDriver.moveBackward(-controlOutput);
    }

  }
  return controlOutput;
}


void BT_Car::resetPID() {
  integral = 0.0;
  previousError = 0;
  prevCallPID = millis();
}


// if ((cachedDistance <= safetyDistance) || (cachedDistance > 800)) {
//   motorDriver.moveBackward(255);
//   return;
// }
// else if ((cachedDistance > safetyDistance) && (cachedDistance <= safetyDistance + 5))  {
//   motorDriver.stop();
//   return;
// }
// else {
//   motorDriver.moveForward(255);
//   return;
// }


// Not going to use a filter for now, since the actual distance is experienced late and that hinders the performance
// for stopping the robot in time
// float BT_Car::getDistance(const unsigned long callInterval) {
//   unsigned long currentMillis = millis();
//   if (currentMillis - prevCallDistance >= callInterval) {
//     prevCallDistance = currentMillis;

//     // Get new distance reading from the sensor
//     float newDistance = distanceSensor.getDistance();

//     // Add the new value to the distance array
//     distanceReadings[currentReadingIndex] = newDistance;
//     currentReadingIndex = (currentReadingIndex + 1) % numReadings; // Update the index

//     // Create a copy to store the values for sorting
//     float sortedReadings[numReadings];
//     for (int i = 0; i < numReadings; i++) {
//       sortedReadings[i] = distanceReadings[i];
//     }

//     // Sort the array
//     std::sort(sortedReadings, sortedReadings+numReadings);

//     // Store the median distance 
//     cachedDistance = sortedReadings[numReadings/2];
//   }
//   // Return the new or cached value if not enough time has passed
//   return cachedDistance;
// }



