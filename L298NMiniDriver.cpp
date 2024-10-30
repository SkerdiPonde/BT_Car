#include "L298NMiniDriver.h"

// Constructor initializes all member variables
L298NMiniDriver::L298NMiniDriver(int leftFwPin, int leftBwPin, int rightFwPin, int rightBwPin)
  : leftFwPin(leftFwPin), leftBwPin(leftBwPin), rightFwPin(rightFwPin), rightBwPin(rightBwPin) {}

// Initializes all pins connected to the driver to output
void L298NMiniDriver::begin() {
  ledcAttach(leftFwPin, freq, resolution);
  ledcAttach(leftBwPin, freq, resolution);
  ledcAttach(rightFwPin, freq, resolution);
  ledcAttach(rightBwPin, freq, resolution);
  stop();
}

// Moves the robot forward based on the given analog value
void L298NMiniDriver::moveForward(int speed) {
  // Set backward movement pins (int2 and int4) to 0 first
  ledcWrite(leftBwPin, 0);
  ledcWrite(rightBwPin, 0);

  // Set the speed to the forward movement pins(int1 and int3)
  ledcWrite(leftFwPin, speed);
  ledcWrite(rightFwPin, speed);
}

// Moves the robot backward based on the given analog value
void L298NMiniDriver::moveBackward(int speed) {
  // Set forward movement pins (int1 and int3) to 0 first
  ledcWrite(leftFwPin, 0);
  ledcWrite(rightFwPin, 0);

  // Set the speed to the backward movement pins(int2 and int4)
  ledcWrite(leftBwPin, speed);
  ledcWrite(rightBwPin, speed);
}

// Rotates the robot counter-clockwise
void L298NMiniDriver::turnLeft(int speed) {
  // Stop left forward movement and right backward movement
  ledcWrite(leftFwPin, 0);
  ledcWrite(rightBwPin, 0);

  // Left wheel should roll backward and right wheel forward
  // for a left turn around the center of the wheel "axis"
  ledcWrite(leftBwPin, speed);
  ledcWrite(rightFwPin, speed);
}

// Rotates the robot clockwise
void L298NMiniDriver::turnRight(int speed) {
  // Stop left backward movement and right forward movement
  ledcWrite(leftBwPin, 0);
  ledcWrite(rightFwPin, 0);

  // Left wheel should roll forward and right wheel backward
  // for a right turn around the center of the wheel "axis"
  ledcWrite(leftFwPin, speed);
  ledcWrite(rightBwPin, speed);
}

// Stops the robot
void L298NMiniDriver::stop() {
  // Stop both wheels
  ledcWrite(leftFwPin, 0);
  ledcWrite(leftBwPin, 0);

  ledcWrite(rightFwPin, 0);
  ledcWrite(rightBwPin, 0);
}

// Sets independent speeds to the motors
void L298NMiniDriver::setSpeeds(int speedLeft, int dirLeft, int speedRight, int dirRight) {
  // If dirLeft set the analog value to the forward pin otherwise backward
  if (dirLeft) {
    ledcWrite(leftBwPin, 0);
    ledcWrite(leftFwPin, speedLeft);
  }
  else {
    ledcWrite(leftFwPin, 0);
    ledcWrite(leftBwPin, -speedLeft);
  }

  // If dirRight set the analog value to the forward pin otherwise backward
  if (dirRight) {
    ledcWrite(rightBwPin, 0);
    ledcWrite(rightFwPin, speedRight);
  }
  else {
    ledcWrite(rightFwPin, 0);
    ledcWrite(rightBwPin, -speedRight);
  }
}
