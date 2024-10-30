#include <Ps3Controller.h>
#include "BT_Car.h"

// ESP-32 ON led indicator pin
const int ps3LedPin = 2;
const int espLedPin = 4;

// Motor Driver (L298N Mini) pins 
const int leftFwPin = 27; 
const int leftBwPin = 14;
const int rightFwPin = 12; 
const int rightBwPin = 13;

// Distance sensor (HC_SR04) pins
const int triggerPin = 32;
const int echoPin = 33;

// Infrared sensor pins
const int irLeft = 34;
const int irRight = 35;

// Ps3 controller mac address and player number
const char* macAddress = "00:00:00:00:00:00";
int playerNumber = 1;

// Create BT_Car instance
BT_Car car(triggerPin, echoPin, leftFwPin, leftBwPin, rightFwPin, rightBwPin, irLeft, irRight);

// Distance from the front sensor to the nearest obstacle
float frontDistance;
int desiredDistance;

int controlValue;

// Value update intervals
const unsigned long distanceInterval = 60;  // Distance check interval
const unsigned long joystickInterval = 40;  // Joystick analog values check interval
const unsigned long commandInterval =  40;  // Motors' speed update interval

const unsigned long lineFollowingInterval = 20;

const unsigned long distancePIDInterval = 30;

const unsigned long desiredDistanceInterval = 100;

// Turns on indicator LED when the ps3 controller is connected
void turnOnLedCallback() {
  pinMode(ps3LedPin, OUTPUT);
  digitalWrite(ps3LedPin, HIGH);
}

// Checks which of the following buttons is pressed to change
// the operating mode of the robot car
void checkModeCallback() {
  if (car.ps3Get().data.button.circle) {
    car.modeSet(0);
  }
  else if (car.ps3Get().data.button.cross) {
    car.modeSet(1);
  }
  else if (car.ps3Get().data.button.triangle) {
    car.resetPID();
    car.modeSet(2);
  }
}

void setup() {
  // Begin Serial communication
  Serial.begin(115200);
  // Initialize the ON led indicator pin and set it to HIGH
  pinMode(espLedPin, OUTPUT);
  digitalWrite(espLedPin, HIGH);

  analogReadResolution(10);

  // Initialize the BT_Car instance
  car.begin(macAddress, playerNumber);
  car.ps3Get().attachOnConnect(turnOnLedCallback);
  car.ps3Get().attach(checkModeCallback);
}

void loop() {
  // Check if the controller is connected to the car
  // does not behave properly for some reason
  // It does not return false when the controller turns off or disconnects
  // but works properly for the initial connection
  if (!car.ps3Get().isConnected()) {
    return;
  }

  int mode = car.modeGet();
  
  switch (mode) {
    case 0:
      frontDistance = car.getDistance(distanceInterval);
      Serial.print(frontDistance);
      Serial.print(" ");
      Serial.println("cm");
    
      car.readJoystick(joystickInterval);
      car.updateMotorSpeeds(commandInterval);
      break;
    
    case 1:
      car.lineFollowing(lineFollowingInterval);
      break;

    case 2:
      frontDistance = car.getDistance(distanceInterval);
      desiredDistance = car.updateDesiredDistance(desiredDistanceInterval);
      controlValue = car.keepDistance(distancePIDInterval);
      Serial.print(frontDistance);
      Serial.print(" ");
      Serial.print(desiredDistance);
      Serial.print(" ");
      // Just a trick so the y-axis of the Serial monitor does not change its scale
      Serial.print(0);
      Serial.print(" ");
      Serial.println(30);
      break;
  }
}