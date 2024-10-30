#include "HC_SR04.h"

// Constructor assigns the given pin values to the member variables
HC_SR04::HC_SR04(int triggerPin, int echoPin)
  : triggerPin(triggerPin), echoPin(echoPin) {}

// Initializes the sensor by configuring the pins
void HC_SR04::begin() {
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

// Sends a 10 microsec pulse to the sensor through triggerPin
void HC_SR04::sendPulse() {
  // Make sure the voltage at triggerPin is LOW
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  // Send the pulse to the sensor
  // 10 microsec is the expected pulse duration for HC-SR04
  // When the pulse ends, the sensor will produce an ultrasonic wave
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
}

// Measures the time it takes the ultrasonic wave to reach the obstacle
// and return
unsigned long HC_SR04::measureTime() {
  // Send the pulse for the senor to produce the sound wave
  sendPulse(); 
  // When the sensor is triggered, the voltage at echoPin will turn HIGH
  // and it will stay so until the sound wave has returned to the sensor
  // Use the built-in function pulseIn() to measure the duration of the 
  // HIGH voltage at echoPin 
  return pulseIn(echoPin, HIGH);
}

// Converts the time (microsec) it takes sound to come back to the sensor to a distance (cm)
// It is assumed that the speed of sound is 343 meters/sec
float HC_SR04::microsecondsToCentimeters(unsigned long microseconds) {
  // 0.0343 cm is the distance sound travels in a microsecond
  // Divide by 2 since the sound travels double the distance
  return microseconds * 0.0343 / 2;
}


// Encapsulates all the previous functions
float HC_SR04::getDistance() {
  unsigned long duration = measureTime(); // Send pulse and measure time
  return microsecondsToCentimeters(duration); // Convert and return
}