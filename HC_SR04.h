#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

// This can be used with the HC-SR04 ultrasonic sensor
class HC_SR04 {
  public:
    HC_SR04(int triggerPin, int echoPin); // Constructor
    void begin(); // Intializes the sensor
    float getDistance(); // Measure the distance to the nearest obstacle
  
  private:
    const int triggerPin; // Pin used to send a signal to the sensor
    const int echoPin; // Pin used to receive a signal from the sensor

    void sendPulse(); // Sends a pulse to trigger the sensor
    unsigned long measureTime(); // Measures the time it takes sound to come back to sensor
    float microsecondsToCentimeters(unsigned long microseconds); // Converts time (microsec) to distance (cm)
};

#endif 