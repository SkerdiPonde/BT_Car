#ifndef L298NMINIDRIVER_H
#define L298NMINIDRIVER_H

#include <Arduino.h>

// This can be used with the L298N Mini Driver
class L298NMiniDriver {
  public:
    L298NMiniDriver(int leftFwPin, int leftBwPin, int rightFwPin, int rightBwPin); // Constructor
    void begin(); // Initializes the pins
    void moveForward(int speed); // Moves the robot forward
    void moveBackward(int speed); // Moves it backward
    void turnLeft(int speed); // Rotates it counter-clockwise
    void turnRight(int speed); // Rotates it clockwise
    void stop(); // Stops it
    void setSpeeds(int speedLeft, int dirLeft, int speedRight, int dirRight); // Sets independed values
  
  private:
    const int leftFwPin; // Int1 Pin - left motor forward
    const int leftBwPin; // Int2 Pin - left motor backward
    const int rightFwPin; // Int3 Pin - right motor forward
    const int rightBwPin; // Int4 Pin - right motor backward

    const int freq = 15000;
    const int resolution = 8;
};

#endif 