#ifndef BT_Car_H
#define BT_Car_H

#include <Ps3Controller.h>
#include "HC_SR04.h"
#include "L298NMiniDriver.h"

// Created a class for convenience
class BT_Car {
  public:
    // using callback_t = void(*)();
    BT_Car(int triggerPin, int echoPin, int leftFwPin, int leftBwPin, int rightFwPin, int rightBwPin, int irLeft, int irRight);
    void begin(String macAddress, int player); 

    Ps3Controller& ps3Get();

    void modeSet(int newMode);
    int modeGet();

    float getDistance(unsigned long callInterval); 
    void readJoystick(unsigned long callInterval);
    void updateMotorSpeeds(unsigned long callInterval);

    void lineFollowing(unsigned long callInterval);

    int updateDesiredDistance(unsigned long callInterval);
    void setDesiredDistance(int newDesiredDistance);
    int keepDistance(unsigned long callInterval);
    
    void resetPID();


  private:
    int mode = 0;

    HC_SR04 distanceSensor;

    L298NMiniDriver motorDriver;

    const int irLeft;
    const int irRight;

    Ps3Controller& ps3; // Will be initialized as a reference to Ps3 (global variable of "Ps3Controller.h")
    String macAddress; // Store the mac address the controller expects
    int player; // Store the player number

    int leftJoyX = 0;
    int leftJoyY = 0;

    unsigned long prevCallDistance = 0;
    unsigned long prevCallJoy = 0;
    unsigned long prevCallSpeed = 0;
    unsigned long prevCallLine = 0;
    unsigned long prevCallPID = 0;
    unsigned long prevCallDesiredDistance = 0;

    // // Variables needed for implementing the median filter to distance readings
    // static const int numReadings = 1;
    // float distanceReadings[numReadings];
    // int currentReadingIndex = 0;
    float cachedDistance = 0;
    int desiredDistance = 10;

    // Safety distance of the front sensor to the nearest obstacle
    float safetyDistance = 12.0;

    // Store the previous velocities of the motors
    int prevLeftSpeed = 0;
    int prevRightSpeed = 0;

    // Variables related to the motors' velocities
    int deadzone = 50;
    int significantChange = 20;

    // PID variables for keeping distance
    float Kp = 40.0;
    float Ki = 5.0;
    float Kd = 15.0;

    float previousError = 0;
    float integral = 0;

    int controlOutput;

    float proportionalTerm;
    float integralTerm;
    float derivativeTerm;
};

#endif 