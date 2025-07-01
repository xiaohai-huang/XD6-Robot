#ifndef MOTOR_INITIALIZATION_H
#define MOTOR_INITIALIZATION_H

#include "Motor.h"

// Declare the number of motors
extern const int NUM_MOTORS;

// Declare the motors array
extern Motor motors[];

// Declare functions to initialize and update all motors
void initializeAllMotors();
void updateAllMotors();

#endif // MOTOR_INITIALIZATION_H
