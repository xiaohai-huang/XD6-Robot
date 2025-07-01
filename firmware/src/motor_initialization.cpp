#include "motor_initialization.h"

// Define the number of motors
const int NUM_MOTORS = 6;

// Define motor configurations
const MotorConfig motorConfigs[NUM_MOTORS] = {
    // step pin, direction pin, home switch pin, steps per revolution (long), max speed, and acceleration
    {17, 22, 27, 400L, 600.0, 400.0},       // Motor 1 configuration
    {16, 15, 14, 400L * 50L, 500.0, 500.0}, // Motor 2 configuration
    {17, 22, 27, 400L, 600.0, 400.0},       // Motor X configuration
    {17, 22, 27, 400L, 600.0, 400.0},       // Motor X configuration
    {17, 22, 27, 400L, 600.0, 400.0},       // Motor X configuration
    {17, 22, 27, 400L, 600.0, 400.0},       // Motor X configuration


};

// Define the motors array
Motor motors[NUM_MOTORS] = {
    {motorConfigs[0]}, // Motor 1
    {motorConfigs[1]}, // Motor 2
    {motorConfigs[2]}, // Motor X
    {motorConfigs[3]}, // Motor X
    {motorConfigs[4]}, // Motor X
    {motorConfigs[5]}, // Motor X

};

void initializeAllMotors() {
    motors[0].initialize();
    motors[1].initialize();
    motors[2].initialize();
    motors[3].initialize();
    motors[4].initialize();
    motors[5].initialize();
}

void updateAllMotors() {
    motors[0].update();
    motors[1].update();
    motors[2].update();
    motors[3].update();
    motors[4].update();
    motors[5].update();
}
