#ifndef MOTOR_H
#define MOTOR_H

#include <AccelStepper.h>
#include <Bounce2.h>

// Define a struct to hold motor configuration
struct MotorConfig
{
    int stepPin;
    int dirPin;
    int homeSwitchPin;
    long stepsPerRevolution;
    float maxSpeed;     // Maximum speed for the motor
    float acceleration; // Acceleration for the motor
};

class Motor
{
public:
    Motor(const MotorConfig &config);
    void initialize();
    void handleButtonPress();
    void update();
    void moveTo(long position);
    void moveByDegrees(float degrees); // New method to move by degrees
    void stop();
    void setSpeed(float speed);
    long getCurrentPosition();

private:
    AccelStepper stepper;
    Bounce2::Button homeSwitch;
    bool homeSwitchActive;
    int homeSwitchPin;
    long stepsPerRevolution;
};

#endif // MOTOR_H
