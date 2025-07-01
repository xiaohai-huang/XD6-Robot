#include "Motor.h"
#include <Arduino.h>

Motor::Motor(const MotorConfig &config)
    : stepper(AccelStepper::DRIVER, config.stepPin, config.dirPin),
      homeSwitchPin(config.homeSwitchPin),
      stepsPerRevolution(config.stepsPerRevolution),
      homeSwitchActive(false)
{
    stepper.setMaxSpeed(config.maxSpeed);         // Set max speed from config
    stepper.setAcceleration(config.acceleration); // Set acceleration from config
}

void Motor::initialize()
{
    stepper.setCurrentPosition(0);
    homeSwitch.attach(homeSwitchPin, INPUT_PULLUP);
    homeSwitch.interval(50);
    homeSwitch.setPressedState(LOW);
}

void Motor::handleButtonPress()
{
    Serial.println("Limit switch activated. Stopping motor.");
    stepper.stop();
    homeSwitchActive = true;
}

void Motor::update()
{
    homeSwitch.update();
    if (homeSwitch.pressed())
    {
        handleButtonPress();
    }
    else if (homeSwitchActive)
    {
        homeSwitchActive = false;
    }
    stepper.run();
}

void Motor::moveTo(long position)
{
    stepper.moveTo(position);
}

void Motor::moveByDegrees(float degrees)
{
    long steps = (long)((degrees / 360.0) * stepsPerRevolution); // Convert degrees to steps
    stepper.move(steps);
}

void Motor::stop()
{
    stepper.stop();
}

void Motor::setSpeed(float speed)
{
    stepper.setMaxSpeed(speed);
}

long Motor::getCurrentPosition()
{
    return stepper.currentPosition();
}
