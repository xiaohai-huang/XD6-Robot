#ifndef MyAccelStepper_h
#define MyAccelStepper_h

#include <Arduino.h>

constexpr unsigned int MIN_PULSE_WIDTH = 1; // Minimum pulse width in microseconds

class MyAccelStepper
{
public:
    MyAccelStepper(uint8_t stepPin, uint8_t dirPin);

    void moveTo(long absolute);
    void moveBy(long relative);
    /// @brief Sets the maximum speed in steps per second.
    /// @param speed steps per second
    void setMaxSpeed(float speed);

    /// @brief Sets the acceleration/deceleration rate in steps per second squared.
    /// @param acceleration acceleration in steps per second squared
    void setAcceleration(float acceleration);

    /// @brief The distance from the current position to the target position.
    /// @return In steps. positive is clockwise, negative is counter-clockwise from the current position.
    long distanceToGo();

    long currentPosition();

    void setCurrentPosition(long position);

    void stop();

    void eStop();

    boolean run();

    boolean runWithCurrentStepInterval();

private:
    uint8_t _directionPin; // Pin to control the direction of the stepper motor
    uint8_t _stepPin;      // Pin to send step pulses to the stepper
    typedef enum
    {
        DIRECTION_CCW = 0,
        DIRECTION_CW = 1
    } Direction;

    /// @brief The current direction of the stepper motor. true == CW
    boolean _direction;

    unsigned long _stepInterval; // The time between steps in microseconds
    float _minStepInterval;      // The minimum step interval in microseconds, calculated from the maximum speed

    long _currentPosition; // The current position in steps
    long _targetPosition;  // The target position in steps
    float _speed;          // The current speed in steps per second
    float _maxSpeed;       // The maximum speed in steps per second
    float _acceleration;   // The acceleration in steps per second squared

    long _currentStep;
    float _initialStepInterval;  // The initial step interval in microseconds
    float _currentStepInterval;  // The current step interval in microseconds
    unsigned long _lastStepTime; // The last time a step was made in microseconds

    /// @brief Computes the next step interval based on the current speed and acceleration. And updates the current speed.
    /// @return
    unsigned long calculateNextStepInterval();

    void step();
};

#endif