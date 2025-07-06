#include <Arduino.h>
#include "MyAccelStepper.h"

MyAccelStepper::MyAccelStepper(uint8_t stepPin, uint8_t dirPin)
{
    _directionPin = dirPin;
    _stepPin = stepPin;
    pinMode(_directionPin, OUTPUT);
    pinMode(_stepPin, OUTPUT);

    _speed = 0;
    _currentStep = 0;
    _initialStepInterval = 0;
    _currentStepInterval = 0;
    _minStepInterval = 0;
    _direction = DIRECTION_CCW;

    setAcceleration(1);
    setMaxSpeed(1);
}

void MyAccelStepper::setMaxSpeed(float newMaxSpeed)
{
    if (newMaxSpeed < 0.0)
    {
        newMaxSpeed = -newMaxSpeed;
    }

    if (_maxSpeed != newMaxSpeed)
    {
        _maxSpeed = newMaxSpeed;
        _minStepInterval = 1000000.0 / _maxSpeed; // Convert steps per <second> to microseconds per <step>

        // Recompute _currentStep from current speed and adjust speed if accelerating or cruising
        if (_currentStep > 0)
        {
            _currentStep = (long)((_speed * _speed) / (2 * _acceleration)); // equation 16
            calculateNextStepInterval();
        }
    }
}

void MyAccelStepper::setAcceleration(float acceleration)
{
    if (acceleration == 0.0)
        return;

    if (acceleration < 0.0)
    {
        acceleration = -acceleration;
    }

    if (_acceleration != acceleration)
    {
        // recompute current step per Equation 17
        _currentStep = _currentStep * (_acceleration / acceleration);
        // new initial step interval per Equation 7, with correction per Equation 15
        _initialStepInterval = 0.676 * sqrt(2.0 / acceleration) * 1000000.0; // Equation 15

        _acceleration = acceleration;
        calculateNextStepInterval();
    }
}

long MyAccelStepper::distanceToGo()
{
    return _targetPosition - _currentPosition;
}

long MyAccelStepper::currentPosition()
{
    return _currentPosition;
}

unsigned long MyAccelStepper::calculateNextStepInterval()
{
    long distanceRemaining = distanceToGo();

    // steps required to stop
    // steps = speed^2 / (2 * acceleration) // equation 16
    long stepsToStop = (_speed * _speed) / (2 * _acceleration);

    // Check if we are at the target position and should stop
    if (distanceRemaining == 0 && stepsToStop <= 1)
    {
        _currentStepInterval = 0;
        _speed = 0;
        _currentStep = 0;
        return _currentStepInterval;
    }

    // Check if we need to start decelerating
    if (distanceRemaining > 0)
    {
        if (_currentStep > 0 && (stepsToStop >= distanceRemaining || _direction == DIRECTION_CCW))
        {
            _currentStep = -stepsToStop;
        }
    }
    else if (distanceRemaining < 0)
    {
        if (_currentStep > 0 && (stepsToStop > -distanceRemaining || _direction == DIRECTION_CW))
        {
            _currentStep = -stepsToStop;
        }
    }

    // Calculate the Step Interval
    if (_currentStep == 0)
    {
        // first step, start from standstill
        _currentStepInterval = _initialStepInterval;
        _direction = (distanceRemaining > 0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    else
    {
        // Subsequent steps
        // interval_n = interval_n-1 - (2 * interval_n-1 ) / (4*currentStep + 1)
        _currentStepInterval = _currentStepInterval - (2 * _currentStepInterval) / (4 * _currentStep + 1);

        // Ensure the step interval does not go below the minimum
        // Make sure it satisfies the max speed constraint
        _currentStepInterval = max(_currentStepInterval, _minStepInterval);
    }

    _currentStep++;
    _speed = 1000000.0 / _currentStepInterval; // Convert microseconds to steps per second
    if (_direction == DIRECTION_CCW)
    {
        _speed = -_speed; // If the direction is counter-clockwise, make the speed negative
    }
    return _currentStepInterval;
}

void MyAccelStepper::step()
{
    // Set the direction based on the current direction
    digitalWrite(_directionPin, _direction ? HIGH : LOW);
    // Set the step pin high
    digitalWrite(_stepPin, HIGH);
    delayMicroseconds(MIN_PULSE_WIDTH); // Minimum pulse width
    // Set the step pin low
    digitalWrite(_stepPin, LOW);
}

boolean MyAccelStepper::runWithCurrentStepInterval()
{
    if (!_currentStepInterval)
    {
        return false; // No step interval set, nothing to run
    }

    unsigned long currentTime = micros();
    if (currentTime - _lastStepTime >= _currentStepInterval)
    {
        if (_direction == DIRECTION_CW)
        {
            _currentPosition++;
        }
        else
        {
            _currentPosition--;
        }
        step();                      // Perform the step
        _lastStepTime = currentTime; // Update the last step time
        return true;                 // Step was performed
    }
    else
    {
        return false; // Not enough time has passed to perform the next step
    }
}

boolean MyAccelStepper::run()
{
    if (runWithCurrentStepInterval())
    {
        calculateNextStepInterval(); // Update the step interval for the next run
    }

    return _speed != 0 || distanceToGo() != 0;
}

void MyAccelStepper::eStop()
{
    _speed = 0.0;
    _targetPosition = _currentPosition; // Stop at the current position
}

void MyAccelStepper::stop()
{
    if (_speed != 0.0)
    {
        long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)) + 1;

        if (_speed > 0)
        {
            moveBy(stepsToStop);
        }
        else
        {
            moveBy(-stepsToStop);
        }
    }
}

void MyAccelStepper::setCurrentPosition(long position)
{
    _targetPosition = position;
    _currentPosition = position;
    _currentStep = 0; // Reset the step count
    _currentStepInterval = 0;
    _speed = 0;
}

void MyAccelStepper::moveTo(long absolute)
{
    if (_targetPosition != absolute)
    {
        _targetPosition = absolute;
        calculateNextStepInterval();
    }
}

void MyAccelStepper::moveBy(long relative)
{
    moveTo(_currentPosition + relative);
}
