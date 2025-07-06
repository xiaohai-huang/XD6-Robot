#pragma once

#include "Arduino.h"
#pragma push_macro("abs")
#undef abs

#include "timers/interfaces.h"
#include "timers/timerfactory.h"
#include <algorithm>
#include <cstdint>
#include <string>

namespace TS4
{
    /**
     * @class StepperBase
     * @brief Manages the motion of a single stepper motor or a group of synchronized steppers.
     *
     * This class implements a trapezoidal velocity profile for point-to-point moves,
     * ensuring smooth acceleration and deceleration. It uses an efficient kinematic equation
     * to calculate step timings without requiring floating-point arithmetic in the ISR.
     *
     * Multi-motor synchronization is achieved using a linked list and a Bresenham-like
     * algorithm, allowing for coordinated linear motion.
     */
    class StepperBase
    {
    public:
        std::string name;
        bool isMoving = false;
        void emergencyStop();
        void overrideSpeed(float factor);

    protected:
        StepperBase(const int stepPin, const int dirPin);

        // Motion planning methods
        void startMoveTo(int32_t target_steps, int32_t end_velocity, uint32_t max_velocity, uint32_t acceleration);
        void startRotate(int32_t target_velocity, uint32_t acceleration);
        void startStopping(int32_t end_velocity_for_stop, uint32_t acceleration);

        inline void setDirection(int d);

        // Core kinematic variables
        int32_t direction_multiplier;      // -1 for reverse, 1 for forward
        int32_t velocity_change_direction; // -1 for decelerating, 1 for accelerating

        volatile int32_t current_position_steps = 0; // Absolute position in steps from origin
        volatile int32_t target_position_steps;      // The final absolute position for the move

        // Motion profile parameters for the current move
        int32_t total_steps_for_move;     // Total steps in the current movement
        int32_t target_velocity;          // The cruise velocity for the move (steps/sec)
        int32_t target_velocity_original; // Stored original target velocity before any overrides
        int64_t target_velocity_sqr;      // Square of the cruise velocity

        int32_t two_times_acceleration;  // Pre-calculated 2 * acceleration for kinematic formula
        int32_t deceleration_start_step; // The step count at which deceleration should begin
        int32_t acceleration_end_step;   // The step count at which acceleration phase ends

        // Volatile state variables updated inside ISR
        volatile int32_t steps_traveled;       // Number of steps taken in the current move
        volatile int32_t current_velocity;     // Current velocity in steps/sec
        volatile int64_t current_velocity_sqr; // Square of the current velocity

        inline void doStep();

        const int stepPin, dirPin;

        // Timer and ISR management
        ITimer *step_timer;
        inline void stepISR();  // ISR for position-based moves (Trapezoidal Profile)
        inline void rotISR();   // ISR for velocity-based moves (Rotate)
        inline void resetISR(); // ISR to reset step pins to LOW

        enum class move_mode_t
        {
            TARGET_POSITION,   // Move to a specific target position
            ROTATE_CONTINUOUS, // Rotate at a target velocity
            STOPPING           // Decelerating to a stop
        } move_mode = move_mode_t::TARGET_POSITION;

        // --- Bresenham's Algorithm for Multi-Motor Synchronization ---
        // This motor acts as the "master" in a linked list of steppers.
        StepperBase *next_stepper = nullptr; // Points to the next "slave" stepper in the chain
        int32_t bresenham_A;                 // The distance the slave motor should travel (Δy)
        int32_t bresenham_B;                 // The error accumulator for the slave motor

        friend class StepperGroupBase;
    };

    //========================================================================================================
    // Inline Implementation
    //========================================================================================================

    /**
     * @brief Executes a single step and coordinates slave motors.
     *
     * This function sends a HIGH signal to the step pin and updates the motor's position.
     * It then iterates through a linked list of "slave" motors, stepping them if needed
     * based on a Bresenham-like line algorithm.
     *
     * Bresenham's Algorithm:
     * - 'this' motor is the master and always steps. Its travel distance is the major axis (A_master).
     * - Each 'next_stepper' is a slave. Its travel distance is the minor axis (A_slave).
     * - The error term 'B' accumulates the minor axis distance. When it exceeds half the
     * major axis distance, the slave motor steps, and the error is reduced by the major axis distance.
     * This ensures the motors maintain a straight-line path relative to each other.
     */
    void StepperBase::doStep()
    {
        digitalWriteFast(stepPin, HIGH);
        steps_traveled += 1;
        current_position_steps += direction_multiplier;

        StepperBase *stepper = next_stepper;
        while (stepper != nullptr) // Move slave motors if required
        {
            if (stepper->bresenham_B >= 0)
            {
                digitalWriteFast(stepper->stepPin, HIGH);
                stepper->current_position_steps += stepper->direction_multiplier;
                stepper->bresenham_B -= this->bresenham_A; // this->A is the master axis distance
            }
            stepper->bresenham_B += stepper->bresenham_A; // stepper->A is the slave axis distance
            stepper = stepper->next_stepper;
        }
    }

    /**
     * @brief Interrupt Service Routine (ISR) for position-controlled moves.
     *
     * Implements a trapezoidal velocity profile using a kinematic equation.
     * The formula v_f^2 = v_i^2 + 2*a*d is used to update velocity.
     * Since distance 'd' is always 1 step, the formula simplifies to:
     * new_v_sqr = old_v_sqr + 2*a
     *
     * This avoids costly square root operations on every step, only requiring it
     * once per step to set the timer frequency.
     *
     * The move is divided into three phases:
     * 1. Acceleration: Velocity increases until it reaches max_velocity or the halfway point.
     * 2. Constant Speed: Motor runs at max_velocity.
     * 3. Deceleration: Velocity decreases to reach zero at the target position.
     */
    void StepperBase::stepISR()
    {
        // Handle an external command to stop the motor mid-move.
        if (move_mode == move_mode_t::STOPPING)
        {
            move_mode = move_mode_t::TARGET_POSITION;
            if (steps_traveled < acceleration_end_step) // Stopping during acceleration phase
            {
                // We need to decelerate over the same distance we accelerated.
                acceleration_end_step = deceleration_start_step = 0;
                total_steps_for_move = 2 * steps_traveled;
            }
            else if (steps_traveled < deceleration_start_step) // Stopping during constant speed phase
            {
                // Start deceleration immediately.
                deceleration_start_step = 0;
                total_steps_for_move = steps_traveled + acceleration_end_step;
            }
        }

        // --- Trapezoidal Motion Profile ---
        if (steps_traveled < acceleration_end_step) // 1. Acceleration Phase
        {
            current_velocity_sqr += two_times_acceleration;
            current_velocity = signum(current_velocity_sqr) * sqrtf(std::abs(current_velocity_sqr));
            step_timer->updateFrequency(std::abs(current_velocity));
            doStep();
        }
        else if (steps_traveled < deceleration_start_step) // 2. Constant Speed (Cruise) Phase
        {
            current_velocity = std::min(sqrtf(current_velocity_sqr), sqrtf(target_velocity_sqr));
            step_timer->updateFrequency(current_velocity);
            doStep();
        }
        else if (steps_traveled < total_steps_for_move) // 3. Deceleration Phase
        {
            current_velocity_sqr -= two_times_acceleration;
            current_velocity = signum(current_velocity_sqr) * sqrtf(std::abs(current_velocity_sqr));
            step_timer->updateFrequency(std::abs(current_velocity));
            doStep();
        }
        else // Target Reached
        {
            step_timer->stop();
            TimerFactory::returnTimer(step_timer);
            step_timer = nullptr;

            // Clean up the linked list of slave motors.
            auto *cur = this;
            while (cur != nullptr)
            {
                auto *tmp = cur->next_stepper;
                cur->next_stepper = nullptr;
                cur = tmp;
            }
            isMoving = false;
        }
    }

    /**
     * @brief Interrupt Service Routine (ISR) for velocity-controlled (continuous rotation) moves.
     *
     * This ISR accelerates or decelerates the motor to a target velocity and maintains it.
     * It uses the same v_f^2 = v_i^2 + 2ad kinematic equation as stepISR.
     */
    void StepperBase::rotISR()
    {
        move_mode = move_mode_t::ROTATE_CONTINUOUS;
        int32_t velocity_absolute;

        // Check if target speed has been reached
        if (std::abs(current_velocity_sqr - target_velocity_sqr) > two_times_acceleration)
        {
            // Accelerate or decelerate towards the target velocity
            current_velocity_sqr += velocity_change_direction * two_times_acceleration;

            direction_multiplier = signum(current_velocity_sqr);
            digitalWriteFast(dirPin, direction_multiplier > 0 ? HIGH : LOW);
            delayMicroseconds(5); // Delay for direction pin to settle

            velocity_absolute = sqrtf(std::abs(current_velocity_sqr));
            step_timer->updateFrequency(velocity_absolute);
            doStep();
        }
        else // Target velocity reached
        {
            direction_multiplier = signum(current_velocity_sqr);
            digitalWriteFast(dirPin, direction_multiplier > 0 ? HIGH : LOW);
            delayMicroseconds(5);

            if (target_velocity != 0)
            {
                // Maintain constant velocity
                velocity_absolute = sqrtf(std::abs(current_velocity_sqr));
                step_timer->updateFrequency(velocity_absolute);
                doStep();
            }
            else // Target velocity is 0, so stop the motor.
            {
                step_timer->stop();
                TimerFactory::returnTimer(step_timer);
                step_timer = nullptr;
                current_velocity_sqr = 0;

                // Clean up the linked list of slave motors.
                auto *cur = this;
                while (cur != nullptr)
                {
                    auto *tmp = cur->next_stepper;
                    cur->next_stepper = nullptr;
                    cur = tmp;
                }
                isMoving = false;
            }
        }
    }

    /**
     * @brief Resets the step pin to LOW for all motors in the sync group.
     *
     * This is typically called by the timer after the step pulse duration has passed.
     */
    void StepperBase::resetISR()
    {
        StepperBase *stepper = this;
        while (stepper != nullptr)
        {
            digitalWriteFast(stepper->stepPin, LOW);
            stepper = stepper->next_stepper;
        }
    }
}
#pragma pop_macro("abs")