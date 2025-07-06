#include "Arduino.h"

#pragma push_macro("abs")
#undef abs

#include "stepperbase.h" // Assuming this is the refactored header file
#include <algorithm>

namespace TS4
{
    // Constructor
    StepperBase::StepperBase(int _stepPin, int _dirPin)
        : steps_traveled(0),
          current_velocity(0),
          current_velocity_sqr(0),
          stepPin(_stepPin),
          dirPin(_dirPin)
    {
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
    }

    /**
     * @brief Starts a continuous rotation, accelerating to a target velocity.
     * @param target_velocity The desired final velocity in steps/sec. Can be negative for reverse.
     * @param acceleration The acceleration in steps/sec^2.
     */
    void StepperBase::startRotate(int32_t target_velocity, uint32_t acceleration)
    {
        this->target_velocity = target_velocity;
        this->target_velocity_original = target_velocity;
        // Calculate the signed square of the target velocity. Multiplying by signum preserves direction.
        this->target_velocity_sqr = (int64_t)signum(target_velocity) * target_velocity * target_velocity;

        // Determine if we need to accelerate (+1) or decelerate (-1).
        this->velocity_change_direction = (int32_t)signum(this->target_velocity_sqr - this->current_velocity_sqr);
        this->two_times_acceleration = 2 * acceleration;

        if (!isMoving)
        {
            step_timer = TimerFactory::makeTimer();
            step_timer->setPulseParams(8, stepPin); // 8 microsecond pulse width
            step_timer->attachCallbacks([this]
                                        { rotISR(); }, [this]
                                        { resetISR(); });

            // Set an initial "kick-start" velocity to get the motor moving immediately.
            // This avoids a division by zero if starting from a standstill.
            current_velocity_sqr = velocity_change_direction * 200 * 200; // TODO: Use a named constant

            move_mode = move_mode_t::ROTATE_CONTINUOUS;
            step_timer->start();
            isMoving = true;
        }
    }

    /**
     * @brief Plans and starts a move to an absolute target position.
     * @param target_position The absolute target position in steps.
     * @param v_e The end velocity (typically 0).
     * @param max_velocity The maximum cruise velocity for the move in steps/sec.
     * @param acceleration The acceleration for the move in steps/sec^2.
     */
    void StepperBase::startMoveTo(int32_t target_position, int32_t v_e, uint32_t max_velocity, uint32_t acceleration)
    {
        steps_traveled = 0;
        int32_t distance_to_travel_steps = std::abs(target_position - current_position_steps);
        this->total_steps_for_move = distance_to_travel_steps;

        // Set the motor's physical direction pin based on the target.
        direction_multiplier = signum(target_position - current_position_steps);
        digitalWriteFast(dirPin, direction_multiplier > 0 ? HIGH : LOW);
        delayMicroseconds(5); // Allow time for the direction pin to settle.

        two_times_acceleration = 2 * acceleration;
        current_velocity_sqr = 0; // Start move from a standstill.
        current_velocity = 0;

        target_velocity_original = max_velocity;
        target_velocity_sqr = (int64_t)max_velocity * max_velocity;

        /*
         * === Calculate Acceleration Distance ===
         * Derives the number of steps required to accelerate to max_velocity using the
         * kinematic formula: v_f^2 = v_i^2 + 2ad
         * Rearranging for distance (d): d = (v_f^2 - v_i^2) / 2a
         * Here, v_f^2 is target_velocity_sqr, v_i^2 is current_velocity_sqr, and 2a is two_times_acceleration.
         */
        int64_t acceleration_distance = (target_velocity_sqr - current_velocity_sqr) / two_times_acceleration + 1;

        // --- Check for Trapezoid vs. Triangle Profile ---
        // If the calculated acceleration distance is more than half the total travel distance,
        // the motor won't have time to reach full speed. The motion profile becomes a
        // triangle (accelerate to halfway point, then decelerate).
        if (acceleration_distance >= distance_to_travel_steps / 2)
        {
            acceleration_distance = distance_to_travel_steps / 2;
        }

        // Set the step counts that define the boundaries of the motion profile phases.
        acceleration_end_step = acceleration_distance - 1;
        deceleration_start_step = total_steps_for_move - acceleration_distance;

        if (!isMoving)
        {
            step_timer = TimerFactory::makeTimer();
            step_timer->attachCallbacks([this]
                                        { stepISR(); }, [this]
                                        { resetISR(); });
            step_timer->setPulseParams(8, stepPin); // 8 microsecond pulse width

            isMoving = true;
            // Set an initial "kick-start" velocity to get the motor moving immediately.
            current_velocity_sqr = 200 * 200; // TODO: Use a named constant

            move_mode = move_mode_t::TARGET_POSITION;
            step_timer->start();
        }
    }

    /**
     * @brief Initiates a controlled stop from the current motion.
     * @param end_velocity The velocity to stop at (usually 0).
     * @param acceleration The deceleration to use.
     */
    void StepperBase::startStopping(int32_t end_velocity, uint32_t acceleration)
    {
        if (move_mode == move_mode_t::ROTATE_CONTINUOUS)
        {
            // If rotating, we can simply start a new "rotation" to the target end_velocity.
            move_mode = move_mode_t::STOPPING;
            startRotate(end_velocity, acceleration);
            move_mode = move_mode_t::STOPPING;
        }
        else
        {
            // If in a target move, just set the mode. The stepISR will see this
            // flag and immediately begin the deceleration phase.
            move_mode = move_mode_t::STOPPING;
        }
    }

    /**
     * @brief Immediately halts all motor activity without deceleration.
     */
    void StepperBase::emergencyStop()
    {
        if (step_timer != nullptr)
        {
            step_timer->stop();
            TimerFactory::returnTimer(step_timer);
            step_timer = nullptr;
        }
        isMoving = false;
        current_velocity_sqr = 0;
    }

    /**
     * @brief Overrides the speed of a continuous rotation move in real-time.
     * @param factor A multiplier for the original target speed (e.g., 1.0 for 100%, 0.5 for 50%).
     */
    void StepperBase::overrideSpeed(float factor)
    {
        if (move_mode == move_mode_t::ROTATE_CONTINUOUS)
        {
            // Disable interrupts to ensure the following shared variables are updated atomically.
            // This prevents the ISR from reading a partially updated state (a race condition).
            noInterrupts();
            target_velocity = target_velocity_original * factor;
            target_velocity_sqr = (int64_t)signum(target_velocity) * target_velocity * target_velocity;
            velocity_change_direction = (int32_t)signum(target_velocity_sqr - current_velocity_sqr);
            interrupts();
        }
    }
}

#pragma pop_macro("abs")