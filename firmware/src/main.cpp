#include <Arduino.h>
#include <Bounce2.h>
#include "AccelStepper.h"

// --- Pin Definitions ---
// Update these pin numbers to match your hardware setup.
const int NUM_AXES = 6;
const int ESTOP_PIN = 3;   // Emergency Stop pin
bool ESTOP_ACTIVE = false; // Set to true if the E-Stop is active low, false if active high

const int J1_STEP_PIN = 25;
const int J1_DIR_PIN = 24;
const int J2_STEP_PIN = 21;
const int J2_DIR_PIN = 20;
const int J3_STEP_PIN = 18;
const int J3_DIR_PIN = 17;
const int J4_STEP_PIN = 15;
const int J4_DIR_PIN = 14;
const int J5_STEP_PIN = 47;
const int J5_DIR_PIN = 46;
const int J6_STEP_PIN = 44;
const int J6_DIR_PIN = 43;

// Limit switch pin, used for homing
const int J1_LIMIT_PIN = 23;
const int J2_LIMIT_PIN = 19;
const int J3_LIMIT_PIN = 16;
const int J4_LIMIT_PIN = 2;
const int J5_LIMIT_PIN = 45;
const int J6_LIMIT_PIN = 42;
// Joint steps per degree configuration
const float J1_STEPS_PER_DEGREE = 88.88;  // (800 * 10 * 4) / 360;
const float J2_STEPS_PER_DEGREE = 111.11; // (800 * 50) / 360;
const float J3_STEPS_PER_DEGREE = 111.11; // (800 * 50) / 360;
const float J4_STEPS_PER_DEGREE = 44.44;  // (800 * 10 * 2) / 360;
const float J5_STEPS_PER_DEGREE = 42.33;  // 15240 / 360;
const float J6_STEPS_PER_DEGREE = 4.44;   // 1600 / 360;

// Joint limits in degrees
const int J1_NEGATIVE_LIMIT = -170;
const int J1_POSITIVE_LIMIT = 115;

const int J2_NEGATIVE_LIMIT = -20;
const int J2_POSITIVE_LIMIT = 108;

const int J3_NEGATIVE_LIMIT = -102;
const int J3_POSITIVE_LIMIT = 38;

const int J4_NEGATIVE_LIMIT = -209;
const int J4_POSITIVE_LIMIT = 145;

const int J5_NEGATIVE_LIMIT = -100.9;
const int J5_POSITIVE_LIMIT = 106;

const int J6_NEGATIVE_LIMIT = -173;
const int J6_POSITIVE_LIMIT = 157;

// --- Motor Direction Configuration ---

// `true` means flip the direction of the motor
// `false` means keep the direction as is
const bool INVERT_DIRECTION[NUM_AXES] = {true, true, true, true, true, true};

// `false` means the motor moves towards negative direction during calibration
// `true` means the motor moves towards positive direction during calibration
const bool CALIBRATION_DIRECTION[NUM_AXES] = {true, false, true, false, false, false};

// --- Global Variables ---
const int stepPins[NUM_AXES] = {J1_STEP_PIN, J2_STEP_PIN, J3_STEP_PIN, J4_STEP_PIN, J5_STEP_PIN, J6_STEP_PIN};
const int dirPins[NUM_AXES] = {J1_DIR_PIN, J2_DIR_PIN, J3_DIR_PIN, J4_DIR_PIN, J5_DIR_PIN, J6_DIR_PIN};
const float STEPS_PER_DEGREE[NUM_AXES] = {J1_STEPS_PER_DEGREE, J2_STEPS_PER_DEGREE, J3_STEPS_PER_DEGREE, J4_STEPS_PER_DEGREE, J5_STEPS_PER_DEGREE, J6_STEPS_PER_DEGREE};
const int JOINT_NEGATIVE_LIMITS[NUM_AXES] = {J1_NEGATIVE_LIMIT, J2_NEGATIVE_LIMIT, J3_NEGATIVE_LIMIT, J4_NEGATIVE_LIMIT, J5_NEGATIVE_LIMIT, J6_NEGATIVE_LIMIT};
const int JOINT_POSITIVE_LIMITS[NUM_AXES] = {J1_POSITIVE_LIMIT, J2_POSITIVE_LIMIT, J3_POSITIVE_LIMIT, J4_POSITIVE_LIMIT, J5_POSITIVE_LIMIT, J6_POSITIVE_LIMIT};
const int LIMIT_SWITCH_PINS[NUM_AXES] = {J1_LIMIT_PIN, J2_LIMIT_PIN, J3_LIMIT_PIN, J4_LIMIT_PIN, J5_LIMIT_PIN, J6_LIMIT_PIN};
Bounce2 ::Button limitSwitches[NUM_AXES] = {
    Bounce2::Button(),
    Bounce2::Button(),
    Bounce2::Button(),
    Bounce2::Button(),
    Bounce2::Button(),
    Bounce2::Button()};
bool isCalibrationDone[NUM_AXES] = {false}; // To indicate if calibration is complete for a joint

AccelStepper steppers[6] = {
    AccelStepper(AccelStepper::DRIVER, stepPins[0], dirPins[0]),
    AccelStepper(AccelStepper::DRIVER, stepPins[1], dirPins[1]),
    AccelStepper(AccelStepper::DRIVER, stepPins[2], dirPins[2]),
    AccelStepper(AccelStepper::DRIVER, stepPins[3], dirPins[3]),
    AccelStepper(AccelStepper::DRIVER, stepPins[4], dirPins[4]),
    AccelStepper(AccelStepper::DRIVER, stepPins[5], dirPins[5])};

// Calibration speeds (steps per second) for each joint.
const float CALIBRATION_SPEEDS[NUM_AXES] = {(5 * STEPS_PER_DEGREE[0]),
                                            (4 * STEPS_PER_DEGREE[1]),
                                            (4 * STEPS_PER_DEGREE[2]),
                                            (20 * STEPS_PER_DEGREE[3]),
                                            (10 * STEPS_PER_DEGREE[4]),
                                            (10 * STEPS_PER_DEGREE[5])};
const float JOINT_MAX_SPEEDS[NUM_AXES] = {(15 * STEPS_PER_DEGREE[0]),
                                          (15 * STEPS_PER_DEGREE[1]),
                                          (30 * STEPS_PER_DEGREE[2]),
                                          (60 * STEPS_PER_DEGREE[3]),
                                          (60 * STEPS_PER_DEGREE[4]),
                                          (100 * STEPS_PER_DEGREE[5])};
const float CALIBRATION_OFFSETS[NUM_AXES] = {0, 0, 0, 0, 0, 0}; // Calibration offsets for each joint
int currentPosition[NUM_AXES] = {0, 0, 0, 0, 0, 0};

// Minimum and maximum allowable speed delays (in microseconds).
// These act as safety limits.
const int MIN_SPEED_DELAY = 50;    // Corresponds to the absolute fastest speed
const int MAX_SPEED_DELAY = 10000; // Corresponds to a very slow start/end speed

// =================================================================
//   UTILITY FUNCTIONS
// =================================================================
Bounce2::Button button = Bounce2::Button();

void setupLimitSwitches()
{
  for (int i = 0; i < NUM_AXES; i++)
  {

    limitSwitches[i].attach(LIMIT_SWITCH_PINS[i], INPUT_PULLUP);
    limitSwitches[i].setPressedState(HIGH); // Assuming high means pressed
    limitSwitches[i].interval(5);           // Set debounce interval to 5ms
  }
}

void updateLimitSwitches()
{
  for (int i = 0; i < NUM_AXES; i++)
  {
    limitSwitches[i].update();
  }
}

/**
 * @brief Pulses the STEP pin for a given motor axis.
 * @param axisIndex The index of the motor to step (0-5).
 */
void stepMotor(int axisIndex)
{
  digitalWrite(stepPins[axisIndex], HIGH);
  delayMicroseconds(2); // A short pulse width is sufficient
  digitalWrite(stepPins[axisIndex], LOW);
}

/**
 * @brief Prints the current position of all motors to the Serial Monitor.
 */
void printCurrentPosition()
{
  Serial.print("CURRENT POSITIONS: [");
  for (int i = 0; i < NUM_AXES; i++)
  {
    Serial.print(currentPosition[i]);
    if (i < NUM_AXES - 1)
    {
      Serial.print(", ");
    }
  }
  Serial.println("]");
}

void stopMotor(int jointIndex)
{
  steppers[jointIndex].setSpeed(0);
  steppers[jointIndex].moveTo(steppers[jointIndex].currentPosition());             // Stop the motor immediately
  steppers[jointIndex].setCurrentPosition(steppers[jointIndex].currentPosition()); // Reset current position to avoid overshoot
}

// =================================================================
//   CORE MOVEMENT FUNCTION with ACCELERATION/DECELERATION
// =================================================================

/**
 * @brief Moves motors in a coordinated line with acceleration and deceleration.
 * @param target The array of target positions in absolute steps.
 * @param moveDurationSec The total desired duration for the move in seconds.
 * @param accelDecelPercent The percentage of the move used for accel/decel (0.0 to 1.0).
 * For example, 0.2 means 10% accel and 10% decel.
 */
void moveMotorsBresenham(int target[NUM_AXES], float moveDurationSec, float accelDecelPercent)
{
  // filter out the axes that are not calibrated
  for (int i = 0; i < NUM_AXES; i++)
  {
    if (!isCalibrationDone[i])
    {
      currentPosition[i] = 0;
      target[i] = 0; // If not calibrated, set target to 0. No operation will be performed on this axis.
    }
  }
  // --- 1. Calculate Deltas and Directions ---
  int delta[NUM_AXES];
  int direction[NUM_AXES];
  for (int i = 0; i < NUM_AXES; i++)
  {
    delta[i] = target[i] - currentPosition[i];
    if (delta[i] > 0)
      direction[i] = 1;
    else if (delta[i] < 0)
      direction[i] = -1;
    else
      direction[i] = 0;
  }

  // --- 2. Set Physical Motor Directions ---
  for (int i = 0; i < NUM_AXES; i++)
  {
    if (delta[i] != 0)
    {
      bool isPositive = (direction[i] > 0);
      uint8_t directionToNeg = HIGH;
      uint8_t directionToPos = LOW;
      if (INVERT_DIRECTION[i])
      {
        directionToNeg = directionToNeg == HIGH ? LOW : HIGH;
        directionToPos = directionToPos == HIGH ? LOW : HIGH;
      }
      digitalWrite(dirPins[i], isPositive ? directionToPos : directionToNeg);
    }
  }

  // --- 3. Find Master Axis and Total Steps ---
  int masterSteps = 0;
  int masterAxis = 0;
  for (int i = 0; i < NUM_AXES; i++)
  {
    if (abs(delta[i]) > masterSteps)
    {
      masterSteps = abs(delta[i]);
      masterAxis = i;
    }
  }

  if (masterSteps == 0)
  {
    // Serial.println("Target is the same as current. No move needed."); // Removed for performance
    return;
  }

  // --- 4. Initialize Bresenham's Decision Parameters ---
  int decisionParams[NUM_AXES];
  for (int i = 0; i < NUM_AXES; i++)
  {
    decisionParams[i] = 2 * abs(delta[i]) - masterSteps;
  }

  // --- 5. Acceleration Profile Calculation (Trapezoidal) ---

  // Sanitize input
  accelDecelPercent = constrain(accelDecelPercent, 0.0, 1.0);

  // Calculate the number of steps for acceleration and deceleration
  int accelSteps = masterSteps * (accelDecelPercent / 2.0);
  int decelStartStep = masterSteps - accelSteps;

  // Calculate the average delay per step to meet the duration goal.
  // This is the target delay for the constant speed (cruise) phase.
  float avgDelayMicroSec = (moveDurationSec * 1000000.0) / masterSteps;

  // The delay at the start and end of the move. Must be slower than avg.
  // For a trapezoidal profile, this is essentially the initial delay before ramping up.
  float initialDelay = avgDelayMicroSec * (1.0 + accelDecelPercent);

  // Clamp the calculated delays to sensible, safe hardware limits.
  float cruiseDelay = constrain(avgDelayMicroSec, MIN_SPEED_DELAY, MAX_SPEED_DELAY);
  float startDelay = constrain(initialDelay, cruiseDelay, MAX_SPEED_DELAY);

  Serial.println("start, cruise " + String(startDelay) + ", " + String(cruiseDelay));
  // --- 6. The Main Bresenham Loop with Ramping ---
  unsigned long loopStartTime = micros(); // Start timing

  float currentDelay = startDelay;

  for (int step = 0; step < masterSteps; step++)
  {
    // --- Check E-Stop ---
    if (ESTOP_ACTIVE)
      return; // If E-Stop is active, exit the function immediately

    // --- RAMPING LOGIC (Trapezoidal) ---
    if (step < accelSteps && accelSteps > 0)
    {
      // ACCELERATION PHASE (Linear ramp)
      // Calculate progress through the acceleration phase (0.0 to 1.0)
      float accelProgress = (float)step / accelSteps;
      // Linearly interpolate the delay between the start delay and the cruise delay
      currentDelay = startDelay - (startDelay - cruiseDelay) * accelProgress;
    }
    else if (step >= decelStartStep && accelSteps > 0)
    {
      // DECELERATION PHASE (Linear ramp)
      // Calculate progress through the deceleration phase (0.0 to 1.0)
      float decelProgress = (float)(step - decelStartStep) / accelSteps;
      // Linearly interpolate the delay between the cruise delay and the final (start) delay
      currentDelay = cruiseDelay + (startDelay - cruiseDelay) * decelProgress;
    }
    else
    {
      // CONSTANT SPEED (CRUISE) PHASE
      currentDelay = cruiseDelay;
    }

    // --- MOTOR STEPPING LOGIC (Bresenham) ---
    // a. Always step the master motor
    stepMotor(masterAxis);
    currentPosition[masterAxis] += direction[masterAxis];

    // b. Check and step slave motors
    for (int i = 0; i < NUM_AXES; i++)
    {
      if (ESTOP_ACTIVE)
        return; // If E-Stop is active, exit the function immediately
      if (i == masterAxis)
        continue;

      if (decisionParams[i] >= 0)
      {
        stepMotor(i);
        currentPosition[i] += direction[i];
        decisionParams[i] -= 2 * masterSteps;
      }
      decisionParams[i] += 2 * abs(delta[i]);
    }

    // c. Apply the calculated delay for speed control
    // Ensure we never delay for less than the minimum safety time
    delayMicroseconds(max((long)currentDelay, MIN_SPEED_DELAY));
  }

  unsigned long loopEndTime = micros();                                    // End timing
  float actualDuration = (float)(loopEndTime - loopStartTime) / 1000000.0; // Convert to seconds

  Serial.print("Actual loop execution time: ");
  Serial.print(actualDuration, 3); // Print with 3 decimal places
  Serial.println(" seconds");
  Serial.print("Difference from expected: ");
  Serial.println(actualDuration - moveDurationSec, 3); // Print difference in seconds
  Serial.println();
  // Serial.println("Move Complete."); // Removed for performance
}

// Helper function to split a String by a delimiter
int splitString(String input, char delimiter, String outputArray[], int maxItems)
{
  int itemCount = 0;
  int startIndex = 0;
  int delimiterIndex = input.indexOf(delimiter);

  while (delimiterIndex != -1 && itemCount < maxItems)
  {
    outputArray[itemCount++] = input.substring(startIndex, delimiterIndex);
    startIndex = delimiterIndex + 1;
    delimiterIndex = input.indexOf(delimiter, startIndex);
  }

  // Add the last item (or the entire string if no delimiter was found)
  if (startIndex < input.length() && itemCount < maxItems)
  {
    outputArray[itemCount++] = input.substring(startIndex);
  }

  return itemCount; // Return the number of items split
}

// Check if the limit switch for the specified joint is pressed
bool isLimitSwitchActive(int jointIndex)
{
  return limitSwitches[jointIndex].isPressed(); // LOW means pressed
}

enum CalibrationPhase
{
  CALIB_IDLE,
  CALIB_SEEK_LIMIT_FAST,
  CALIB_BACKOFF_FROM_LIMIT,
  CALIB_SEEK_LIMIT_SLOW,
  CALIB_MOVE_TO_CENTER,
  CALIB_DONE,
  CALIB_FAILED
};

CalibrationPhase calibrationPhase[NUM_AXES] = {CALIB_IDLE};
bool calibrationInProgress[NUM_AXES] = {false}; // To indicate if a calibration is active for a joint
// Function to start calibration for a specific joint
void startCalibrateJoint(int jointIndex)
{
  if (jointIndex < 0 || jointIndex >= NUM_AXES)
    return;
  Serial.print("Starting calibration for Joint ");
  Serial.println(jointIndex + 1);
  calibrationPhase[jointIndex] = CALIB_IDLE; // Reset phase
  calibrationInProgress[jointIndex] = true;
  stopMotor(jointIndex);
  currentPosition[jointIndex] = 0;       // Clear software position for safety
  isCalibrationDone[jointIndex] = false; // Reset calibration status
}

// This function will be called repeatedly in loop() for each joint
void runJointCalibration(int jointIndex)
{
  if (!calibrationInProgress[jointIndex])
    return; // Only run if calibration is active for this joint

  // Configure direction based on CALIBRATION_DIRECTION
  int positiveDirection = CALIBRATION_DIRECTION[jointIndex] ? 1 : -1; // +1 for positive move, -1 for negative move
  if (INVERT_DIRECTION[jointIndex])
  {
    positiveDirection = -positiveDirection; // Invert direction if configured
  }

  float backOffDegrees = 15.0; // Back off 15 degrees from the limit

  switch (calibrationPhase[jointIndex])
  {
  case CALIB_IDLE:
    // Check if already on limit switch
    if (isLimitSwitchActive(jointIndex))
    {
      Serial.print("Joint ");
      Serial.print(jointIndex + 1);
      Serial.println(": Already on limit, moving away.");
      long backOffSteps = (long)(backOffDegrees * STEPS_PER_DEGREE[jointIndex]);
      steppers[jointIndex].setMaxSpeed(CALIBRATION_SPEEDS[jointIndex]);
      steppers[jointIndex].setAcceleration(CALIBRATION_SPEEDS[jointIndex] / 2); // Simple acceleration for backoff
      steppers[jointIndex].move(positiveDirection * backOffSteps);              // Move in the positive (away) direction
      calibrationPhase[jointIndex] = CALIB_BACKOFF_FROM_LIMIT;
    }
    else
    {
      Serial.print("Joint ");
      Serial.print(jointIndex + 1);
      Serial.println(": Seeking limit fast.");
      long maxTravelSteps = (long)((abs(JOINT_NEGATIVE_LIMITS[jointIndex]) + abs(JOINT_POSITIVE_LIMITS[jointIndex])) * STEPS_PER_DEGREE[jointIndex]); // Max travel
      steppers[jointIndex].setMaxSpeed(CALIBRATION_SPEEDS[jointIndex]);
      steppers[jointIndex].setAcceleration(CALIBRATION_SPEEDS[jointIndex] / 2);
      steppers[jointIndex].move(-positiveDirection * maxTravelSteps); // Move towards limit (negative direction relative to calibration direction)
      calibrationPhase[jointIndex] = CALIB_SEEK_LIMIT_FAST;
    }
    break;

  case CALIB_SEEK_LIMIT_FAST:
    if (isLimitSwitchActive(jointIndex))
    {
      stopMotor(jointIndex); // Stop the stepper motor immediately
      Serial.print("Joint ");
      Serial.print(jointIndex + 1);
      Serial.println(": Limit switch hit (fast). Backing off.");
      long backOffSteps = (long)(5 * STEPS_PER_DEGREE[jointIndex]);
      steppers[jointIndex].setAcceleration(CALIBRATION_SPEEDS[jointIndex] / 2); // Set acceleration for backoff
      steppers[jointIndex].setMaxSpeed(CALIBRATION_SPEEDS[jointIndex]);         // Keep same speed for backoff
      steppers[jointIndex].move(positiveDirection * backOffSteps);              // Move away from limit
      calibrationPhase[jointIndex] = CALIB_BACKOFF_FROM_LIMIT;
    }
    else if (steppers[jointIndex].distanceToGo() == 0)
    {
      // Moved maximum distance and limit not hit
      Serial.print("Joint ");
      Serial.print(jointIndex + 1);
      Serial.println(": Max travel reached, limit not found. Failed calibration.");
      calibrationPhase[jointIndex] = CALIB_FAILED;
      calibrationInProgress[jointIndex] = false;
    }
    break;

  case CALIB_BACKOFF_FROM_LIMIT:
    if (steppers[jointIndex].distanceToGo() == 0)
    {
      if (isLimitSwitchActive(jointIndex))
      {
        Serial.print("Joint ");
        Serial.print(jointIndex + 1);
        Serial.println(": Still on limit after backoff. Failed calibration.");
        calibrationPhase[jointIndex] = CALIB_FAILED;
        calibrationInProgress[jointIndex] = false;
      }
      else
      {
        Serial.print("Joint ");
        Serial.print(jointIndex + 1);
        Serial.println(": Backed off, now seeking limit slowly.");
        long fineApproachSteps = (long)((backOffDegrees + 5) * STEPS_PER_DEGREE[jointIndex]); // Small distance for fine approach
        steppers[jointIndex].setMaxSpeed(CALIBRATION_SPEEDS[jointIndex] / 5.0);               // Slower speed
        steppers[jointIndex].setAcceleration(CALIBRATION_SPEEDS[jointIndex] / 10.0);
        steppers[jointIndex].move(-positiveDirection * fineApproachSteps); // Move towards limit slowly
        calibrationPhase[jointIndex] = CALIB_SEEK_LIMIT_SLOW;
      }
    }
    break;

  case CALIB_SEEK_LIMIT_SLOW:
    if (isLimitSwitchActive(jointIndex))
    {
      stopMotor(jointIndex);
      Serial.print("Joint ");
      Serial.print(jointIndex + 1);
      Serial.println(": Limit switch hit (slow). Moving to center.");
      steppers[jointIndex].setCurrentPosition(0); // Set current position to 0 at the limit switch
      currentPosition[jointIndex] = 0;            // Sync your software position array

      long stepsToCenter = 0;
      if (CALIBRATION_DIRECTION[jointIndex])
      {
        stepsToCenter = (long)((abs(JOINT_POSITIVE_LIMITS[jointIndex]) + CALIBRATION_OFFSETS[jointIndex]) * STEPS_PER_DEGREE[jointIndex]);
      }
      else
      {
        stepsToCenter = (long)((abs(JOINT_NEGATIVE_LIMITS[jointIndex]) + CALIBRATION_OFFSETS[jointIndex]) * STEPS_PER_DEGREE[jointIndex]);
      }

      steppers[jointIndex].setMaxSpeed(JOINT_MAX_SPEEDS[jointIndex]);         // Use max operating speed for center move
      steppers[jointIndex].setAcceleration(JOINT_MAX_SPEEDS[jointIndex] / 2); // Set appropriate acceleration
      steppers[jointIndex].move(positiveDirection * stepsToCenter);           // Move to the calculated center
      calibrationPhase[jointIndex] = CALIB_MOVE_TO_CENTER;
    }
    else if (steppers[jointIndex].distanceToGo() == 0)
    {
      Serial.print("Joint ");
      Serial.print(jointIndex + 1);
      Serial.println(": Fine approach finished, limit not found. Failed calibration.");
      calibrationPhase[jointIndex] = CALIB_FAILED;
      calibrationInProgress[jointIndex] = false;
    }
    break;

  case CALIB_MOVE_TO_CENTER:
    if (steppers[jointIndex].distanceToGo() == 0)
    {
      Serial.print("Joint ");
      Serial.print(jointIndex + 1);
      Serial.println(": Moved to center. Calibration successful.");
      currentPosition[jointIndex] = 0; // Update software position
      calibrationPhase[jointIndex] = CALIB_DONE;
    }
    break;

  case CALIB_DONE:
    // Calibration for this joint is complete.
    stopMotor(jointIndex); // Stop the stepper motor
    Serial.print("Calibration complete for Joint ");
    Serial.println(jointIndex + 1);
    calibrationInProgress[jointIndex] = false; // Reset the calibration state
    isCalibrationDone[jointIndex] = true;      // Mark this joint as calibrated
    break;

  case CALIB_FAILED:
    // Calibration for this joint failed.
    // You might want to signal an error or retry.
    Serial.println("Calibration failed for Joint " + String(jointIndex + 1));
    calibrationInProgress[jointIndex] = false; // Reset the calibration state
    stopMotor(jointIndex);                     // Stop the stepper motor
    break;
  }
  steppers[jointIndex].run(); // Keep the stepper running in its current state
}

void printCalibrationStatus()
{
  Serial.print("CALIBRATION STATUS: [");
  for (int i = 0; i < NUM_AXES; i++)
  {
    int status = 0; // 0 - Not Calibrated
    if (isCalibrationDone[i])
      status = 2; // 2 - Calibrated
    else if (calibrationInProgress[i])
      status = 1; // 1 - In Progress

    Serial.print(status);
    if (i < NUM_AXES - 1)
      Serial.print(",");
  }
  Serial.println("]");
}

int degreeToSteps(int jointIndex, float degrees)
{
  // Convert degrees to steps for the specified joint
  return (int)(degrees * STEPS_PER_DEGREE[jointIndex]);
}

bool isInRange(int jointIndex, float degrees)
{
  // Check if the given degrees are within the limits for the specified joint
  return (degrees >= JOINT_NEGATIVE_LIMITS[jointIndex] && degrees <= JOINT_POSITIVE_LIMITS[jointIndex]);
}

void onEstopChanged()
{
  if (digitalRead(ESTOP_PIN) == LOW)
  {
    ESTOP_ACTIVE = true; // E-Stop is pressed
    Serial.println("E-Stop activated");
  }
  else
  {
    ESTOP_ACTIVE = false; // E-Stop is released
    Serial.println("E-Stop released");
  }
}

void handle_MOVE_JOINTS(String input)
{
  // MOVE_JOINTS j1_degree,j2_degree,j3_degree,j4_degree,j5_degree,j6_degree,duration_sec,accel_decel_percent
  // use split
  String parts[8];
  splitString(input, ',', parts, 8);
  if (parts[0].length() > 0 && parts[1].length() > 0 && parts[2].length() > 0 && parts[3].length() > 0 && parts[4].length() > 0 && parts[5].length() > 0 && parts[6].length() > 0 && parts[7].length() > 0)
  {
    int targetDegreesInSteps[NUM_AXES];
    for (int i = 0; i < NUM_AXES; i++)
    {
      if (isCalibrationDone[i] == false)
      {
        Serial.println("Joint " + String(i + 1) + " is not calibrated. Please calibrate before moving.");
        return; // Exit if any joint is not calibrated
      }

      // Check if the joint is within limits
      float degrees = parts[i].toFloat();
      if (!isInRange(i, degrees))
      {
        Serial.println("Joint " + String(i + 1) + " out of range: " + String(degrees) + " degrees. Valid range: [" + String(JOINT_NEGATIVE_LIMITS[i]) + ", " + String(JOINT_POSITIVE_LIMITS[i]) + "]");
        return; // Exit if any joint is out of range
      }
      targetDegreesInSteps[i] = degreeToSteps(i, parts[i].toFloat());
    }
    float moveDurationSec = parts[6].toFloat();
    float accelDecelPercent = parts[7].toFloat();
    // Call the move function
    moveMotorsBresenham(targetDegreesInSteps, moveDurationSec, accelDecelPercent);
    Serial.println("MOVE_JOINTS COMPLETE");
  }
  else
  {
    Serial.println("Invalid MOVE_JOINTS command format. Use: MOVE_JOINTS <j1>,<j2>,<j3>,<j4>,<j5>,<j6>,<duration_sec>,<accel_decel_percent>");
  }
}

void handle_MOVE_JOINT(String input)
{
  // MOVE_JOINT jointNum,targetDegree,duration_sec,accel_decel_percent
  String parts[4];
  splitString(input, ',', parts, 4);
  int jointIndex = parts[0].toInt() - 1;
  float targetDegree = parts[1].toFloat();
  float duration = parts[2].toFloat();
  float accelDecelPercent = parts[3].toFloat();

  int degreeInSteps = degreeToSteps(jointIndex, targetDegree);

  int targetSteps[NUM_AXES] = {0};
  for (int idx = 0; idx < NUM_AXES; idx++)
  {
    if (idx == jointIndex)
    {
      targetSteps[idx] = degreeInSteps;
    }
    else
    {
      targetSteps[idx] = currentPosition[idx];
    }
  }

  moveMotorsBresenham(targetSteps, duration, accelDecelPercent);
  Serial.println("MOVE_JOINT " + parts[0] + " COMPLETE");
}

void handle_MOVE_JOINT_BY(String input)
{
  // MOVE_JOINT_BY jointNum,degreeDelta,duration_sec,accel_decel_percent
  String parts[4];
  splitString(input, ',', parts, 4);
  int jointIndex = parts[0].toInt() - 1;
  float degreeDelta = parts[1].toFloat();
  float duration = parts[2].toFloat();
  float accelDecelPercent = parts[3].toFloat();

  int degreeDeltaInSteps = degreeToSteps(jointIndex, degreeDelta);

  int targetSteps[NUM_AXES] = {0};
  for (int idx = 0; idx < NUM_AXES; idx++)
  {
    if (idx == jointIndex)
    {
      targetSteps[idx] = currentPosition[idx] + degreeDeltaInSteps;
    }
    else
    {
      targetSteps[idx] = currentPosition[idx];
    }
  }

  moveMotorsBresenham(targetSteps, duration, accelDecelPercent);
  Serial.println("MOVE_JOINT_BY " + parts[0] + " COMPLETE");
}

void handle_S()
{
  for (int i = 0; i < NUM_AXES; i++)
  {
    stopMotor(i); // Stop all motors
    if (calibrationInProgress[i])
    {
      calibrationPhase[i] = CALIB_FAILED;
    }
  }
  Serial.println("All motors stopped.");
}

void handle_STOP_JOINT(String input)
{
  // Parse the command
  // STOP_JOINT 1
  int jointNum = input.toInt();
  stopMotor(jointNum - 1);
  Serial.println("STOP_J " + String(jointNum));
}

void handle_CALIBRATE_JOINTS(String input)
{
  // Parse the command
  // CALIBRATE_JOINTS 1,2,3
  // CALIBRATE_JOINTS 4,5,6
  String axes[NUM_AXES];
  splitString(input, ',', axes, NUM_AXES);
  for (int i = 0; i < NUM_AXES; i++)
  {
    if (axes[i].length() > 0)
    {
      int jointIndex = axes[i].toInt() - 1; // Convert to zero-based index
      if (jointIndex >= 0 && jointIndex < NUM_AXES)
      {
        // Call the calibration function
        startCalibrateJoint(jointIndex);
        Serial.println("Calibration started for Joint " + String(jointIndex + 1));
      }
      else
      {
        Serial.println("Invalid joint index: " + String(jointIndex + 1) + ". Use a number between 1 and " + String(NUM_AXES) + ".");
      }
    }
  }
}

String command = "";

// Command Hex Codes
#define CMD_ECHO 0x00
#define CMD_S 0x01
#define CMD_STOP_JOINT 0x02
#define CMD_MOVE_JOINTS 0x03
#define CMD_CALIBRATE_JOINTS 0x04
#define CMD_PRINT_POS 0x05
#define CMD_PRINT_CALIBRATION_STATUS 0x06
#define CMD_ADD 0x07
#define CMD_MOVE_JOINT 0x08
#define CMD_MOVE_JOINT_BY 0x09

void processSerialCommands()
{
  if (Serial.available())
  {
    // Read full line until newline
    String line = Serial.readStringUntil('\n');
    line.trim(); // Remove whitespace

    if (line.length() < 2)
      return; // At least two hex digits

    // Extract hex command (first 2 characters)
    String cmdHex = line.substring(0, 2);
    int cmd = strtol(cmdHex.c_str(), nullptr, 16);

    // Extract arguments (if any)
    String args = "";
    if (line.length() > 2 && line.charAt(2) == ' ')
    {
      args = line.substring(3); // Skip space after command
    }

    // Command handling
    switch (cmd)
    {
    case CMD_ECHO:
      Serial.println(args);
      break;

    case CMD_S:
      handle_S();
      break;

    case CMD_STOP_JOINT:
      handle_STOP_JOINT(args);
      break;

    case CMD_MOVE_JOINTS:
      handle_MOVE_JOINTS(args);
      break;

    case CMD_MOVE_JOINT:
      handle_MOVE_JOINT(args);
      break;

    case CMD_MOVE_JOINT_BY:
      handle_MOVE_JOINT_BY(args);
      break;

    case CMD_CALIBRATE_JOINTS:
      handle_CALIBRATE_JOINTS(args);
      break;

    case CMD_PRINT_POS:
      printCurrentPosition();
      break;

    case CMD_PRINT_CALIBRATION_STATUS:
      printCalibrationStatus();
      break;

    case CMD_ADD:
    {
      int commaIndex = args.indexOf(',');
      if (commaIndex != -1)
      {
        String num1 = args.substring(0, commaIndex);
        String num2 = args.substring(commaIndex + 1);
        int sum = num1.toInt() + num2.toInt();
        Serial.println("Sum: " + String(sum));
      }
      else
      {
        Serial.println("Invalid ADD format. Use: 07 <num1>,<num2>");
      }
      break;
    }

    default:
      Serial.println("Unknown command: " + cmdHex);
      break;
    }
  }
}

void handleEstop()
{
  if (ESTOP_ACTIVE)
  {
    // If E-Stop is active, stop all motors and ignore commands
    for (int i = 0; i < NUM_AXES; i++)
    {
      stopMotor(i);                       // Stop all motors immediately
      calibrationInProgress[i] = false;   // Reset calibration state for all joints
      calibrationPhase[i] = CALIB_FAILED; // Set all joints to failed calibration state
    }
  }
}

void runAllJointCalibrations()
{
  for (int i = 0; i < NUM_AXES; i++)
  {
    runJointCalibration(i);
  }
}

// =================================================================
//   SETUP
// =================================================================
void setup()
{
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate
  for (int i = 0; i < NUM_AXES; i++)
  {
    pinMode(stepPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
    digitalWrite(stepPins[i], LOW);
  }

  // limit switch pins
  setupLimitSwitches();
  pinMode(ESTOP_PIN, INPUT_PULLUP); // Set E-Stop pin as input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), onEstopChanged, CHANGE);
  // ESTOP_ACTIVE = digitalRead(ESTOP_PIN) == HIGH; // Initialize E-Stop state
  // Serial.println("E-Stop state initialized: " + String(ESTOP_ACTIVE ? "ACTIVE" : "INACTIVE"));
}

// =================================================================
//   MAIN LOOP
// =================================================================
void loop()
{
  updateLimitSwitches();
  handleEstop();
  processSerialCommands();
  runAllJointCalibrations();
}