#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp" 
#include "Lidar.hpp"
#include <Wire.h>
#include "IMUOdometry.hpp"  
#include <MPU6050.h>
#include "OledDisplay.hpp"

#define MOT1PWM 9
#define MOT1DIR 10
#define MOT2PWM 11
#define MOT2DIR 12

#define EN1A 2
#define EN1B 7
#define EN2A 3
#define EN2B 8

#define LIDAR 4

int task31();
int task32();
int task33();
bool turnToYaw(float targetYaw, int tolerance = 2, int speed = 30);

// GLOBAL VARIABLES 
float ax_offset = 0.0;
float ay_offset = 0.0;
float gz_offset = 0.0;
int basePWM = 100;
int leftPWM = basePWM;
int rightPWM = basePWM;
const float wheelDiameterCM = 3.55;
const float countsPerRev = 1400;
const float cmPerCount = (PI * wheelDiameterCM) / countsPerRev; 
static float targetDistanceCM = 50;
const int targetCounts = targetDistanceCM / cmPerCount; 
// Task 31
volatile bool objectDetected = false;
// Task 32
float targetYaw = 0.0;
bool hasTurnedInitially = false;
bool wasLifted = false;
//States
bool turnLeft = false;
bool turnRight = false;
bool forward = false;
bool reverse = false;
bool stop = true;
// Lidar pins
int LIDAR1 = A0;
int LIDAR2 = A1;
int LIDAR3 = A2;

// Objects 
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
mtrn3100::Encoder encoder1(EN1A, EN1B,1400);
mtrn3100::Encoder encoder2(EN2A, EN2B,1400);
mtrn3100::PIDController pid(1,0.1,0.05);


mtrn3100::Lidar leftLidar(LIDAR1,  0x30);
mtrn3100::Lidar frontLidar(LIDAR2,  0x31);
mtrn3100::Lidar rightLidar(LIDAR3,  0x32);

IMU imuOdom;
OledDisplay oled;
// LIDAR interrupt
void lidarISR() {
  objectDetected = true;
}

void encoder1ISR() {
  encoder1.readEncoder();
}

void encoder2ISR() {
  encoder2.readEncoder();
}

void encoderReset(){
  encoder1.count = 0;
  encoder2.count = 0;
}

void reset(){
  objectDetected = false;
  targetYaw = 0.0;
  hasTurnedInitially = false;
  wasLifted = false;
  turnLeft = false;
  turnRight = false;
  forward = false;
  reverse = false;
  stop = true;
}

// ---- Wall alignment helper ----
// Returns a small delta-heading (degrees) that nudges the robot away from walls.
// +delta means steer LEFT, -delta means steer RIGHT.
//
// Tuning knobs:
//  - desiredOffsetMM: desired gap to a single visible wall
//  - kCenter, kSingle: deg per mm (proportional gains)
//  - deltaLimitDeg: saturates max one-loop correction to avoid jerks
float computeWallAlignDeltaDeg() {
  // Read side distances (assume units = mm; -1 or 0 = invalid/no return)
  int dl = leftLidar.readDistanceAndTrigger();
  int dr = rightLidar.readDistanceAndTrigger();

  const int desiredOffsetMM = 25;   // keep ~25 mm from a single wall
  const int tolMM = 7;              // deadband
  const float kCenter = 0.08f;      // deg per mm when both walls seen
  const float kSingle = 0.10f;      // deg per mm when only one wall seen
  const float deltaLimitDeg = 6.0f; // saturate per-loop change

  float delta = 0.0f;

  bool leftValid  = (dl > 0);
  bool rightValid = (dr > 0);

  if (leftValid && rightValid) {
    // Centering: positive (dl - dr) means left is farther → steer LEFT
    float err = (float)dl - (float)dr;
    if (fabs(err) > tolMM) {
      delta = kCenter * err;  // +delta => steer LEFT
    }
  } else if (leftValid && !rightValid) {
    // Single-wall (left): err > 0 -> too far from left → steer RIGHT (negative)
    float err = (float)dl - (float)desiredOffsetMM;
    if (fabs(err) > tolMM) {
      delta = -kSingle * err; // +err -> -delta (steer RIGHT), -err -> +delta (steer LEFT)
    }
  } else if (rightValid && !leftValid) {
    // Single-wall (right): err > 0 -> too far from right → steer LEFT (positive)
    float err = (float)dr - (float)desiredOffsetMM;
    if (fabs(err) > tolMM) {
      delta = +kSingle * err; // +err -> +delta (steer LEFT), -err -> -delta (steer RIGHT)
    }
  } else {
    // No walls visible → no LIDAR-based correction
    delta = 0.0f;
  }

  // Saturate to avoid sudden jerks
  if (delta >  deltaLimitDeg) delta =  deltaLimitDeg;
  if (delta < -deltaLimitDeg) delta = -deltaLimitDeg;

  return delta;
}

// Utility to wrap any angle into [0,360)
static inline float wrap360(float a) {
  while (a >= 360.0f) a -= 360.0f;
  while (a <    0.0f) a += 360.0f;
  return a;
}


void printIMUStatus(float currentYaw, float targetYaw, float diff) {
  Serial.print("\033[2J");  // Clear entire screen
  Serial.print("\033[H");   // Move cursor to top-left corner

  Serial.print("Current Yaw: ");
  Serial.print(currentYaw, 2);
  Serial.print(" | Target Yaw: ");
  Serial.print(targetYaw, 2);
  Serial.print(" | Diff: ");
  Serial.println(diff, 2);
}

bool turnToYaw(float targetYaw, int tolerance, int maxSpeed) {
  imuOdom.update();
  float currentYaw, dummy1, dummy2;
  imuOdom.getOrientation(dummy1, dummy2, currentYaw);

  // Normalise yaw difference to [-180, 180]
  float error = targetYaw - currentYaw;
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;

  printIMUStatus(currentYaw, targetYaw, error);

  static mtrn3100::PIDController yawPID(1.2, 0.0, 0.1);
  static unsigned long prevTime = millis();

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0f;
  prevTime = now;

  // Use actual yaw values in PID
  float correction = yawPID.compute(0, -error, dt);

  if (abs(error) <= tolerance) {
    motor1.stop();
    motor2.stop();
    yawPID.reset();
    return true;
  }

  int pwm = constrain(abs(correction), 20, min(maxSpeed, abs(error) * 2));

  if (correction > 0) {
    motor1.forward(pwm);
    motor2.forward(pwm);
  } else {
    motor1.reverse(pwm);
    motor2.reverse(pwm);
  }

  return false;
}

bool moveStraightWithHeadingCorrection(float targetHeading, float distanceCM) {
  encoder1.reset();
  encoder2.reset();

  long targetCounts = distanceCM / cmPerCount;

  static float headingKp = 1.0;  
  static float headingKd = 0.5;
  static float headingKi = 0.0;
  static mtrn3100::PIDController headingPID(headingKp, headingKi, headingKd);

  unsigned long prevTime = millis();

  // We'll adjust this *locally* during the motion so you keep your global heading clean.
  float headingCmd = wrap360(targetHeading);

  while (true) {
    // --- Safety checks ---
    int distFront = frontLidar.readDistanceAndTrigger();
    if (distFront > 0 && distFront < 45) { // obstacle ahead (mm)
      motor1.stop();
      motor2.stop();
      return false; // interrupted by obstacle
    }

    // Side proximity hard-stop to prevent scraping (optional but recommended)
    int dl = leftLidar.readDistanceAndTrigger();
    int dr = rightLidar.readDistanceAndTrigger();
    if ((dl > 0 && dl < 18) || (dr > 0 && dr < 18)) { // too close to side wall (mm)
      motor1.stop();
      motor2.stop();
      return false; // interrupted by side proximity
    }

    // --- LIDAR-based wall alignment: update heading command continuously ---
    float deltaHeading = computeWallAlignDeltaDeg();
    headingCmd = wrap360(headingCmd + deltaHeading);

    // --- IMU heading control ---
    imuOdom.update();
    float currentYaw, dummy1, dummy2;
    imuOdom.getOrientation(dummy1, dummy2, currentYaw);

    float error = headingCmd - currentYaw;
    if (error > 180) error -= 360;
    else if (error < -180) error += 360;

    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.0f;
    prevTime = now;

    // PID correction (IMU)
    float correction = headingPID.compute(0, error, dt);

    // Mix into wheel speeds
    int pwmLeft  = basePWM - correction;
    int pwmRight = basePWM + correction;
    pwmLeft  = constrain(pwmLeft,  0, 255);
    pwmRight = constrain(pwmRight, 0, 255);

    // NOTE: your robot wiring uses motor1.forward + motor2.reverse for forward motion
    motor1.forward(pwmLeft);
    motor2.reverse(pwmRight);

    // Distance check
    long leftCount = abs(encoder1.getCount());
    long rightCount = abs(encoder2.getCount());
    long avgCount = (leftCount + rightCount) / 2;

    if (avgCount >= targetCounts) {
      motor1.stop();
      motor2.stop();
      return true; // completed normally
    }

    delay(10);
  }
}

void balanceMotorsWithPID(bool isForward) {
  int basePWM = 100;
  int pwmLeft = basePWM;
  int pwmRight = basePWM;

  static unsigned long prevTime = 0;
  static long prevLeftCount = 0;
  static long prevRightCount = 0;

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0f;
  if (dt < 0.02f) return;
  prevTime = now;

  long leftCount = encoder1.getCount();
  long rightCount = -encoder2.getCount();

  long deltaLeft = leftCount - prevLeftCount;
  long deltaRight = rightCount - prevRightCount;

  prevLeftCount = leftCount;
  prevRightCount = rightCount;

  float error = deltaLeft - deltaRight;

  static float smoothedError = 0;
  const float alpha = 0.1f;
  smoothedError = alpha * error + (1 - alpha) * smoothedError;

  float correction = pid.compute(0, smoothedError, dt);

  if (correction > 0) {
    pwmLeft = basePWM - abs(correction);
    pwmRight = basePWM + abs(correction) * 0.5f;
  } else if (correction < 0) {
    pwmLeft = basePWM + abs(correction) * 0.5f;
    pwmRight = basePWM - abs(correction);
  }

  pwmLeft = constrain(pwmLeft, 0, 255);
  pwmRight = constrain(pwmRight, 0, 255);

  if (isForward) {
    motor1.forward(pwmLeft);
    motor2.reverse(pwmRight);
  } else {
    motor1.reverse(pwmLeft);
    motor2.forward(pwmRight);
  }
}


// Call this function to move forward or backward a certain distance with balancing
void moveDistance(bool isForward, float distanceCM) {
  // Reset encoders at start
  encoder1.reset();
  encoder2.reset();

  // Convert target distance to encoder counts
  long targetCounts = distanceCM / cmPerCount;

  while (true) {
    // Get average encoder counts traveled (absolute values)
    long leftCount = abs(encoder1.getCount());
    long rightCount = abs(encoder2.getCount());

    // Average counts (optional for more accurate measurement)
    long avgCount = (leftCount + rightCount) / 2;

    if (avgCount >= targetCounts) {
      // Target reached - stop motors
      motor1.stop();
      motor2.stop();
      break;
    } else {
      // Still need to move - balance motors with PID
      balanceMotorsWithPID(isForward);
    }

    delay(10);  // small delay to avoid busy loop
  }
}

void setState(bool mForward, bool mReverse, bool mTurnLeft, bool mTurnRight, bool mStop){
  forward= mForward;
  reverse= mReverse;
  turnLeft = mTurnLeft;
  turnRight = mTurnRight;
  stop = mStop;
}

void Forward(){ setState(true,false,false,false,false); }
void Reverse(){ setState(false,true,false,false,false); }
void TurnRight(){ setState(false,false,false,true,false); }
void TurnLeft(){ setState(false,false,true,false,false); }
void Halt(){ setState(false,false,false,false,true); }


int task32() {
  imuOdom.update();
  float currentYaw;
  float ax, ay, az;
  imuOdom.getOrientation(ax, ay, currentYaw);
  imuOdom.getAcceleration(ax, ay, az);
  

  if (!hasTurnedInitially) {
    if (turnToYaw(-90)) {
      imuOdom.getOrientation(ax, ay, targetYaw);
      hasTurnedInitially = true;
      // Serial.print("Initial turn complete. Target yaw: ");
      // Serial.println(targetYaw);
      // Serial.println("Current yaw: ");
      // Serial.println(currentYaw);
    }
    return 0;
  }

  float accel_mag = sqrt(ax * ax + ay * ay + az * az);
  if (accel_mag < 0.7) {
    wasLifted = true;
  }

  if (turnToYaw(targetYaw)) {
    // Serial.println("Returned to original orientation.");
    // Serial.print(" Target yaw: ");
    // Serial.println(targetYaw);
    // Serial.println("Current yaw: ");
    // Serial.println(currentYaw);
    wasLifted = false;
  }
  return 0;
}

int task33() {
  static String command = "lflffrrfs";   // Command sequence: left, forward, left, forward, forward, right, right, forward, stop
  static int currentStep = 0;
  static bool executing = true;
  static float initialYaw = 0.0;
  static float targetYaw = 0.0;
  static bool hasStoredInitialYaw = false;
  static bool isTurning = false;
  static int turnCount = 0;  // To keep track of how many turns made

  if (!hasStoredInitialYaw) {
    imuOdom.update();
    float dummy1, dummy2;
    imuOdom.getOrientation(dummy1, dummy2, initialYaw);
    targetYaw = initialYaw;
    hasStoredInitialYaw = true;
    Serial.print("Stored initial yaw: ");
    Serial.println(initialYaw);
  }

  if (executing && currentStep < command.length()) {
    char action = command[currentStep];

    switch (action) {
      case 'f': {
        Serial.println("Action: FORWARD");
        float target_distance_mm = 30.0;
        Forward();
        if (encoder1.move(target_distance_mm, wheelDiameterCM) && encoder2.move(target_distance_mm, wheelDiameterCM)) {
          encoder1.reset();
          encoder2.reset();
          currentStep++;
          Halt();
          Serial.println("Forward movement complete");
        }
        break;
      }

      case 'l': {
        Serial.println("Action: LEFT TURN");

        if (!isTurning) {
          // Calculate new target yaw for left turn (+90 degrees from initial yaw per turn)
          float desiredAngle = fmod(initialYaw + 90.0 * (turnCount + 1), 360.0);
          if (desiredAngle > 180.0) desiredAngle -= 360.0;  // Normalize angle to [-180, 180]
          targetYaw = desiredAngle;
          Serial.print("Target Yaw for left turn: ");
          Serial.println(targetYaw);
          isTurning = true;
        }

        if (turnToYaw(targetYaw, 1, 30)) {  // Use your existing turnToYaw function
          encoder1.reset();
          encoder2.reset();
          isTurning = false;
          turnCount++;
          currentStep++;
          Serial.println("Left turn completed");
        }
        break;
      }

      case 'r': {
        Serial.println("Action: RIGHT TURN");

        if (!isTurning) {
          // Calculate new target yaw for right turn (-90 degrees from initial yaw per turn)
          float desiredAngle = fmod(initialYaw - 90.0 * (turnCount + 1), 360.0);
          if (desiredAngle > 180.0) desiredAngle -= 360.0;  // Normalize angle to [-180, 180]
          targetYaw = desiredAngle;
          Serial.print("Target Yaw for right turn: ");
          Serial.println(targetYaw);
          isTurning = true;
        }

        if (turnToYaw(targetYaw, 1, 30)) {
          encoder1.reset();
          encoder2.reset();
          isTurning = false;
          turnCount++;
          currentStep++;
          Serial.println("Right turn completed");
        }
        break;
      }

      case 's': {
        Serial.println("Action: STOP");
        Halt();
        executing = false;
        command = "";
        break;
      }

      default:
        Serial.print("Unknown command: ");
        Serial.println(action);
        currentStep++;
        break;
    }
  }

  if (executing && currentStep >= command.length()) {
    executing = false;
    command = "";
    Serial.println("Completed all commands");
  }

  return 0;
}


void setup() {
  Serial.begin(9600);
  Wire.begin();
  attachInterrupt(digitalPinToInterrupt(EN1A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EN2A), encoder2ISR, CHANGE);
  
  startLidars();

  encoder1.reset();
  encoder2.reset();

  imuOdom.begin();
}
int starttime = millis();

void startLidars() {
  pinMode(LIDAR1, OUTPUT);
  pinMode(LIDAR2, OUTPUT);
  pinMode(LIDAR3, OUTPUT);

  // Hold all in reset
  digitalWrite(LIDAR1, LOW);
  digitalWrite(LIDAR2, LOW);
  digitalWrite(LIDAR3, LOW);
  delay(10);

  // --- LEFT LIDAR ---
  digitalWrite(LIDAR1, HIGH);
  delay(10);
  leftLidar.init();     
  delay(10);

  // --- FRONT LIDAR ---
  digitalWrite(LIDAR2, HIGH);
  delay(10);
  frontLidar.init();
  delay(10);

  // --- RIGHT LIDAR ---
  digitalWrite(LIDAR3, HIGH);
  delay(10);
  rightLidar.init();
  delay(10);

  Serial.println("LIDARs initialized");
}

void executeCommandSequence(String command) {
  static int stepIndex = 0;
  static bool actionInProgress = false;
  static float currentHeading = 0.0;

  if (stepIndex >= command.length()) {
    motor1.stop();
    motor2.stop();
    return;  // Sequence complete
  }

  char action = command.charAt(stepIndex);

  switch (action) {
    case 'f': {
      if (!actionInProgress) {
        moveStraightWithHeadingCorrection(currentHeading, 19.0);
        actionInProgress = true;
      } else {
        stepIndex++;
        actionInProgress = false;
      }
      break;
    }

    case 'l': {
      float targetHeading = currentHeading + 90.0;
      if (targetHeading >= 360.0) targetHeading -= 360.0;

      if (turnToYaw(targetHeading, 2, 100)) {
        currentHeading = targetHeading;
        stepIndex++;
      }
      break;
    }

    case 'r': {
      float targetHeading = currentHeading - 90.0;
      if (targetHeading < 0.0) targetHeading += 360.0;

      if (turnToYaw(targetHeading, 2, 100)) {
        currentHeading = targetHeading;
        stepIndex++;
      }
      break;
    }

    case 's': {
      motor1.stop();
      motor2.stop();
      stepIndex = command.length();  // End sequence
      break;
    }

    default: {
      Serial.print("Unknown command: ");
      Serial.println(action);
      stepIndex++;
      break;
    }
  }

  delay(10);  // Small delay for stability
}

struct Step {
  float heading;
  float distance;
};

// Adjust to your maximum expected steps
const int MAX_STEPS = 10;
static Step steps[MAX_STEPS];
static int stepCount = 0;

void executeHeadingDistanceSequence(String sequence) {
  static int stepIndex = 0;
  static bool actionInProgress = false;
  static float currentHeading = 0.0;

  // Only parse sequence the first time
  if (stepCount == 0) {
    int start = 0;
    while (start < sequence.length() && stepCount < MAX_STEPS) {
      int hStart = sequence.indexOf('(', start);
      int comma = sequence.indexOf(',', hStart);
      int dEnd = sequence.indexOf(')', comma);
      if (hStart == -1 || comma == -1 || dEnd == -1) break;

      steps[stepCount].heading  = sequence.substring(hStart + 1, comma).toFloat();
      steps[stepCount].distance = sequence.substring(comma + 1, dEnd).toFloat();
      stepCount++;

      start = dEnd + 1;
    }
  }

  if (stepIndex >= stepCount) {
    motor1.stop();
    motor2.stop();
    return;
  }

  float targetHeading = steps[stepIndex].heading;
  float distance      = steps[stepIndex].distance;

  switch (actionInProgress) {
    case false: {
      if (turnToYaw(targetHeading, 2, 100)) {
        currentHeading = targetHeading;
        actionInProgress = true;
      }
      break;
    }

    case true: {
      moveStraightWithHeadingCorrection(currentHeading, distance);
      stepIndex++;
      actionInProgress = false;
      break;
    }
  }

  delay(10);
}

// Global state variables
enum ExploreState {DECIDE, TURNING, MOVING};
static ExploreState exploreState = DECIDE;
static float exploreHeading = 0.0;  // in degrees
static float moveDistanceCM = 19.0;
static float targetHeading = 0.0;

void mazeExplore() {
    static bool actionStarted = false;  // flag to track action progress

    switch (exploreState) {
        case DECIDE: {
            // Read LIDARs
            int distFront = frontLidar.readDistanceAndTrigger();
            int distLeft  = leftLidar.readDistanceAndTrigger();
            int distRight = rightLidar.readDistanceAndTrigger();

            Serial.print("Front: "); Serial.print(distFront);
            Serial.print(" | Left: "); Serial.print(distLeft);
            Serial.print(" | Right: "); Serial.println(distRight);

            const int obstacleThresholdLow = 0;
            const int obstacleThresholdHigh = 70;

            bool frontBlocked = distFront > 0 && distFront < obstacleThresholdHigh && distFront > obstacleThresholdLow;
            bool leftBlocked  = distLeft > 0 && distLeft < obstacleThresholdHigh && distLeft > obstacleThresholdLow;
            bool rightBlocked = distRight > 0 && distRight < obstacleThresholdHigh && distRight > obstacleThresholdLow;

            // Decide next action
            if (!frontBlocked) {
                exploreState = MOVING;
            } 
            else if (!rightBlocked) {
                targetHeading = exploreHeading - 90.0;
                if (targetHeading >= 360.0) targetHeading -= 360.0;
                exploreState = TURNING;
            } 
            else if (!leftBlocked) {
                targetHeading = exploreHeading + 90.0;
                if (targetHeading < 0.0) targetHeading += 360.0;
                exploreState = TURNING;
            } 
            else {
                targetHeading = exploreHeading + 180.0;
                if (targetHeading >= 360.0) targetHeading -= 360.0;
                exploreState = TURNING;
            }

            actionStarted = false;  // reset action flag
            break;
        }

        case TURNING: {
            if (!actionStarted) {
                Serial.print("Turning to "); Serial.println(targetHeading);
                actionStarted = true;
            }

            // Perform the turn
            if (turnToYaw(targetHeading)) {  // returns true when turn completed
                exploreHeading = targetHeading;  // update heading

                // Check front after turn
                int distFront = frontLidar.readDistanceAndTrigger();
                const int obstacleThresholdLow = 0;
                const int obstacleThresholdHigh = 70;
                bool frontBlocked = distFront > 0 && distFront < obstacleThresholdHigh && distFront > obstacleThresholdLow;

                if (!frontBlocked) {
                    exploreState = MOVING; // front is free → move
                } else {
                    exploreState = DECIDE; // front blocked → decide again
                }

                actionStarted = false;
            }
            break;
        }

        case MOVING: {
            if (!actionStarted) {
                Serial.println("Moving forward with wall alignment");
                actionStarted = true;
            }

            static float distLeftPrev = 0;
            static float distRightPrev = 0;
            static unsigned long lastCorrectionTime = 0;

            // Read side distances
            int distLeftRaw  = leftLidar.readDistanceAndTrigger();
            int distRightRaw = rightLidar.readDistanceAndTrigger();

            // Apply moving average / low-pass filter
            const float alpha = 0.3;
            float distLeft  = alpha * distLeftRaw  + (1 - alpha) * distLeftPrev;
            float distRight = alpha * distRightRaw + (1 - alpha) * distRightPrev;
            distLeftPrev  = distLeft;
            distRightPrev = distRight;

            // Only apply correction every 1 second
            unsigned long now = millis();
            if (now - lastCorrectionTime > 1000) {
                lastCorrectionTime = now;

                // Alignment settings
                const float desiredOffset = 25.0; // mm from single wall
                const float centerTolerance = 7.0; // mm tolerance
                const float headingAdjustStep = 0.5; // degrees per correction

                // Case 1: Both walls visible → center alignment
                if (distLeft > 0 && distRight > 0) {
                    float centerError = distLeft - distRight; // positive → closer to right wall
                    if (fabs(centerError) > centerTolerance) {
                        exploreHeading += (centerError > 0 ? headingAdjustStep : -headingAdjustStep);
                    }
                }
                // Case 2: Only left wall visible → keep fixed offset
                else if (distLeft > 0 && distRight <= 0) {
                    float error = distLeft - desiredOffset;
                    if (fabs(error) > centerTolerance) {
                        exploreHeading += (error < 0 ? headingAdjustStep : -headingAdjustStep); // corrected sign
                    }
                }
                // Case 3: Only right wall visible → keep fixed offset
                else if (distRight > 0 && distLeft <= 0) {
                    float error = distRight - desiredOffset;
                    if (fabs(error) > centerTolerance) {
                        exploreHeading += (error > 0 ? -headingAdjustStep : headingAdjustStep); // corrected sign
                    }
                }
            }

            // Move straight using corrected heading
            moveStraightWithHeadingCorrection(exploreHeading, moveDistanceCM);

            // Move finished, go back to decision state
            exploreState = DECIDE;
            actionStarted = false;
            break;
        }


    }
}


// String sequence = "(90, 20); (180, 15); (270, 25); (0, 10)";
// void loop() {
//   executeHeadingDistanceSequence(sequence);
// }


 
// // String command = "lfrflfffflfrflfrflfffflfs";
// String command = "rflfrffffrflfrflfrffffrfs";
// void loop() {
//   executeCommandSequence(command);
// }

void loop() {
  mazeExplore();
}