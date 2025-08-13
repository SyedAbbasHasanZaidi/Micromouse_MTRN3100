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
// Objects 
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
mtrn3100::Encoder encoder1(EN1A, EN1B,1400);
mtrn3100::Encoder encoder2(EN2A, EN2B,1400);
mtrn3100::PIDController pid(1,0.1,0.05);
mtrn3100::Lidar lidar(LIDAR);
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

void moveStraightWithHeadingCorrection(float targetHeading, float distanceCM) {
  encoder1.reset();
  encoder2.reset();

  long targetCounts = distanceCM / cmPerCount;

  static float headingKp = 2.0;  
  static float headingKd = 0.5;
  static float headingKi = 0.0;
  static mtrn3100::PIDController headingPID(headingKp, headingKi, headingKd);

  float previousError = 0;
  float integral = 0;

  unsigned long prevTime = millis();

  while (true) {
    imuOdom.update();
    float currentYaw, dummy1, dummy2;
    imuOdom.getOrientation(dummy1, dummy2, currentYaw);

    // Calculate yaw error
    float error = targetHeading - currentYaw;

    // Normalize error to [-180, 180]
    if (error > 180) error -= 360;
    else if (error < -180) error += 360;

    // Time delta
    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.0f;
    prevTime = now;

    // PID correction
    float correction = headingPID.compute(0, error, dt);

    // Adjust motor speeds based on correction
    int pwmLeft = basePWM - correction;
    int pwmRight = basePWM + correction;

    pwmLeft = constrain(pwmLeft, 0, 255);
    pwmRight = constrain(pwmRight, 0, 255);

    motor1.forward(pwmLeft);
    motor2.reverse(pwmRight);

    // Check if target distance reached
    long leftCount = abs(encoder1.getCount());
    long rightCount = abs(encoder2.getCount());
    long avgCount = (leftCount + rightCount) / 2;

    if (avgCount >= targetCounts) {
      motor1.stop();
      motor2.stop();
      break;
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

int task31() {
  static bool hasMoved = false;
  const int obstacleThreshold = 100;
  const int tolerance = 5;

  if (!hasMoved) {
    encoder1.reset();
    encoder2.reset();
    hasMoved = true;
  }

  int distance = lidar.readDistanceAndTrigger(obstacleThreshold);

  if (distance >= 0) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  }

  if (distance < 0 || distance > 1000) {
    Forward();
    Serial.println("No object detected — moving FORWARD");
  } else if (distance <= obstacleThreshold + 5 && distance >= obstacleThreshold - tolerance) {
    Halt();
    Serial.println("Object within range! STOPPED.");
  } else if (distance > 0 && distance < obstacleThreshold - tolerance) {
    Reverse();
    Serial.println("REVERSE");
  } 
  return 0;
}

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
  attachInterrupt(digitalPinToInterrupt(LIDAR), lidarISR, RISING);

  encoder1.reset();
  encoder2.reset();

  bool success = lidar.begin();
  if (success) Serial.println("LIDAR initialized successfully.");
  else {
    Serial.println("LIDAR initialization failed!");
    while (true);
  }

  imuOdom.begin();
}
int starttime = millis();

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

String sequence = "(90, 20); (180, 15); (270, 25); (0, 10)";

void loop() {
  executeHeadingDistanceSequence(sequence);
}

 
// // String command = "lfrflfffflfrflfrflfffflfs";
// String command = "rflfrffffrflfrflfrffffrfs";
// void loop() {
//   executeCommandSequence(command);
// }