#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"  // Disabled PID
#include "Lidar.hpp"
#include <Wire.h>
#include "IMUOdometry.hpp"
#include <MPU6050.h>

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
bool turnToYaw(float targetYaw, int tolerance = 1, int speed = 40);

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
static float targetDistanceCM = 2000;
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
MPU6050 imu;

// Initialize motors and encoders
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
mtrn3100::Encoder encoder1(EN1A, EN1B,1400);
mtrn3100::Encoder encoder2(EN2A, EN2B,1400);
// Initialise pid
mtrn3100::PIDController pid(1,0.1,0.05);
// Initialise lidar
mtrn3100::Lidar lidar(LIDAR);
// Initialise IMU
mtrn3100::IMUOdometry imuOdom;

// LIDAR interrupt
void lidarISR() {
  objectDetected = true;
}
// Encoder interrupt service routines
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
  // Task 31
  objectDetected = false;
  // Task 32
  targetYaw = 0.0;
  hasTurnedInitially = false;
  wasLifted = false;
  //States
  turnLeft = false;
  turnRight = false;
  forward = false;
  reverse = false;
  stop = true;
}

bool turnToYaw(float targetYaw, int tolerance = 1, int speed = 40) {
    
    static bool getInitialYaw = false;
    // Read IMU data
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accel_x = ax / 16384.0;
  float accel_y = ay / 16384.0;
  float gyro_z_dps = gz / 131.0;

  imuOdom.update(accel_x, accel_y, gyro_z_dps);

  float currentYaw = imuOdom.getYaw();
  float rotationDiff = currentYaw - targetYaw;

  if (rotationDiff < -tolerance) {
    TurnRight();
    return false;
  } else if (rotationDiff > tolerance) {
    TurnLeft();
    return false;
  } else {
    Halt();
    return true;
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

    // Use direction flag
    if (isForward) {
        motor1.forward(pwmLeft);
        motor2.reverse(pwmRight);
    } else {
        motor1.reverse(pwmLeft);
        motor2.forward(pwmRight);
    }
}

void setState(bool mforward, bool mReverse, bool mTurnLeft, bool mTurnRight, bool mStop){
  forward= mforward;
  reverse= mReverse;
  turnLeft = mTurnLeft;
  turnRight = mTurnRight;
  stop = mStop;
}

void Forward(){
  setState(true,false,false,false,false);
}

void Reverse(){
  setState(false,true,false,false,false);
}

void TurnRight(){
  setState(false,false,false,true,false);
}

void TurnLeft(){
  setState(false,false,true,false,false);
}

void Halt(){
  setState(false,false,false,false,true);
}

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
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accel_x = ax / 16384.0;
  float accel_y = ay / 16384.0;
  float accel_z = az / 16384.0;
  float gyro_z_dps = gz / 131.0;

  imuOdom.update(accel_x, accel_y, gyro_z_dps);
  float currentYaw = imuOdom.getYaw();

  if (!hasTurnedInitially) {
    if (turnToYaw(-90)) {
      targetYaw = imuOdom.getYaw();
      hasTurnedInitially = true;
      Serial.print("Initial turn complete. Target yaw: ");
      Serial.println(targetYaw);
    }
    return 0;
  }

  float accel_mag = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
  if (accel_mag < 0.7) {
    wasLifted = true;
  }

  if (turnToYaw(targetYaw)) {
    Serial.println("Returned to original orientation.");
    wasLifted = false;
  }
  return 0;
}

int task33() {
  static String command = "frs";   // <-- s for stop flfrflfrs
  static int currentStep = 0;
  static bool executing = true;         // <-- Start immediately
  static float targetYaw;                // Initial heading
  static bool hasStoredInitialYaw = false;
  static bool isTurning = false;        // Tracks if currently turning

  if (!hasStoredInitialYaw) {
    targetYaw = imuOdom.getYaw();  // Store initial yaw once
    hasStoredInitialYaw = true;
    Serial.print("Stored yaw: ");
    Serial.println(targetYaw);
  }

  if (executing && currentStep < command.length()) {
    char action = command[currentStep];

    switch (action) {
      case 'f': {
        Serial.println("Action: FORWARD");
        float target_distance_mm = 100.0;
        Forward();
        if (encoder1.move(target_distance_mm, wheelDiameterCM) && encoder2.move(target_distance_mm, wheelDiameterCM)) {
          currentStep++;
          Halt();
        }
        break;
      }

      case 'l': {
        Serial.println("Action: LEFT TURN");

        if (!isTurning) {
          targetYaw -= 90;  // update once per turn command
          isTurning = true; 
        }

        if (turnToYaw(targetYaw, 1, 30)) {
          currentStep++;
          isTurning = false;
          Serial.println("Left turn completed");
        }
        break;
      }

      case 'r': {
        Serial.println("Action: RIGHT TURN");

        if (!isTurning) {
          targetYaw += 90;  // update once per turn command
          isTurning = true;
        }

        if (turnToYaw(targetYaw, 1, 30)) {
          currentStep++;
          isTurning = false;
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
        break;
    }
  }

  if (executing && currentStep >= command.length()) {
    executing = false;
    command = "";
  }

  return 0;
}


void setup() {
  Serial.begin(9600);
  // attaching interrupts 
  attachInterrupt(digitalPinToInterrupt(EN1A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EN2A), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIDAR), lidarISR, RISING);

  encoder1.reset();
  encoder2.reset(); // encoder resets 

  bool success = lidar.begin();
  if (success) {
    Serial.println("LIDAR initialized successfully.");
  } else {
    Serial.println("LIDAR initialization failed!");
    while (true);
  }

  Wire.begin();
  imu.initialize();

  if (imu.testConnection()) {
    Serial.println("IMU connected successfully.");
  } else {
    Serial.println("IMU connection failed!");
    while (true);
  }

  delay(1000);
  imuOdom.calibrateIMU(imu);
  imuOdom.calibrate(ax_offset, ay_offset, gz_offset);

}

int starttime = millis();

void loop() {
  //INSERT TASK HERE
  task33();
  if(forward || reverse){
    balanceMotorsWithPID(forward);  
  }

  else if(turnRight){
    motor1.forward(basePWM);
    motor2.forward(basePWM);
  }

  else if(turnLeft){
    motor1.reverse(basePWM);
    motor2.reverse(basePWM);
  }

  else{
    motor1.stop();
    motor2.stop();
    encoderReset();
    }
}




//arduino-cli compile --fqbn arduino:avr:nano "C:\Users\Admin\Documents\UNIVERSITY\MTRN3100\Micromouse_MTRN3100\Assessment1Code\Assessment1code.ino" 
//arduino-cli upload -p COM3 --fqbn arduino:avr:nano "C:\Users\Admin\Documents\UNIVERSITY\MTRN3100\Micromouse_MTRN3100\Assessment1Code\Assessment1code.ino"

// “left, forward, right, forward, forward, left, forward, right”