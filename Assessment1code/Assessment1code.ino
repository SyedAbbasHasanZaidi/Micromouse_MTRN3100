// #include "Encoder.hpp"
// #include "Motor.hpp"
// #include "PIDController.hpp"
// #include "Lidar.hpp"
 
// #define MOT1PWM 9
// #define MOT1DIR 10
// #define MOT2PWM 11
// #define MOT2DIR 12
 
// #define EN1A 2
// #define EN1B 7
// #define EN2A 3
// #define EN2B 8

// #define LIDAR 4
 
// mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
// mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
// mtrn3100::Encoder encoder1(EN1A, EN1B);
// mtrn3100::Encoder encoder2(EN2A, EN2B);
 
// // PID for left and right motors
// // mtrn3100::PIDController pid1(0.7, 0.01, 0.15);
// // mtrn3100::PIDController pid2(0.7, 0.01, 0.15);

// mtrn3100::Lidar lidar(LIDAR);

// volatile bool objectDetected = false;
 
// // Encoder interrupt service routines
// void encoder1ISR() {
//   encoder1.readEncoder();
// }
// void encoder2ISR() {
//   encoder2.readEncoder();
// }
 
// void lidarISR(){
//   objectDetected = true;
// }

// void setup() {
//   Serial.begin(9600);
//   attachInterrupt(digitalPinToInterrupt(EN1A), encoder1ISR, RISING);
//   attachInterrupt(digitalPinToInterrupt(EN2A), encoder2ISR, RISING);
//   attachInterrupt(digitalPinToInterrupt(LIDAR), lidarISR, RISING);
  
//   encoder1.count = 0;
//   encoder2.count = 0;

//   lidar.begin();
// }
 
// // Constants for distance tracking
// const float wheelDiameterCM = 3.55;
// const float countsPerRev = 700.0;
// const float cmPerCount = (PI * wheelDiameterCM) / countsPerRev; // ~0.0314 cm
// const float targetDistanceCM = 500;
// const int targetCounts = targetDistanceCM / cmPerCount; // ~6366 counts
// const int obstacleThreshold = 100;  // mm threshold for object detection
 
// void loop() {
//   static bool hasMoved = false;
//   static unsigned long lastTime = 0;
//   const unsigned long interval = 20; // PID update interval (ms)

//   if (!hasMoved) {
//     encoder1.count = 0;
//     encoder2.count = 0;
//     hasMoved = true;
//   }

//   int distance = lidar.readDistanceAndTrigger(obstacleThreshold);

//   if (distance >= 0) { // Valid reading
//     Serial.print("Distance: ");
//     Serial.print(distance);
//     Serial.println(" mm");
//   } else {
//     Serial.println("Distance reading error");
//   }

//   if (distance > 0 && distance < obstacleThreshold) {
//     // Object detected, stop motors
//     motor1.setPWM(0);
//     motor2.setPWM(0);
//     Serial.println("Object detected! Motors stopped.");
//   } else {
//     // No close object, continue PID control
//     if (millis() - lastTime >= interval && hasMoved) {
//       lastTime = millis();

//       int m1_error = -encoder1.count - targetCounts;  // motor1 reversed
//       int m1_pwm = pid1.compute(0, m1_error);
//       m1_pwm = constrain(m1_pwm, -150, 150);
//       if (abs(m1_pwm) < 60) m1_pwm = (m1_pwm >= 0) ? 60 : -60;
//       motor1.setPWM(m1_pwm);

//       int m2_error = encoder2.count - targetCounts;
//       int m2_pwm = pid2.compute(0, m2_error);
//       m2_pwm = constrain(m2_pwm, -150, 150);
//       if (abs(m2_pwm) < 60) m2_pwm = (m2_pwm >= 0) ? 60 : -60;
//       motor2.setPWM(-m2_pwm);  // motor2 reversed physically

//       if (abs(encoder1.count) >= targetCounts && abs(encoder2.count) >= targetCounts) {
//         motor1.setPWM(0);
//         motor2.setPWM(0);
//         Serial.println("Target reached!");
//         while (true);  // halt forever
//       }

//       Serial.print("M1 Count: ");
//       Serial.print(encoder1.count);
//       Serial.print(" | M2 Count: ");
//       Serial.println(encoder2.count);
//     }
//   }

//   delay(10);
// }


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
int imuTurn(char dir);
bool turnToYaw(float targetYaw, int tolerance = 1, int speed = 40);


float ax_offset = 0.0;
float ay_offset = 0.0;
float gz_offset = 0.0;

MPU6050 imu;

// Initialize motors and encoders
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
mtrn3100::Encoder encoder1(EN1A, EN1B,1400);
mtrn3100::Encoder encoder2(EN2A, EN2B,1400);

//mtrn3100::PIDController pid(2,0.3,0.05);
mtrn3100::PIDController pid(1,0.1,0.05);
// Initialise lidar
mtrn3100::Lidar lidar(LIDAR);

// Initialise IMU
mtrn3100::IMUOdometry imuOdom;

volatile bool objectDetected = false;

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

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(EN1A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EN2A), encoder2ISR, CHANGE);

  attachInterrupt(digitalPinToInterrupt(LIDAR), lidarISR, RISING);

  encoder1.count = 0;
  encoder2.count = 0;

  

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

  // Reset encoder counts
  encoder1.count = 0;
  encoder2.count = 0;

  
int basePWM = 100;
motor1.forward(basePWM);
motor2.reverse(basePWM); // assuming both forward now for same direction
}

// Constants for distance tracking
const float wheelDiameterCM = 3.55;
const float countsPerRev = 700.0;
const float cmPerCount = (PI * wheelDiameterCM) / countsPerRev; // ~0.0314 cm
const float targetDistanceCM = 2000;
const int targetCounts = targetDistanceCM / cmPerCount; // ~6366 counts



int task31() {
  static bool hasMoved = false;
  const int speed = 100;
  const int obstacleThreshold = 100;
  const int tolerance = 5;

  if (!hasMoved) {
    encoder1.count = 0;
    encoder2.count = 0;
    hasMoved = true;
  }

  int distance = lidar.readDistanceAndTrigger(obstacleThreshold);

  if (distance >= 0) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  }

  if (distance < 0 || distance > 1000) {
    motor1.forward(speed);
    motor2.reverse(speed);
    Serial.println("No object detected — moving FORWARD");
    return 0;
  } else if (distance <= obstacleThreshold + 5 && distance >= obstacleThreshold - tolerance) {
    motor1.stop();
    motor2.stop();
    Serial.println("Object within range! STOPPED.");
  } else if (distance > 0 && distance < obstacleThreshold - tolerance) {
    motor1.reverse(speed);
    motor2.forward(speed);
    Serial.println("REVERSE");
  } else {
    motor1.forward(speed);
    motor2.reverse(speed);
    Serial.println("FORWARD");
  }
  return 0;
}

bool turnToYaw(float targetYaw, int tolerance = 1, int speed = 40);

float targetYaw = 0.0;
bool hasTurnedInitially = false;
bool wasLifted = false;

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

bool turnToYaw(float targetYaw, int tolerance = 1, int speed = 40) {
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
    motor1.forward(speed);
    motor2.forward(speed);
    return false;
  } else if (rotationDiff > tolerance) {
    motor1.reverse(speed);
    motor2.reverse(speed);
    return false;
  } else {
    motor1.stop();
    motor2.stop();
    return true;
  }
}

int imuTurn(char dir) {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accel_x = ax / 16384.0;
  float accel_y = ay / 16384.0;
  float gyro_z_dps = gz / 131.0;

  imuOdom.update(accel_x, accel_y, gyro_z_dps);
  int change = (dir == 'l') ? -90 : 90;

  int rotationDiff = gyro_z_dps + change;

  while ((rotationDiff < -3) || (rotationDiff > 3)) {
    if (rotationDiff < -3) {
      motor1.setPWM(100);
      motor2.setPWM(100);
    } else if (rotationDiff > 3) {
      motor1.setPWM(-100);
      motor2.setPWM(-100);
    }

    imuOdom.update(accel_x, accel_y, gyro_z_dps);
    rotationDiff = gyro_z_dps + change;
  }

  motor1.setPWM(0);
  motor2.setPWM(0);
  return 0;
}


int task33() {
  // if (Serial.available() > 0 && command == "") {
  //   command = Serial.readStringUntil('\n');
  //   command.trim();
  //   Serial.print("Received command: ");
  //   Serial.println(command);
  //   currentStep = 0;
  //   executing = true;
  //   targetYaw = imuOdom.getYaw(); // Set initial heading
  // }

  static String command = "flfrflfrs";   // <-- s for stop
  static int currentStep = 0;
  static bool executing = true;         // <-- Start immediately
  static float targetYaw = imuOdom.getYaw(); // Initial heading

  if (executing && currentStep < command.length()) {
    char action = command[currentStep];

    switch (action) {
      case 'f': {
        Serial.println("Action: FORWARD");

        encoder1.count = 0;
        encoder2.count = 0;

        float target_distance_mm = 400.0;

        motor1.forward(70);
        motor2.reverse(70);

        while (!encoder1.move(target_distance_mm, wheelDiameterCM) && !encoder2.move(target_distance_mm, wheelDiameterCM)) {
          encoder1.readEncoder();
          encoder2.readEncoder();
        }

        motor1.stop();
        motor2.stop();
        break;
      }

      case 'l': {
        Serial.println("Action: LEFT TURN");
        targetYaw -= 90;
        while(!turnToYaw(targetYaw, 1 , 30));
        Serial.print("Left turn completed");
        break;
      }

      case 'r': {
        Serial.println("Action: RIGHT TURN");
        targetYaw += 90;
        while(!turnToYaw(targetYaw, 1 , 30));
        Serial.print(" turn completed");
        break;
      }
      
      
      case 's': {
        Serial.println("Action: STOP");
        motor1.stop();
        motor2.stop();
        executing = false;
        command = "";
        break;
      }

      default:
        Serial.print("Unknown command: ");
        Serial.println(action);
        break;
    }

    currentStep++;
  }

  if (executing && currentStep >= command.length()) {
    executing = false;
    command = "";
  }

  return 0;
}

// void balanceMotorsWithPID() {
//     int basePWM = 50;
//     int pwmLeft = basePWM;
//     int pwmRight = basePWM;

//     static unsigned long prevTime = 0;
//     static long prevLeftCount = 0;
//     static long prevRightCount = 0;

//     unsigned long now = millis();
//     float dt = (now - prevTime) / 1000.0f;

//     if (dt < 0.02f) return; // run every ~50ms

//     long leftCount = encoder1.getCount();
//     long rightCount = - 1*encoder2.getCount();

//     long deltaLeft = leftCount - prevLeftCount;
//     long deltaRight = rightCount - prevRightCount;

//     float error = deltaLeft - deltaRight;
//     static float smoothedError = 0;
//     const float alpha = 0.4;  // smoothing factor (0.1–0.5 good range)
//     smoothedError = alpha * error + (1 - alpha) * smoothedError;

//     float correction = pid.compute(0, smoothedError, dt);  // target is 0 error
//     // correction = constrain(correction, -40, 40);

//     if (abs(correction) < 2) correction = 0;

//     // Asymmetric but PHYSICALLY CORRECT approach
//   if (error > 0) {  // Left is leading
//       pwmLeft = basePWM - abs(correction);  // Slow down left
//       pwmRight = basePWM;                   // Maintain right
//   } 
//   else if (error < 0) {  // Right is leading
//       pwmLeft = basePWM;                   // Maintain left
//       pwmRight = basePWM - abs(correction);  // Slow down right
//   } 
//   else {
//       pwmLeft = basePWM;
//       pwmRight = basePWM;
//   }

// // Apply constraints
// // pwmLeft = constrain(pwmLeft, 120, 180);
// // pwmRight = constrain(pwmRight, 120, 180);

//     motor1.setPWM(pwmLeft);   // just sets PWM (forwards only)
//     motor2.setPWM(pwmRight); // just sets PWM (forwards only)

//     prevTime = now;
//     prevLeftCount = leftCount;
//     prevRightCount = rightCount;
// }

void balanceMotorsWithPID() {
    int basePWM = 100;
    int pwmLeft = basePWM;
    int pwmRight = basePWM;

    static unsigned long prevTime = 0;
    static long prevLeftCount = 0;
    static long prevRightCount = 0;

    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.0f;

    if (dt < 0.02f) return; // run every ~50ms
    prevTime = now;

    long leftCount = encoder1.getCount();
    long rightCount = -encoder2.getCount();

    long deltaLeft = leftCount - prevLeftCount;
    long deltaRight = rightCount - prevRightCount;

    prevLeftCount = leftCount;
    prevRightCount = rightCount;

    float error = deltaLeft - deltaRight;
    static float smoothedError = 0;
    const float alpha = 0.1f;  // more smoothing
    smoothedError = alpha * error + (1 - alpha) * smoothedError;

    float correction = pid.compute(0, smoothedError, dt);  // target 0 error

    if (correction > 0) {  // Left motor is leading
        pwmLeft = basePWM - abs(correction);
        pwmRight = basePWM + abs(correction) * 0.5f;  // small boost on right
    } else if (correction < 0) {  // Right motor is leading
        pwmLeft = basePWM + abs(correction) * 0.5f;   // small boost on left
        pwmRight = basePWM - abs(correction);
    } else {
        pwmLeft = basePWM;
        pwmRight = basePWM;
    }

    // Clamp PWM values to valid range (0-255)
    pwmLeft = constrain(pwmLeft, 0, 255);
    pwmRight = constrain(pwmRight, 0, 255);

    // Set motor PWM outputs here
  motor1.setPWM(pwmLeft);
  motor2.setPWM(pwmRight);
}


// long startLeftCount = 0;
// long startRightCount = 0;
// float integralCumulativePosition = 0;
// float derivative_error = 0;
// float prev_error = 0;

// void balanceMotorsWithPID() {
//   const int basePWM = 150;
//   static unsigned long prevTime = 0;
//   static long prevLeftCount = 0;
//   static long prevRightCount = 0;

//   // Timing control (20ms interval)
//   if (millis() - prevTime < 20) return;

//   encoder1.readEncoder();
//   encoder2.readEncoder();
//   long leftCount = encoder1.count;
//   long rightCount = -encoder2.count;  // Invert right encoder

//   // Cap max delta to prevent spikes from encoder glitches
//   long maxTickJump = 50;
//   long leftDelta = constrain(leftCount - prevLeftCount, -maxTickJump, maxTickJump);
//   long rightDelta = constrain(rightCount - prevRightCount, -maxTickJump, maxTickJump);

//   // Calculate raw errors
//   long rawPositionError = (leftCount - startLeftCount) - (rightCount - startRightCount);
//   long rawSpeedError = leftDelta - rightDelta;

//   // Low-pass filtering (Exponential Moving Average)
//   static float filteredPositionError = 0;
//   static float filteredSpeedError = 0;
//   const float alpha = 0.5;  // Smoothing factor

//   filteredPositionError = alpha * rawPositionError + (1 - alpha) * filteredPositionError;
//   filteredSpeedError = alpha * rawSpeedError + (1 - alpha) * filteredSpeedError;

//   // Compute PID correction
//   float dt = (millis() - prevTime) / 1000.0f;

//   integralCumulativePosition = constrain(integralCumulativePosition + filteredPositionError * dt, -1000, 1000);

//   const float Kp_pos = 1;
//   const float Ki_pos = 1;
//   const float Kd_pos =1;

//   static float prev_error = 0;
//   float derivative_error = (filteredPositionError - prev_error) / dt;
//   prev_error = filteredPositionError;

//   float correction = (Kp_pos * filteredPositionError) +
//                      (Ki_pos * integralCumulativePosition) +
//                      (Kd_pos * derivative_error);

//   // Apply deadband
//   if (abs(correction) < 3) correction = 0;

//   // ASYMMETRIC CORRECTION (Slows down leading wheel)
//   int pwmLeft, pwmRight;

//   if (filteredPositionError > 0) {  // Left is leading
//     pwmLeft = basePWM - abs(correction);
//     pwmRight = basePWM + abs(correction);
//   } else if (filteredPositionError < 0) {  // Right is leading
//     pwmLeft = basePWM + abs(correction);
//     pwmRight = basePWM - abs(correction);
//   } else {
//     pwmLeft = basePWM;
//     pwmRight = basePWM;
//   }

//   // Apply PWM constraints
//   pwmLeft = constrain(pwmLeft, 120, 180);
//   pwmRight = constrain(pwmRight, 120, 180);

//   // Drive motors
//   motor1.setPWM(pwmLeft);
//   motor2.setPWM(pwmRight);

//   // Update previous values
//   prevTime = millis();
//   prevLeftCount = leftCount;
//   prevRightCount = rightCount;
// }




int starttime = millis();
// void loop() {
//   float target_distance_cm = 180.0;
//   int targetCountsLeft = target_distance_cm / (3.55) * encoder1.counts_per_revolution;
//   int targetCountsRight = target_distance_cm / (3.55) * encoder2.counts_per_revolution;

//   encoder1.readEncoder();
//   encoder2.readEncoder();

//   long leftCount = encoder1.count;
//   long rightCount = -encoder2.count;

//   Serial.print("timestamp: ");
//   Serial.print(starttime + millis());
//   Serial.print(" |Left Count: ");
//   Serial.print(leftCount);
//   Serial.print(" | Right Count: ");
//   Serial.print(rightCount);
//   Serial.print(" | Diff: ");
//   Serial.println(leftCount - rightCount);  // Corrected diff

//   // long leftTravel = leftCount - startLeftCount;
//   // long rightTravel = rightCount - startRightCount;

//   balanceMotorsWithPID();  // Uncomment when needed
// }

void loop() {
  float target_distance_cm = 180.0;
  int targetCountsLeft = target_distance_cm / (3.55) * encoder1.counts_per_revolution;
  int targetCountsRight = target_distance_cm / (3.55) * encoder2.counts_per_revolution;

  encoder1.readEncoder();
  encoder2.readEncoder();

  long leftCount = encoder1.count;
  long rightCount = -encoder2.count;

  Serial.print("timestamp: ");
  Serial.print(starttime + millis());
  Serial.print(" | Left Count: ");
  Serial.print(leftCount);
  Serial.print(" | Right Count: ");
  Serial.print(rightCount);
  Serial.print(" | Diff: ");
  Serial.println(leftCount - rightCount);

  // Check if target reached
  if (leftCount >= targetCountsLeft && rightCount >= targetCountsRight) {
    // Target reached — stop motors
    motor1.stop();
    motor2.stop();
  } else {
    // Not yet reached — run PID balancing
    balanceMotorsWithPID();
  }
}




  // Balance motor speed using PID
  //balanceMotorsWithPID();


// void loop() {
//   encoder1.readEncoder();
//   encoder2.readEncoder();

//   Serial.print("Left Encoder Count: ");
//   Serial.print(encoder1.count);
//   Serial.print(" | Right Encoder Count: ");
//   Serial.println(encoder2.count);

//   delay(100); // print every 100ms
// }




//arduino-cli compile --fqbn arduino:avr:nano "C:\Users\Admin\Documents\UNIVERSITY\MTRN3100\Micromouse_MTRN3100\Assessment1Code\Assessment1code.ino" 
//arduino-cli upload -p COM3 --fqbn arduino:avr:nano "C:\Users\Admin\Documents\UNIVERSITY\MTRN3100\Micromouse_MTRN3100\Assessment1Code\Assessment1code.ino"

// “left, forward, right, forward, forward, left, forward, right”