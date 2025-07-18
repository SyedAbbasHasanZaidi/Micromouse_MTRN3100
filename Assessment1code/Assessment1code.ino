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
// #include "PIDController.hpp"  // Disabled PID
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
bool turnToYaw(float targetYaw, int tolerance = 3, int speed = 40);

float ax_offset = 0.0;
float ay_offset = 0.0;
float gz_offset = 0.0;

MPU6050 imu;

// Initialize motors and encoders
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
mtrn3100::Encoder encoder1(EN1A, EN1B);
mtrn3100::Encoder encoder2(EN2A, EN2B);

// Initialise lidar
mtrn3100::Lidar lidar(LIDAR);

// Initialise IMU
mtrn3100::IMUOdometry imuOdom;

volatile bool objectDetected = false;

// LIDAR interrupt
void lidarISR() {
  objectDetected = true;
}

void setup() {
  Serial.begin(9600);
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

bool turnToYaw(float targetYaw, int tolerance, int speed) {
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

  static String command = "frfl";   // <-- Predefined command string
  static int currentStep = 0;
  static bool executing = true;         // <-- Start immediately
  static float targetYaw = 0.0;

  if (executing && currentStep < command.length()) {
    char action = command[currentStep];

    switch (action) {
      case 'f': {
        Serial.println("Action: FORWARD");

        encoder1.count = 0;
        encoder2.count = 0;

        float target_distance_mm = 100.0;

        motor1.forward(30);
        motor2.reverse(30);

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
        while(!turnToYaw(targetYaw));
        Serial.print("Left turn completed");
        break;
      }

      case 'r': {
        Serial.println("Action: RIGHT TURN");
        targetYaw += 90;\
        while(!turnToYaw(targetYaw));
        Serial.print(" turn completed");
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



void loop() {
  task32();
}



//arduino-cli compile --fqbn arduino:avr:nano "C:\Users\Admin\Documents\UNIVERSITY\MTRN3100\Micromouse_MTRN3100\Assessment1Code\Assessment1code.ino" 
//arduino-cli upload -p COM3 --fqbn arduino:avr:nano "C:\Users\Admin\Documents\UNIVERSITY\MTRN3100\Micromouse_MTRN3100\Assessment1Code\Assessment1code.ino"

// “left, forward, right, forward, forward, left, forward, right”