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
 
#define MOT1PWM 9
#define MOT1DIR 10
#define MOT2PWM 11
#define MOT2DIR 12
 
#define EN1A 2
#define EN1B 7
#define EN2A 3
#define EN2B 8

#define LIDAR 4

// Initialize motors and encoders
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
mtrn3100::Encoder encoder1(EN1A, EN1B);
mtrn3100::Encoder encoder2(EN2A, EN2B);

// Disabled PID controllers
// mtrn3100::PIDController pid1(0.7, 0.01, 0.15);
// mtrn3100::PIDController pid2(0.7, 0.01, 0.15);

// Initialize lidar
mtrn3100::Lidar lidar(LIDAR);

volatile bool objectDetected = false;

// Encoder interrupt service routines
void encoder1ISR() {
  encoder1.readEncoder();
}
void encoder2ISR() {
  encoder2.readEncoder();
}
 
// LIDAR interrupt
void lidarISR(){
  objectDetected = true;
}

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(EN1A), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(EN2A), encoder2ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LIDAR), lidarISR, RISING);
  
  encoder1.count = 0;
  encoder2.count = 0;

  lidar.begin();
}

// Constants for distance tracking
const float wheelDiameterCM = 3.55;
const float countsPerRev = 700.0;
const float cmPerCount = (PI * wheelDiameterCM) / countsPerRev; // ~0.0314 cm
const float targetDistanceCM = 500;
const int targetCounts = targetDistanceCM / cmPerCount; // ~6366 counts
const int obstacleThreshold = 100;  // mm threshold for object detection

void loop() {
  static bool hasMoved = false;

  if (!hasMoved) {
    encoder1.count = 0;
    encoder2.count = 0;
    hasMoved = true;
  }

  int distance = lidar.readDistanceAndTrigger(obstacleThreshold);

  if (distance >= 0) { // Valid reading
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  } 
  //else {
  //   Serial.println("Distance reading error");
  // }

  if (distance <= obstacleThreshold + 5 && distance >= obstacleThreshold - 5 ) {
    // Object detected, stop motors
    motor1.setPWM(0);
    motor2.setPWM(0);
    Serial.println("Object detected! Motors stopped.");
  }else if(distance> 0 && distance < obstacleThreshold - 5 ){
    motor1.setPWM(-100);
    motor2.setPWM(100);
    Serial.println("REVERSE");
  } else {
    // No close object, move forward without PID
    motor1.setPWM(100); // Adjust sign/direction if motors are reversed
    motor2.setPWM(-100);  // Adjust as needed
    Serial.println("FORWARD");
  }


  delay(10);
}

