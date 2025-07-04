#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "EncoderOdometry.hpp"
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);


#define MOT1PWM 9
#define MOT1DIR 10
#define MOT2PWM 11
#define MOT2DIR 12

#define EN1A 2
#define EN1B 7
#define EN2A 3
#define EN2B 8

mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
mtrn3100::Encoder encoder1(EN1A, EN1B);
mtrn3100::Encoder encoder2(EN2A, EN2B);

// PID for left and right motors
mtrn3100::PIDController pid1(0.7, 0.01, 0.15);
mtrn3100::PIDController pid2(0.7, 0.01, 0.15);

// Encoder interrupt service routines
void encoder1ISR() {
  encoder1.readEncoder();
}
void encoder2ISR() {
  encoder2.readEncoder();
}
//for the odometer
mtrn3100::EncoderOdometry encoder_odometry(32,78.5); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH


void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(EN1A), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(EN2A), encoder2ISR, RISING);
  
  encoder1.count = 0;
  encoder2.count = 0;

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){return; } // stop everything if could not connect to MPU6050
    
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true);
  Serial.println("Done!\n");
}


// Constants for distance tracking
const float wheelDiameterCM = 3.55;
const float countsPerRev = 700.0;
const float cmPerCount = (PI * wheelDiameterCM) / countsPerRev; // ~0.0314 cm
const float targetDistanceCM = 21.5;
const int targetCounts = targetDistanceCM / cmPerCount; // ~6366 counts

void loop() {
  static bool hasMoved = false;
  static unsigned long lastTime = 0;
  const unsigned long interval = 20; // ms

  if (!hasMoved) {
    encoder1.count = 0;
    encoder2.count = 0;
    hasMoved = true;
  }

  if (millis() - lastTime >= interval && hasMoved) {
    lastTime = millis();

    // === Motor 1 PID ===
    int m1_error = -encoder1.count - targetCounts;  // motor1 is reversed
    int m1_pwm = pid1.compute(0, m1_error);
    m1_pwm = constrain(m1_pwm, -150, 150);
    if (abs(m1_pwm) < 60) m1_pwm = (m1_pwm >= 0) ? 60 : -60;
    motor1.setPWM(m1_pwm);

    // === Motor 2 PID ===
    int m2_error = encoder2.count - targetCounts;
    int m2_pwm = pid2.compute(0, m2_error);
    m2_pwm = constrain(m2_pwm, -150, 150);
    if (abs(m2_pwm) < 60) m2_pwm = (m2_pwm >= 0) ? 60 : -60;
    motor2.setPWM(-m2_pwm);  // motor2 reversed physically
  //odometer calculation and printing
  delay(50);
  encoder_odometry.update(encoder1.getRotation(),encoder2.getRotation());
  Serial.print("ODOM:\t\t");
  Serial.print(encoder_odometry.getX());
  Serial.print(",\t\t");
  Serial.print(encoder_odometry.getY());
  Serial.print(",\t\t");
  Serial.print(encoder_odometry.getH());


    // === Stop if both reached target ===
    if (abs(encoder1.count) >= targetCounts && abs(encoder2.count) >= targetCounts) {
      motor1.setPWM(0);
      motor2.setPWM(0);
      Serial.println("Target reached: 200 cm");
      while (true); // halt forever
    }

    

    // === Serial Output ===
    Serial.print("M1 Count: ");
    Serial.print(encoder1.count);
    Serial.print(" | M2 Count: ");
    Serial.println(encoder2.count);

  }
}
