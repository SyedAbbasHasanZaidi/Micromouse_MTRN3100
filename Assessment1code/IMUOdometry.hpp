#ifndef IMU_ODOMETRY_HPP
#define IMU_ODOMETRY_HPP

#include <Wire.h>
#include <MPU6050.h>

class IMU {
private:
  MPU6050 imu;

  // Biases for accelerometer
  float ax_bias;
  float ay_bias;
  float az_bias;

  // Bias for gyro z-axis (yaw)
  float gz_bias;

  // Constants
  static constexpr float G_TO_MS2 = 9.81f;      // conversion factor
  static constexpr float alpha = 0.98f;         // complementary filter coefficient for filter

  // Angles
  float pitch;
  float roll;
  float yaw;

  // Timing
  unsigned long prevTime;

  // Store latest acceleration for print()
  float ax, ay, az;

  // Calibrate accelerometer biases by averaging N stationary samples.
  // Input: samples - number of samples to average
  // Output: none (updates ax_bias, ay_bias, az_bias)
  void calibrateAccel(int samples);

  // Calibrate gyro Z-axis bias (yaw) by averaging N stationary samples.
  // Input: samples - number of samples to average
  // Output: none (updates gz_bias)
  void calibrateGyroZ(int samples);

public:
  // Default constructor
  IMU();

  // Initialize MPU6050 and calibrate sensors.
  // Input: none
  // Output: none
  void begin();

  // Read sensor data, correct biases, and update pitch, roll, yaw.
  // Input: none
  // Output: none (updates pitch, roll, yaw, ax, ay, az)
  void update();

  // Print acceleration (m/s^2) and orientation angles (degrees) to Serial.
  // Input: none
  // Output: none (prints to Serial)
  void print();
  // Reset orientation estimates (pitch, roll, yaw) to zero.
// Input: none
// Output: none (resets internal state)
void reset();

// Get current orientation angles (pitch, roll, yaw).
// Input: references to store values
// Output: pitch, roll, yaw set via references
void getOrientation(float& pitch_out, float& roll_out, float& yaw_out);

// Get current linear acceleration (ax, ay, az) in m/s^2.
// Input: references to store values
// Output: ax, ay, az set via references
void getAcceleration(float& ax_out, float& ay_out, float& az_out);

};

#endif // IMU_ODOMETRY_HPP
