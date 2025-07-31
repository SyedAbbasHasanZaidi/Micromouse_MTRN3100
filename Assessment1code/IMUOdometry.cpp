#include "IMUOdometry.hpp"
#include <math.h> 

IMU::IMU()
  : ax_bias(0), ay_bias(0), az_bias(0),
    gz_bias(0), pitch(0), roll(0), yaw(0),
    prevTime(0), ax(0), ay(0), az(0) {}

void IMU::begin() {
  Wire.begin();
  Serial.println("Initializing MPU6050...");
  imu.initialize();
  imu.setDLPFMode(MPU6050_DLPF_BW_5);

  if (!imu.testConnection()) {
    Serial.println("MPU6050 connection failed. Check wiring.");
    while (1);
  }

  Serial.println("MPU6050 connected. Starting calibration...");
  calibrateAccel(1000);
  calibrateGyroZ(2000);
  Serial.println("Calibration complete.");
  delay(500);

  prevTime = micros();
}

void IMU::update() {
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;
  imu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

  // Correct accelerometer raw data by subtracting bias
  float ax_g = ax_raw / 16384.0f - ax_bias;
  float ay_g = ay_raw / 16384.0f - ay_bias;
  float az_g = az_raw / 16384.0f - az_bias;

  // Convert to m/s²
  float ax_ms2 = ax_g * G_TO_MS2;
  float ay_ms2 = ay_g * G_TO_MS2;
  float az_ms2 = az_g * G_TO_MS2;

  // Correct gyro raw data (deg/s) and subtract bias
  float gx = (gx_raw / 131.0f);
  float gy = (gy_raw / 131.0f);
  float gz = (gz_raw / 131.0f) - gz_bias;

  // Calculate elapsed time in seconds
  unsigned long currentTime = micros();
  float dt = (currentTime - prevTime) / 1000000.0f;
  prevTime = currentTime;

  // Calculate pitch and roll from accelerometer (radians)
  float pitch_acc = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g));
  float roll_acc  = atan2(-ax_g, az_g);

  // Convert pitch_acc and roll_acc from radians to degrees
  pitch_acc = pitch_acc * RAD_TO_DEG;
  roll_acc  = roll_acc * RAD_TO_DEG;

  // Complementary filter to combine gyro and accel data for pitch and roll
  pitch = alpha * (pitch + gy * dt) + (1 - alpha) * pitch_acc;
  roll  = alpha * (roll + gx * dt)  + (1 - alpha) * roll_acc;

  // Integrate gyro z-axis to get yaw (deg)
  yaw += gz * dt;

  // Normalize yaw to [-180, 180]
  if (yaw > 180) yaw -= 360;
  else if (yaw < -180) yaw += 360;

  // Save latest acceleration for printing
  ax = ax_ms2;
  ay = ay_ms2;
  az = az_ms2;
}

void IMU::print() {
  Serial.print("\033[2J");  // Clear screen
  Serial.print("\033[H");   // Cursor to top-left

  Serial.print("Accel [m/s^2]:\n");
  Serial.print(" X: ");
  Serial.print(ax, 2);
  Serial.print("\n Y: ");
  Serial.print(ay, 2);
  Serial.print("\n Z: ");
  Serial.print(az, 2);

  Serial.print("\n\nAngles:\n");
  Serial.print(" Pitch: ");
  Serial.print(pitch, 2);
  Serial.print("°\n Roll: ");
  Serial.print(roll, 2);
  Serial.print("°\n Yaw: ");
  Serial.print(yaw, 2);
  Serial.print("°\n");
}


void IMU::calibrateAccel(int samples) {
  long sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    imu.getAcceleration(&ax, &ay, &az);

    sumX += ax;
    sumY += ay;
    sumZ += az;

    delay(5);
  }

  ax_bias = (sumX / (float)samples) / 16384.0f;
  ay_bias = (sumY / (float)samples) / 16384.0f;
  az_bias = (sumZ / (float)samples) / 16384.0f - 1.0f;

  Serial.print("Accel bias [g]: X=");
  Serial.print(ax_bias, 4);
  Serial.print(" Y=");
  Serial.print(ay_bias, 4);
  Serial.print(" Z=");
  Serial.println(az_bias, 4);
}

void IMU::calibrateGyroZ(int samples) {
  long sumGz = 0;

  Serial.println("Calibrating gyro Z bias... Keep MPU still!");

  for (int i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    imu.getRotation(&gx, &gy, &gz);

    sumGz += gz;
    delay(5);
  }

  gz_bias = (sumGz / (float)samples) / 131.0f;

  Serial.print("Gyro Z bias (deg/s): ");
  Serial.println(gz_bias, 6);
}

void IMU::reset() {
  pitch = 0.0f;
  roll = 0.0f;
  yaw = 0.0f;
}

void IMU::getOrientation(float& pitch_out, float& roll_out, float& yaw_out) {
  pitch_out = pitch;
  roll_out = roll;
  yaw_out = yaw;
}

void IMU::getAcceleration(float& ax_out, float& ay_out, float& az_out) {
  ax_out = ax;
  ay_out = ay;
  az_out = az;
}



