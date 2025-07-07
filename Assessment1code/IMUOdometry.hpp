#ifndef IMU_ODOMETRY_HPP
#define IMU_ODOMETRY_HPP

#include <Arduino.h>

namespace mtrn3100 {
    class IMUOdometry {
    public:
        IMUOdometry()
            : x(0), y(0), vx(0), vy(0), yaw(0),
              ax_offset(0), ay_offset(0), gz_offset(0),
              lastUpdateTime(millis()) {}

        void calibrate(float ax_off, float ay_off, float gz_off) {
            ax_offset = ax_off;
            ay_offset = ay_off;
            gz_offset = gz_off;
        }

        void update(float accel_x, float accel_y, float gyro_z_dps) {
            unsigned long currentTime = millis();
            float dt = (currentTime - lastUpdateTime) / 1000.0; // seconds
            lastUpdateTime = currentTime;

            // Apply offsets
            accel_x -= ax_offset;
            accel_y -= ay_offset;
            gyro_z_dps -= gz_offset;

            // Threshold to reduce drift
            if (abs(accel_x) < 0.03) accel_x = 0;
            if (abs(accel_y) < 0.03) accel_y = 0;
            if (abs(gyro_z_dps) < 0.5) gyro_z_dps = 0;

            // Integrate acceleration to get velocity
            vx += accel_x * dt;
            vy += accel_y * dt;

            // Integrate velocity to get position
            x += vx * dt;
            y += vy * dt;

            // Integrate gyro to get yaw (degrees)
            yaw += gyro_z_dps * dt;
        }

        float getX() const { return x; }
        float getY() const { return y; }
        float getYaw() const { return yaw; }

    private:
        float x, y;
        float vx, vy;
        float yaw;
        float ax_offset, ay_offset, gz_offset;
        unsigned long lastUpdateTime;
    };
}

#endif // IMU_ODOMETRY_HPP
