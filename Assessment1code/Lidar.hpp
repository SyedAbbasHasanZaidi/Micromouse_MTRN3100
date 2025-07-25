// #ifndef LIDAR_HPP
// #define LIDAR_HPP

// #include <Wire.h>
// #include <Adafruit_VL6180X.h>

// namespace mtrn3100 {

// class Lidar {
// public:
//     explicit Lidar(uint8_t alertPin) : interruptPin(alertPin) {}

//     bool begin() {
//         Wire.begin();
//         pinMode(interruptPin, OUTPUT);
//         digitalWrite(interruptPin, LOW);

//         if (!sensor.begin()) {
//             Serial.println("Failed to find VL6180X sensor!");
//             return false;
//         }

//         Serial.println("VL6180X sensor initialized successfully.");
//         return true;
//     }

//     // Reads distance, triggers alertPin HIGH if below threshold, else LOW
//     int readDistanceAndTrigger(int threshold) {
//         unsigned long start = millis();
//         int distance = -1;

//         while (millis() - start < 100) {  // 100 ms timeout
//             distance = sensor.readRange();
//             if (sensor.readRangeStatus() != VL6180X_ERROR_NONE) {
//                 delay(1);
//                 continue;
//             }
//             break; // got a valid reading
//         }

//         if (distance >= 0 && sensor.readRangeStatus() == VL6180X_ERROR_NONE) {
//             if (distance < threshold) {
//                 digitalWrite(interruptPin, HIGH);
//             } else {
//                 digitalWrite(interruptPin, LOW);
//             }
//             return distance;
//         } else {
//             digitalWrite(interruptPin, LOW);
//             return -1;
//         }
//     }


// private:
//     Adafruit_VL6180X sensor;
//     uint8_t interruptPin;
// };

// } // namespace mtrn3100

// #endif // LIDAR_HPP

#ifndef LIDAR_HPP
#define LIDAR_HPP

#include <Wire.h>
#include <Adafruit_VL6180X.h>

namespace mtrn3100 {

class Lidar {
public:
    explicit Lidar(uint8_t alertPin) : interruptPin(alertPin) {}
    explicit Lidar(uint8_t alertPin) 
        : interruptPin(alertPin), filteredDistance(-1) {}

    bool begin() {
        Wire.begin();
        pinMode(interruptPin, OUTPUT);
        digitalWrite(interruptPin, LOW);

        if (!sensor.begin()) {
            Serial.println("Failed to find VL6180X sensor!");
            return false;
        }

        Serial.println("VL6180X sensor initialized successfully.");
        return true;
    }

    // Reads distance, triggers alertPin HIGH if below threshold, else LOW
    int readDistanceAndTrigger(int threshold) {
        unsigned long start = millis();
        int distance = -1;

        while (millis() - start < 100) {  // 100 ms timeout
            distance = sensor.readRange();
            if (sensor.readRangeStatus() != VL6180X_ERROR_NONE) {
                delay(1);
                continue;
            }
            break; // got a valid reading
        }

        if (distance >= 0 && sensor.readRangeStatus() == VL6180X_ERROR_NONE) {
            // Low-pass filter (EMA)
            const float alpha = 0.7f;
            if (filteredDistance < 0) {
                // Initialize filter on first valid reading
                filteredDistance = distance;
            } else {
                filteredDistance = alpha * filteredDistance + (1 - alpha) * distance;
            }

            int smoothedDistance = static_cast<int>(filteredDistance);

            if (smoothedDistance < threshold) {
                digitalWrite(interruptPin, HIGH);
            } else {
                digitalWrite(interruptPin, LOW);
            }

            return smoothedDistance;
        } else {
            digitalWrite(interruptPin, LOW);
            return -1;
        }
    }

private:
    Adafruit_VL6180X sensor;
    uint8_t interruptPin;
    float filteredDistance;  // for low-pass smoothing
};

} // namespace mtrn3100

#endif // LIDAR_HPP

