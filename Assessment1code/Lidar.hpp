#ifndef LIDAR_HPP
#define LIDAR_HPP

#include <Wire.h>
#include <Adafruit_VL6180X.h>

namespace mtrn3100 {

class Lidar {
public:
    explicit Lidar(uint8_t alertPin) : interruptPin(alertPin) {}
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

    int readDistanceAndTrigger(int threshold) {
        unsigned long start = millis();
        int distance = -1;

        while (millis() - start < 100) { 
            distance = sensor.readRange();
            if (sensor.readRangeStatus() != VL6180X_ERROR_NONE) {
                delay(1);
                continue;
            }
            break; 
        }

        if (distance >= 0 && sensor.readRangeStatus() == VL6180X_ERROR_NONE) {
            if (distance < threshold) {
                digitalWrite(interruptPin, HIGH);
            } else {
                digitalWrite(interruptPin, LOW);
            }
            return distance;
        } else {
            digitalWrite(interruptPin, LOW);
            return -1;
        }
    }


private:
    Adafruit_VL6180X sensor;
    uint8_t interruptPin;
};

} // namespace mtrn3100

#endif // LIDAR_HPP
