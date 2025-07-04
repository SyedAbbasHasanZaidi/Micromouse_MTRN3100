#ifndef LIDAR_HPP
#define LIDAR_HPP

#include <Wire.h>
#include <Adafruit_VL6180X.h>

namespace mtrn3100 {

class Lidar {
public:
    explicit Lidar(uint8_t alertPin) : interruptPin(alertPin) {}

    void begin() {
        Wire.begin();
        pinMode(interruptPin, OUTPUT);
        digitalWrite(interruptPin, LOW);

        if (!sensor.begin()) {
            while (true) {
                Serial.println("Failed to find VL6180X sensor!");
                delay(1000);
            }
        }
    }

    // Reads distance, triggers alertPin HIGH if below threshold, else LOW
    int readDistanceAndTrigger(int threshold) {
        int distance = sensor.readRange();
        if (sensor.readRangeStatus() == VL6180X_ERROR_NONE) {
            if (distance < threshold) {
                digitalWrite(interruptPin, HIGH);  // Object detected - trigger interrupt pin HIGH
            } else {
                digitalWrite(interruptPin, LOW);   // No object - pin LOW
            }
            return distance;
        }
        digitalWrite(interruptPin, LOW);  // Default LOW if error
        return -1;
    }

    uint8_t getInterruptPin() const {
        return interruptPin;
    }

private:
    Adafruit_VL6180X sensor;
    uint8_t interruptPin;
};

} // namespace mtrn3100

#endif // LIDAR_HPP
