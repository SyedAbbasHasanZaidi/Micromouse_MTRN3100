#pragma once

#include <Arduino.h>
#include "math.h"

namespace mtrn3100 {

class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2) : encoder1_pin(enc1), encoder2_pin(enc2) {
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);
    }

    void readEncoder() {
        if (digitalRead(encoder1_pin) == digitalRead(encoder2_pin)) {
            count++;
            direction = 1;
        } else {
            count--;
            direction = -1;
        }
    }

    float getRotation() {
        return (2 * PI * count / counts_per_revolution);
    }

public:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile long count = 0;
    volatile int8_t direction = 0;
    uint16_t counts_per_revolution = 700;
};

}  // namespace mtrn3100
