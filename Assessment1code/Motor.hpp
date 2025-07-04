#pragma once

#include <Arduino.h>
#include <math.h>

namespace mtrn3100 {

class Motor {
public:
    Motor(uint8_t pwm_pin, uint8_t dir_pin) : pwm_pin(pwm_pin), dir_pin(dir_pin) {
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
    }

    void setPWM(int16_t pwm) {
        // Clamp PWM between -255 and 255
        pwm = constrain(pwm, -255, 255);

        if (pwm >= 0) {
            digitalWrite(dir_pin, HIGH);  // Forward
        } else {
            digitalWrite(dir_pin, LOW);   // Reverse
            pwm = -pwm; // Make pwm positive for analogWrite
        }

        analogWrite(pwm_pin, pwm);
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
};

}  // namespace mtrn3100
