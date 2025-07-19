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

    void setPWM(uint8_t pwm) {
        analogWrite(pwm_pin, pwm);
    }

    void forward(int16_t pwm) {
        pwm = constrain(pwm, 0, 255);
        digitalWrite(dir_pin, HIGH);
        setPWM(pwm);
    }

    void reverse(int16_t pwm) {
        pwm = constrain(pwm, 0, 255);
        digitalWrite(dir_pin, LOW);
        setPWM(pwm);
    }       

    void stop() {
        setPWM(0);
    }

    void move(float distance_mm, mtrn3100::Encoder& encoder, int speed = 100, float wheel_radius = 3.55) {
        long initialCount = encoder.count;
        long targetCounts = (long)((abs(distance_mm) / (2 * PI * wheel_radius)) * encoder.counts_per_revolution);

        if (distance_mm >= 0) {
            forward(speed);
            while (abs(encoder.count - initialCount) < targetCounts) {
                // wait until target distance is reached
            }
        } else {
            reverse(speed);
            while (abs(encoder.count - initialCount) < targetCounts) {
                // wait until target distance is reached
            }
        }
        stop();
    }



private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
};

}  // namespace mtrn3100
