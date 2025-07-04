#pragma once

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd)
        : kp(kp), ki(ki), kd(kd), prev_error(0), integral(0) {}

    float compute(float target, float current) {
        float error = target - current;
        integral += error;
        float derivative = error - prev_error;
        prev_error = error;

        return kp * error + ki * integral + kd * derivative;
    }

private:
    float kp, ki, kd;
    float prev_error;
    float integral;
};

}  // namespace mtrn3100
