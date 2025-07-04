#pragma once

#include <Arduino.h>
#include <math.h> // Needed for cos() and sin()

namespace mtrn3100 {

class EncoderOdometry {
public:
    EncoderOdometry(float radius, float wheelBase)
        : x(0), y(0), h(0), R(radius), B(wheelBase), lastLPos(0), lastRPos(0) {}

    void update(float leftValue, float rightValue) {
        // 1. Change in radians since the last update
        float delta_left_radians = leftValue - lastLPos;
        float delta_right_radians = rightValue - lastRPos;

        // 2. Convert to distance travelled by each wheel
        float d_left = R * delta_left_radians;
        float d_right = R * delta_right_radians;

        // 3. Compute forward kinematics
        float d = (d_left + d_right) / 2.0f;     // average linear displacement
        float dh = (d_right - d_left) / B;       // change in heading

        // 4. Update pose using midpoint heading for more accuracy
        x += d * cos(h + dh / 2.0f);
        y += d * sin(h + dh / 2.0f);
        h += dh;

        // 5. Store current values for next update
        lastLPos = leftValue;
        lastRPos = rightValue;

    }

    float getX() const { return x; }
    float getY() const { return y; }
    float getH() const { return h; }

private:
    float x, y, h;
    const float R, B;
    float lastLPos, lastRPos;
};

}
