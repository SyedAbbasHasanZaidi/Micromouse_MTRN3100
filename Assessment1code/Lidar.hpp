#ifndef LIDAR_HPP
#define LIDAR_HPP
 
#include <Wire.h>
#include <VL6180X.h>
 
namespace mtrn3100 {
 
class Lidar {
public:
    // alertXshutPin: single pin for both XSHUT and alert
    // newAddr: new I2C address to assign to this sensor
    Lidar(uint8_t alertXshutPin, uint8_t newAddr)
        : pin(alertXshutPin),
          i2cAddress(newAddr),
          filteredDistance(-1) {
          }
 
    void setAddress() {
        sensor.setAddress(i2cAddress);
    }
 
   void init() {
    Serial.println("[LIDAR] Initializing...");
 
    sensor.init();  // No return value
    Serial.println("[LIDAR] Sensor init() called.");
 
    sensor.configureDefault();
    Serial.println("[LIDAR] Default configuration applied.");
 
    sensor.setTimeout(1000);
    Serial.println("[LIDAR] Timeout set to 250ms.");
 
    Serial.println("[LIDAR] Setting I2C address...");
    setAddress();
    Serial.println("[LIDAR] Address set successfully.");
 
    // Optional: test read
    int testDist = sensor.readRangeSingleMillimeters();
    if (sensor.timeoutOccurred()) {
        Serial.println("[LIDAR] ERROR: Timeout during test read!");
    } else {
        Serial.print("[LIDAR] Test distance: ");
        Serial.print(testDist);
        Serial.println(" mm");
    }
    }
 
 
 
    int readDistanceAndTrigger() {
      int distance = sensor.readRangeSingleMillimeters();
 
      if (sensor.readRangeStatus() != VL6180X_ERROR_NONE) {
          return -1; // invalid reading
      }
 
      // Low-pass filter (EMA)
      const float alpha = 0.3f;
      if (filteredDistance < 0) {
          filteredDistance = distance;
      } else {
          filteredDistance = alpha * filteredDistance + (1 - alpha) * distance;
      }
 
      return static_cast<int>(filteredDistance);
  }
 
private:
    VL6180X sensor;
    uint8_t pin;         // shared pin for XSHUT + Alert
    uint8_t i2cAddress;  // assigned I2C address
    float filteredDistance;
};
 
} // namespace mtrn3100
 
#endif // LIDAR_HPP