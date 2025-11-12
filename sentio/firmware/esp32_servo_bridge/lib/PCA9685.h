#ifndef PCA9685_H
#define PCA9685_H

#include <Wire.h>
#include <Adafruit_PCA9685.h>

// PCA9685 register addresses
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_SUBADR1 0x02
#define PCA9685_SUBADR2 0x03
#define PCA9685_SUBADR3 0x04
#define PCA9685_ALLCALLADR 0x05
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_LED0_ON_H 0x07
#define PCA9685_LED0_OFF_L 0x08
#define PCA9685_LED0_OFF_H 0x09
#define PCA9685_ALL_LED_ON_L 0xFA
#define PCA9685_ALL_LED_ON_H 0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD
#define PCA9685_PRESCALE 0xFE

// Helper functions for PCA9685
inline uint16_t microsecondsToTicks(uint16_t microseconds, uint16_t frequency_hz = 50) {
  // Convert microseconds to PCA9685 ticks
  // frequency_hz * 4096 / 1000000 = ticks per microsecond
  uint32_t ticks_per_us = (frequency_hz * 4096) / 1000000;
  return (microseconds * ticks_per_us);
}

inline uint16_t degreesToMicroseconds(uint16_t degrees, uint16_t min_us = 1000, uint16_t max_us = 2000) {
  // Convert degrees (0-180) to microseconds (min_us-max_us)
  return min_us + (degrees * (max_us - min_us) / 180);
}

#endif // PCA9685_H
