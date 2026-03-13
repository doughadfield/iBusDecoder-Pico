// header for pca9685.c library
#ifndef PCA9685_H
#define PCA9685_H
#endif // PCA9685_H
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// PCA9685 default I2C address
#define PCA9685_ADDR 0x40

// I2C configuration
#define I2C_PORT i2c0
#define SDA_PIN 4  // GPIO4 for SDA
#define SCL_PIN 5  // GPIO5 for SCL
#define I2C_BAUDRATE 400000  // 100 kHz

// Servo timing (µs) - adjust per your servo model
// #define SERVO_MIN_PULSE_US  1000     // some servos can only support 1000uS min
// #define SERVO_MAX_PULSE_US  2000     // some servos can only support 2000uS max
#define SERVO_MIN_PULSE_US  500      // ~0° - some servos support full 180degrees movement
#define SERVO_MAX_PULSE_US  2500     // ~180° - beware of servo binding at 0 or 180 degrees
// #define SERVO_FREQ_HZ       50       // Standard 20 ms period
#define SERVO_FREQ_HZ       250       // Faster 4 ms period for quicker response (adjust as needed)

// initialise I2C port and PCA9685 chip
extern void initialise_pca9685(); 
// set all PWM outputs to off (no pulses)
extern void pca9685_set_all_off();
// send raw pwm timing directly to the pwm hardware (0-4096 12bit resolution)
extern void pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off);
// send servo angle in degrees 
extern void pca9685_set_servo_angle(uint8_t channel, float angle_deg);
// send servo position in microseconds (500 to 2500us for full 180degrees of movement)
extern void pca9685_set_servo_position(uint8_t channel, uint16_t pulse_us);
