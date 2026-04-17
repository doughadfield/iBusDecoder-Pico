
/*
 * InternalPWM.h
 *
 *  Created on: Apr 1, 2023
 *      Author: Doug
 * This file contains code for generating PWM signals to control a servo motor
 * using the Raspberry Pi Pico's internal PWM hardware.
 * It is designed to be used in conjunction with the iBus receiver code to control a servo based on remote control input.
 * The code initializes the PWM hardware, sets the appropriate frequency for servo control,
 * and provides a function to set the servo position based on pulse width modulation.  
 * 
 */

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pwm.h"

// #define SERVO_FREQ_HZ   250     // 250Hz for servo control (4ms period, which allows for pulse widths up to 2.5ms - suitable for most modern digital servos)
#define SERVO_FREQ_HZ   80     // sound generator can't do 250Mhz; 80 is highest reliable frequency
#define MOTOR_FREQ_HZ   10415 // 10kHz for ESC control (10415 equiates to wrap=12000)
#define SERVO_MAX_FREQ_HZ 400   // above this, we assume it's an ESC driving a motor
// full 180degree range for modern digital servos is typically around 500–2500 µs
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

extern uint32_t servo_wrap[8]; // wrap value for each of the 8 PWM slices (0-7)

// Initialize PWM hardware for one or more servo pins
// pins[]   = array of GPIO numbers (must be PWM-capable)
// pulse_freq_hz = frequency of the PWM signal (e.g., slower for servos, faster for ESCs)
void hwpwm_init(const uint8_t pins[], uint8_t num_pins, uint pulse_freq_hz);

// Set pulse width (µs) for a specific servo GPIO
void servo_set_pulse_us(uint gpio, uint32_t pulse_us);

// Set pulse width (µs) for a specific motor GPIO
void motor_set_pulse_width(uint gpio, uint32_t pulse_width);

// Set angle in degrees (0–180°) for a specific servo GPIO
void servo_set_angle(uint gpio, float angle_deg);

// Quick center (1500 µs) for a servo
void servo_center(uint gpio);

#endif // SERVO_H
