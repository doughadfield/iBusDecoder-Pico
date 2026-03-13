
/*
 * InternalPWM.c
 *
 *  Created on: Apr 1, 2023
 *      Author: Doug
 * This file contains code for generating PWM signals to control a servo motor
 * using the Raspberry Pi Pico's internal PWM hardware.
*/


#include <stdio.h>
#include <pico/types.h>
#include "InternalPWM.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// Internal state
uint32_t servo_wrap[8]    = {0}; // wrap value for each of the 8 PWM slices (0-7)


// Initialize PWM on multiple GPIO pins
void hwpwm_init(const uint8_t pins[], uint8_t num_pins, uint pulse_freq_hz) {

    float div;
    uint32_t wrap;

    if(pulse_freq_hz < 1907) {      // For lower frequencies, we can use the maximum wrap value of 65535 for best resolution, and adjust the divider to achieve the target frequency
        wrap = 65535;
        div = (clock_get_hz(clk_sys) / (pulse_freq_hz * ((float)wrap + 1.f)));
    }
    else {
        // For higher frequencies, we need to reduce the wrap value as clock divider is already down to 1
        div = 1; // pwm slice being clocked directly from system clock with no divider, so wrap must be reduced to achieve target frequency
        wrap = (clock_get_hz(clk_sys) / pulse_freq_hz) - 1;
    }   

    for (uint i = 0; i < num_pins; i++) {
        uint gpio = pins[i];

        // Set GPIO to PWM function
        gpio_set_function(gpio, GPIO_FUNC_PWM);

        // Configure slice
        uint slice_num = pwm_gpio_to_slice_num(gpio);
        servo_wrap[slice_num] = wrap; // store wrap value for this slice for later use in pulse width calculations
        pwm_set_enabled(slice_num, false);      // Disable while configuring
        pwm_set_clkdiv(slice_num, div);       // Set clock divider for this slice
        pwm_set_wrap(slice_num, wrap);       // Use max wrap for best resolution; actual frequency set by clkdiv
        pwm_set_enabled(slice_num, true);

        // Set PWM output to 0 (pulse width 0) initially, until we get valid iBus data
        pwm_set_gpio_level(gpio, 0);
    }
}

// Set pulse width in microseconds
void servo_set_pulse_us(uint gpio, uint32_t pulse_us) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    uint64_t level = ((uint64_t)pulse_us * (65535 + 1)) / (1000000ULL / SERVO_FREQ_HZ); // convert pulse_us to PWM level based on frequency and wrap

    if (level > 65535) level = 65535;

    pwm_set_gpio_level(gpio, (uint16_t)level);
}

// Set angle (0–180°)
void servo_set_angle(uint gpio, float angle_deg) {
    if (angle_deg < 0)   angle_deg = 0;
    if (angle_deg > 180) angle_deg = 180;

    uint32_t pulse_us = SERVO_MIN_US +
                        (uint32_t)((SERVO_MAX_US - SERVO_MIN_US) *
                                   (angle_deg / 180.0f));

    servo_set_pulse_us(gpio, pulse_us);
}

// Center position (1500 µs)
void servo_center(uint gpio) {
    servo_set_pulse_us(gpio, 1500);
}
