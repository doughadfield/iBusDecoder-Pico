#include "pca9685.h"


// Registers
#define MODE1 0x00
#define PRESCALE 0xFE
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09
#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD



// Function to write a byte to a register
void pca9685_write(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, PCA9685_ADDR, data, 2, false);
}

// Function to read a byte from a register
uint8_t pca9685_read(uint8_t reg) {
    uint8_t value;
    i2c_write_blocking(I2C_PORT, PCA9685_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, PCA9685_ADDR, &value, 1, false);
    return value;
}

// Initialize PCA9685 chip
void pca9685_init() {
    uint8_t prescale;           // value to set in prescale reg

    pca9685_write(MODE1, 0x00); // Reset chip to power-on state
    sleep_ms(10);               // wait for chip to initialise

    // Set to sleep to change prescale
    uint8_t mode1 = pca9685_read(MODE1);
    mode1 = (mode1 & 0x7F) | 0x10;  // Sleep
    pca9685_write(MODE1, mode1);

    if(SERVO_FREQ_HZ == 250)    // set prescale exactly for this case
        prescale = 25;          // exact prescale value for 250Hz as measured
    else
        // Formula: prescale = round(chip clock / (4096 * freq)) - 1
        prescale = (uint8_t)((25000000.0f / (4096.0f * SERVO_FREQ_HZ)) - 1.0f + 0.5f);

    pca9685_write(PRESCALE, prescale);

    printf("PCA9685 target %d Hz → prescale = %d (actual ~%.1f Hz)\n",
           SERVO_FREQ_HZ, prescale,
           25000000.0f / (4096.0f * (prescale + 1)));


    // Wake up from sleep mode
    mode1 = pca9685_read(MODE1);
    mode1 &= ~0x10;  // Clear sleep
    pca9685_write(MODE1, mode1);
    sleep_ms(5);        // give chip time to wake up from sleep

    // Enable auto-increment and restart
    mode1 |= 0xA1;  // AI + RESTART
    pca9685_write(MODE1, mode1);
}

// Set raw PWM value for a channel (channel 0-15, width 0-4095)
// pwm: set on count and off count for pulse
void pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t reg = LED0_ON_L + (4 * channel);
    pca9685_write(reg, on & 0xFF);
    pca9685_write(reg + 1, (on >> 8) & 0xFF);
    pca9685_write(reg + 2, off & 0xFF);
    pca9685_write(reg + 3, (off >> 8) & 0xFF);
}

// Set all channels off with a single register set write
void pca9685_set_all_off() {
    pca9685_write(ALLLED_ON_L, 0);
    pca9685_write(ALLLED_ON_H, 0);
    pca9685_write(ALLLED_OFF_L, 0);
    pca9685_write(ALLLED_OFF_H, 0x10);  // Full off
}

// init entry point - initialises I2C bus and PCA chip
void initialise_pca9685() {
    // I2C Initialisation
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);


    sleep_ms(100);  // Wait for I2C to stabilize

    pca9685_init(); // Now use I2C bus to initialise PCA chip

}

// Set servo angle (0–180°) on a channel
void pca9685_set_servo_angle(uint8_t channel, float angle_deg) {
    if (angle_deg < 0.0f)   angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;

    // Linear interpolation from min to max pulse
    uint16_t pulse_us = (uint16_t)(SERVO_MIN_PULSE_US +
                          (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * (angle_deg / 180.0f));

    // Convert µs to 12-bit PWM count (4096 steps per 20 ms period)
    uint16_t pwm_val = (uint16_t)((pulse_us * 4096UL) / (1000000UL / SERVO_FREQ_HZ));

    // Set on=0 (immediate start), off=pwm_val
    pca9685_set_pwm(channel, 0, pwm_val);
}

// Set servo position in us (500-2500µs) on a channel
void pca9685_set_servo_position(uint8_t channel, uint16_t pulse_us) {
    if (pulse_us < SERVO_MIN_PULSE_US) pulse_us = SERVO_MIN_PULSE_US;
    if (pulse_us > SERVO_MAX_PULSE_US) pulse_us = SERVO_MAX_PULSE_US;

    if(SERVO_FREQ_HZ == 250)        // exactly 1uS per count
        pca9685_set_pwm(channel, 0, pulse_us);
    else
    {
        // Convert µs to 12-bit PWM count (4096 steps per pulse period)
        uint16_t pwm_val = (uint16_t)((pulse_us * 4096UL) / (1000000UL / SERVO_FREQ_HZ));
        pca9685_set_pwm(channel, 0, pwm_val);
    }
}
