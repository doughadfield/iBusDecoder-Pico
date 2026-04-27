
/*
 * RC_Decoder.c
 *
 * Receives channel data from an RC receiver and uses it to control servos, motors and switches
 * via both the internal PWM hardware and an external PCA9685 board.
 * Also includes examples of using the PIO for blinking an LED and using timers.
 *
 * Supports both iBus and CRSF receivers (selectable via #define at top of file).
 *
 * Pinouts for RP2040:
 * 0-3: PWM outputs to servos (GPIO 0,1 on slice 0; 2,3 on slice 1)
 * 4,5: UART1 (TX,RX) for telemetry output to receiver (optional - PWM slice 2 if not using telemetry)
 * 6-15: PWM outputs to servos (GPIO 6,7 on slice 3; 8,9 on slice 4; 10,11 on slice 5; 12,13 on slice 6; 14,15 on slice 7)
 * 16: onboard RGB LED (dedicated - no external pin)
 * 17-25: Optional additional PWM outputs to servos (from solder pads on bottom of board)
 *      17 on pwm slice 0(pinB); 18,19 on slice 1; 20,21 on slice 2; 22,23 on * slice 3; 24,25 on slice 4)
 * 26,27: Analog inputs or I2C1 for PCA9685 board if no additional analog inputs needed)
 * 28: Analog input for telemetry (if TELEMETRY is defined - for reading battery voltage )
 * 29: UART0 (RX) for iBus/CRSF input (only needs one RX pin - TX not used for iBus)
 * Optionally UART1 (GPIO 4,5) can be used for telemetry output to receiver if supported by receiver and desired
 *
 * specific whole slices can be set for brushed motor ESC control if desired
 *               faster update rate, full pwm range (0-100% duty cycle)
 *               Motors are bi-directional, so use A and B outputs on same slice for each motor,
 *               with A as forward and B as reverse)
 * version 0.9
 */

#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include "InternalPWM.h"
#include "blink.pio.h"
#include "rgbled.h"

// #define DEBUG  // Enable debug print statements (comment out to disable)
#ifndef DEBUG
    #define printf(fmt, ...) ((void)0)
#endif

#define VBAT 28  // Analog input for battery voltage monitoring

#define CRSF     // Receiver is CRSF protocol
// #define IBUS    // Receiver is iBus protocol (comment out if using CRSF, as code is currently set up to use either CRSF or iBus, but not both at
// the same time) #define I2C_ENABLE   // Uncomment to enable I2C for controlling PCA9685 board (comment out if using analog inputs on GPIO 26 and 27
// instead)
#define RGBLED  // Enable use of on-board RGB LED for status indication (uses PIO program to control LED blinking)

#ifdef I2C_ENABLE
// I2C defines for PCA9685 board
#include "pca9685.h"
#define I2C_ID i2c1
#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27
#define PCA9685_ADDR 0x40  // Default I2C address for PCA9685 board
#else
#define ANALOG_1 26
#define ANALOG_2 27
#endif  // I2C_ENABLE

#ifdef CRSF
#include "crsf.h"
#undef IBUS
#define NUM_CHANNELS CRSF_NUM_CHANNELS  // CRSF supports up to 16 channels
#elif defined(IBUS)
#include "Ibus.h"
#undef CRSF
#define NUM_CHANNELS IBUS_NUM_CHANNELS  // iBus supports up to 14 channels
#else
#error "Please define either CRSF or IBUS to specify the receiver protocol being used"
#endif

// Flag to indicate new valid data is available
volatile uint8_t RC_new_data_flag = -1;  // -1 = no data yet, 0 = data read, >0 = we've missed an iBus packet

// enum to define channel types
enum ChannelType
{
    REV_MOTOR,  // bidirectional (REVersable) motor (ESC) using two pins per motor for forward/reverse control
    UNI_MOTOR,  // unidirectional motor (ESC) using one pin per motor (only forward control)
    SERVO,
    SWITCH
};

// struct to hold channel configuration - type and pin assignment
struct channel
{
    uint8_t Chan_No;        // RC channel number  Note: channel 0 corresponds to RC Channel 1 on the transmitter, channel 1 to RC Channel 2, etc.
    enum ChannelType type;  // Type of output (REV_MOTOR, UNI_MOTOR, SERVO, SWITCH)
    uint8_t pin[2];         // GPIO pin(s) for this channel (1 pin for SERVO, SWITCH, UNI_MOTOR; 2 pins for REV_MOTOR for forward/reverse control)
    uint8_t num_pins;       // Number of pins used (1 for SERVO, SWITCH, UNI_MOTOR; 2 for REV_MOTOR)
} channels[] = {
    // Supports 14 channels with IBUS, 16 channels with CRSF (ELRS)
    {0, REV_MOTOR, {0, 1}, 2},  // Ch 1:  Tank left track
    {1, REV_MOTOR, {2, 3}, 2},  // Ch 2:  Tank right track
    {2, SERVO, {8}, 1},         // Ch 3:  Gun elevation
    {3, REV_MOTOR, {6, 7}, 2},  // Ch 4:  Tank turret rotation
    {4, SWITCH, {9}, 1},        // Ch 5:  Motor arm
    {5, SWITCH, {10}, 1},       // Ch 6:  Gun fire
    {6, SWITCH, {11}, 1},       // Ch 7:  LED Headlights
    {7, SERVO, {12}, 1},        // Ch 8:  Spare
    {8, SERVO, {13}, 1},        // Ch 9:  Spare
    {9, SERVO, {14}, 1},        // Ch 10: Spare
    {10, SERVO, {15}, 1},       // Ch 11: Spare
    {11, SWITCH, {17}, 1},      // Ch 12: These pins are on solder pads on the bottom of the board
    {12, SWITCH, {18}, 1},      // Ch 13: These pins are on solder pads on the bottom of the board
    {13, SWITCH, {19}, 1},      // Ch 14: These pins are on solder pads on the bottom of the board
    {14, SWITCH, {20}, 1},      // Ch 15: These pins are on solder pads on the bottom of the board
    {15, SWITCH, {21}, 1}       // Ch 16: These pins are on solder pads on the bottom of the board
};

/*
 * Drive motor (ESC) with given pulse width (1000-2000 µs)
 * bidirectionally using two pins per motor (pin[0] for forward, pin[1] for reverse)
 * using both pins of a single PWM slice
 * Added pulse stretch to enable full stick control range for ESCs.
 */

#define DEADBAND 10     // noise value around zero or midband from transmitter
#define MIN_MOTOR 6600  // value at which motor just starts moving

void motor_drive(struct channel channel, uint16_t pulse_width)
{
    if (RC_Channels[ARM_CHANNEL - 1] < 1400)  // arm switch is OFF (array index one less than channel number)
    {
        // Motor is disarmed, set outputs to 0 (off)
        pwm_set_gpio_level(channel.pin[0], 0);      // Forward pin off
        if (channel.type == REV_MOTOR)
            pwm_set_gpio_level(channel.pin[1], 0);  // Reverse pin off
        return;
    }
    // Constrain input to valid servo range
    if (pulse_width < 1000) pulse_width = 1000;
    if (pulse_width > 2000) pulse_width = 2000;

    uint32_t throttle = pulse_width - 1000;  // for uni-directional motor, throttle is directly proportional to pulse width above 1000µs (1000µs = off, 2000µs = full forward)
    if (channel.type == REV_MOTOR)
    {
        if (pulse_width > 1500)               // forward direction
            throttle = (pulse_width * 2) - 3000;  // for bi-directional motor, stretch 0-500µs range to 0-1000µs (centre is 1500)
        else
            throttle = 1000 - ((pulse_width * 2) - 2000);       // reverse, so subtract the pulse value from 1000 to get a positive throttle value for reverse direction
    }
    throttle = throttle < DEADBAND ? 0 : throttle;                           // zero throttle within deadband

#define SLICENUM (pwm_gpio_to_slice_num(channel.pin[0]))                     // determine PWM hardware slice to get servo_wrap value
    uint16_t level = 0;                                                      // level is 0 if within deadband
    if (throttle > 0)
        level = MIN_MOTOR + (throttle * (servo_wrap[SLICENUM] + 1 - MIN_MOTOR)) / 1000;  // scale throttle to PWM level based on servo_wrap for the slice

    if (channel.type == UNI_MOTOR || pulse_width > 1500)  // both uni-directional motors and bi-directional motors with forward throttle use forward pin
    {
        // === FORWARD ===
        pwm_set_gpio_level(channel.pin[0], level);  // Forward pin
        pwm_set_gpio_level(channel.pin[1], 0);      // Reverse pin off
        printf("FWD:%5u ", level);
    }
    else
    {
        // === REVERSE ===
        pwm_set_gpio_level(channel.pin[1], level);  // Reverse pin
        pwm_set_gpio_level(channel.pin[0], 0);      // Forward pin off
        printf("REV:%5u ", level);
    }
}

/*
 * initialise channel outputs
 */
void init_channels(void)
{
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)  // loop through all channels in config and initialise hardware based on type
    {
        switch (channels[i].type)
        {
        case REV_MOTOR:
            hwpwm_init(channels[i].pin, channels[i].num_pins, MOTOR_FREQ_HZ);
            pwm_set_gpio_level(channels[i].pin[0], 0);  // Forward pin off
            pwm_set_gpio_level(channels[i].pin[1], 0);  // Reverse pin off
            RC_Channels[i] = 1500;                      // Set (bi-directional) motor channel to centre (off) on startup
            break;

        case UNI_MOTOR:
            hwpwm_init(channels[i].pin, channels[i].num_pins, MOTOR_FREQ_HZ);
            pwm_set_gpio_level(channels[i].pin[0], 0);  // Forward pin off
            RC_Channels[i] = 1000;                      // Set (uni-directional) motor channel to minimum (off) on startup
            break;

        case SERVO:
            hwpwm_init(channels[i].pin, channels[i].num_pins,
                       SERVO_FREQ_HZ);                  // initialise PWM pin for this servo channel at servo frequency for servo control
            pwm_set_gpio_level(channels[i].pin[0], 0);  // servo output off (0 pulse width) on startup
            RC_Channels[i] = 1500;                      // Set servo channel to centre position on startup
            break;

        case SWITCH:
            gpio_init(channels[i].pin[0]);               // Set switch pin as gpio
            gpio_set_dir(channels[i].pin[0], GPIO_OUT);  // Set switch pin as output
            gpio_put(channels[i].pin[0], 0);             // Set switch pin to 0 (off)
            RC_Channels[i] = 1000;                       // Set switch channel to minimum (off) on startup
            break;
        }
    }
}

/*
 * Initialise hardware, set up iBus/CRSF read loop on core 1,
 * then in the main loop check for new RC data and update servo positions accordingly,
 * as well as checking for telemetry queries and responding if needed.
 *
 */
int main()
{
    stdio_init_all();
    init_channels();                                                 // initialise all channel outputs
#ifdef DEBUG
    sleep_ms(1000);                                                  // Wait for usb serial to settle after reset

    printf("START of PROGRAM\n");                                    // DEBUG indication that program has started
    if (watchdog_caused_reboot())
    {
        printf("! - System recovered from a Watchdog Reset - !\n");  // DEBUG
    }
#endif // DEBUG

#ifdef RGBLED
    ws2812_pio_init(WS2812_PIO, WS2812_SM, WS2812_PIN);  // ws2812 output pio state machine
#endif                                                   // RGBLED

#ifdef CRSF
    crsf_init();                                         // Start CRSF receiver on Core 1
    multicore_launch_core1(crsf_decode_loop);
#elif defined(IBUS)
    Ibus_Init();  // Start iBus receiver on Core 1
    multicore_launch_core1(ibus_decode_loop);
#endif                                                   // CRSF or IBUS

    printf("Core 1 running RC receiver - polling channels...\n");
#ifdef DEBUG
    watchdog_enable(2000, true);                          // Lengthen watchdog timeout to 2 seconds for debugging
#else
    watchdog_enable(200, true);                          // Enable watchdog with 200ms timeout
#endif // DEBUG

    while (true)
    {
        if (RC_new_data_flag == 0)  // Flag gets reset when new RC packet recieved on core 1)
        {
            RC_new_data_flag++;     // Increment flag to indicate we've processed the new RC data
            put_pixel(GREEN);       // flash green on the RGB LED to indicate normal operation (comment out if not using RGB LED)

            for (uint8_t i = 0; i < NUM_CHANNELS; i++)
            {
                printf("CH%d:%4u", i + 1, RC_Channels[i]);

                switch (channels[i].type)
                {
                case UNI_MOTOR:
                case REV_MOTOR:
                    printf("mt ");
                    motor_drive(channels[i], RC_Channels[i]);  // run motor at set speed
                    break;

                case SERVO:
                    printf("sv ");
                    servo_set_pulse_us(channels[i].pin[0], RC_Channels[i]);  // Set internal pwm servo to the value of ibus channel
                    break;

                case SWITCH:
                    printf("sw ");
                    gpio_put(channels[i].pin[0], RC_Channels[i] > 1500 ? 1 : 0);  // Set switch GPIO high if channel value above 1500, otherwise low
                    break;
                }
            }
            printf("\n");
        }
        else
        {
            if (RC_new_data_flag++ > 10)     // increment flag to indicate we've missed an RC packet
            {                                // We've missed more than 10 packets, so we're likely in a failsafe condition
                put_pixel(RED);              // flash red on the RGB LED to indicate FAILSAFE condition
                if (RC_new_data_flag > 250)  // if we've missed more than 250 packets, reset flag to prevent overflow and keep printing message
                {
                    RC_new_data_flag = 250;  // prevent counter overflow
                }
            }
        }
#ifdef TELEMETRY
        crsf_telemetry_send(crsf_packet);  // Send telemetry packet with battery voltage to receiver
#endif                                     // TELEMETRY
        watchdog_update();                 // Reset watchdog timer before delay

#ifdef DEBUG
        sleep_ms(200);                     // Slow down loop for debugging (comment out or reduce delay for normal operation)
#else
        sleep_ms(2);                       // Slow down loop slightly to ensure we don't miss RC packets
#endif // DEBUG
    }
    return 0;
}
