
/*
 * iBusReceiver.c
 * 
 * Receives iBus data from an RC receiver and uses it to control servos via both the internal PWM hardware and an external PCA9685 board. Also includes examples of using the PIO for blinking an LED and using timers.
 * Pinouts for RP2040:
 * 0-3: PWM outputs to servos (GPIO 0,1 on slice 0; 2,3 on slice 1)
 * 4,5: UART1 (TX,RX) for telemetry output to receiver (optional - PWM slice 2 if not using telemetry)
 * 6-15: PWM outputs to servos (GPIO 6,7 on slice 3; 8,9 on slice 4;
 *              10,11 on slice 5; 12,13 on slice 6; 14,15 on slice 7)
 * 16: onboard RGB LED (dedicated - no external pin)
 * 17-25: Optional additional PWM outputs to servos (from solder pads on bottom of board)
 *              17 on slice 0(pinB); 18,19 on slice 1; 20,21 on slice 2; 22,23 on slice 3; 24,25 on slice 4)
 * 26,27: Analog inputs or I2C1 for PCA9685 board if no additional analog inputs needed)
 * 28: Analog input for telemetry (optional - for reading battery voltage )
 * 29: UART0 (RX) for iBus input (only needs one RX pin - TX not used for iBus)
 * 
 * specific whole slices can be set for brushed motor ESC control if desired
 *               faster update rate, full pwm range (0-100% duty cycle)
 *               Motors are bi-directional, so use A and B outputs on same slice for each motor,
 *               with A as forward and B as reverse)
 * version 0.9
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

#include "pca9685.h"
#include "InternalPWM.h"
#include "Ibus.h"
#include "blink.pio.h"


#define VBAT 28 // Analog input for battery voltage monitoring

// #define I2C_ENABLE

#ifdef I2C_ENABLE
// I2C defines for PCA9685 board
#define I2C_ID i2c1
#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27
#define PCA9685_ADDR 0x40   // Default I2C address for PCA9685 board
#else
#define ANALOG_1 26
#define ANALOG_2 27
#endif // I2C_ENABLE 

// enum to define channel types
enum ChannelType {
    MOTOR,
    SERVO,
    SWITCH
};

// struct to hold channel configuration - type and pin assignment
struct channel {
    uint8_t Chan_No;    // iBus channel number (0-13)   
    enum ChannelType type;   // Type of output (MOTOR, SERVO, SWITCH)
    uint8_t pin[2];     // GPIO pin(s) for this channel (1 pin for SERVO, 2 pins for MOTOR for forward/reverse control)
    uint8_t num_pins;     // Number of pins used (1 for SERVO, 2 for MOTOR)
} channels[] = {
    {0, MOTOR, {0,1}, 2},   // Tank left track
    {1, MOTOR, {2,3}, 2},   // Tank right track
    {2, MOTOR, {6,7}, 2},   // Tank turret rotation
    {3, SERVO, {8}, 1},     // Gun elevation
    {4, SWITCH, {9}, 1},    // Motor arm
    {5, SWITCH, {10}, 1},   // Gun fire
    {6, SWITCH, {11}, 1},   // Spare
    {7, SERVO, {12}, 1},    // Spare
    {8, SERVO, {13}, 1},    // Spare
    {9, SERVO, {14}, 1},    // Spare
    {10, SERVO, {15}, 1},   // Spare
    {11, SWITCH, {17}, 1},
    {12, SWITCH, {18}, 1},
    {13, SWITCH, {19}, 1}
};

#ifdef RGBLED
// PIO addressing on-board RGB LED (GPIO 16) for on-board indications
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}
#endif // RGBLED

// future - use alarm callback for failsafe timeout if no iBus data received for a certain time period
//      eg set all servos to a safe position in the callback function
int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}

/*
* Drive motor (ESC) with given pulse width (1000-2000 µs)
* bidirectionally using two pins per motor (pin[0] for forward, pin[1] for reverse)
* using both pins of a single PWM slice
*/

#define DEADBAND 30 // deadband in microseconds for motor control around neutral (1500 µs) to prevent jitter

void motor_drive(struct channel channel, uint16_t pulse_width) {
    // For motors (ESCs), we assume pulse_width is already in the correct range (e.g., 1000-2000 µs) and just convert to PWM level
    if (pulse_width < 1000) pulse_width = 1000; // constrain to minimum
    if (pulse_width > 2000) pulse_width = 2000; // constrain to maximum

    uint slice_num = pwm_gpio_to_slice_num(channel.pin[0]); // both forward and reverse pins are on the same slice, so just use pin[0] to get slice number

    if(pulse_width > 1500 + DEADBAND) {
        // forward
        uint16_t level = ((uint64_t)(pulse_width - 1500) * (servo_wrap[slice_num] + 1)) / 500 ; // convert pulse_width to PWM level based on frequency and wrap
        pwm_set_gpio_level(channel.pin[0], level); // output full pulse width range for ESC control on forward pin
        pwm_set_gpio_level(channel.pin[1], 0); // set reverse pin to 0
    }
    else if(pulse_width < 1500 - DEADBAND) {
        // reverse
        uint16_t level = ((uint64_t)(1500 - pulse_width) * (servo_wrap[slice_num] + 1)) / 500 ; // convert pulse_width to PWM
        pwm_set_gpio_level(channel.pin[1], level); // output full pulse width range for ESC control on reverse pin
        pwm_set_gpio_level(channel.pin[0], 0); // set forward pin to 0
    }
    else {        // within deadband around neutral - set both forward and reverse pins to 0 to prevent jitter
        
        pwm_set_gpio_level(channel.pin[0], 0); // set forward pin to 0
        pwm_set_gpio_level(channel.pin[1], 0); // set reverse pin to 0
    }
}

/*
* Initialise hardware, set up iBus read loop on core 1,
* then in the main loop check for new iBus data and update servo positions accordingly,
* as well as checking for telemetry queries and responding if needed.
*
*/
int main()
{
    stdio_init_all();

    sleep_ms(1000);     // Wait for usb serial to settle after reset

    printf("START of PROGRAM\n");       // DEBUG indication that program has started
    
    for(uint8_t i=0; i<14; i++) {

        if(channels[i].type == MOTOR) {
            hwpwm_init(channels[i].pin, channels[i].num_pins, MOTOR_FREQ_HZ);    // initialise PWM pins for this motor channel at 10kHz for ESC control

        } else if(channels[i].type == SERVO) {
            hwpwm_init(channels[i].pin, channels[i].num_pins, SERVO_FREQ_HZ);    // initialise PWM pin for this servo channel at 250Hz for servo control
        }
    }

    #ifdef NEVER
    // #ifdef PICO_DEFAULT_LED_PIN
    // PIO Blinking example
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    
    blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
    #endif

    // Timer example code - This example fires off the callback after 2000ms
    // add_alarm_in_ms(2000, alarm_callback, NULL, false);
    // For more examples of timer use see https://github.com/raspberrypi/pico-examples/tree/master/timer

    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks

    #ifdef I2C_ENABLE
    // Initialize I2C for PCA9685 board
    initialise_pca9685();   // initialise PWM on pca9685 board
    #endif // I2C_ENABLE

    Ibus_Init();            // Start iBus receiver on Core 1

    while (true) {
        if (ibus_new_data_flag) {
            // Clear flag
            ibus_new_data_flag = false;

#ifdef I2C_ENABLE
            // DEBUG: print channel values to console
            for (uint8_t i = 0; i < 14; i++) {
                pca9685_set_servo_position(i,ibus_channels[i]); // Set servo on each channel to the value of ibus channel
                printf("%4u ", ibus_channels[i]);
            }
            printf("\n");
#endif // I2C_ENABLE

            for (uint8_t i=0; i< IBUS_NUM_CHANNELS; i++) {
                printf("%4u ", ibus_channels[i]);
                if(channels[i].type == MOTOR) {
                    // for motor channels, we want to output the full PWM pulse width range for each motor (pin A for forward, pin B for reverse)
                    // with 1000-2000 input mapped to 0-wrap value of slice
                    printf("M ");
                    motor_drive(channels[i], ibus_channels[i]);
                } else if(channels[i].type == SERVO) {
                    printf("S ");
                    servo_set_pulse_us(channels[i].pin[0], ibus_channels[i]); // Set internal pwm servo to the value of ibus channel
                }
            }   
            printf("\n");

        }
        telemetry_send(); // Check for telemetry queries and respond if needed
        // reduce cpu usage by sleeping for a short time (adjust as needed for responsiveness)
        sleep_ms(5);
    }
    return 0;
}
