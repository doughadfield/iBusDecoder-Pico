#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "blink.pio.h"
#include "rgbled.h"

volatile uint32_t led_state;                                // current value last sent to WS2812 LED

void ws2812_pio_init(PIO pio, uint sm, uint pin)            // Set up PIO SM for ws2812 LED module
{
    uint offset = pio_add_program(pio, &ws2812_program);    // PIO program position in SM memory
    pio_gpio_init(pio, pin);                                // Set up GPIO pin for PIO...
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);  // ...output

    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, 24);           // set shift direction RIGHT, no autopull
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);          // use 8 deep TX FIFO

#define CYCLES_PER_BIT ((ws2812_T1)+(ws2812_T2)+(ws2812_T3)) // constants defined in .pio source file
    float div = clock_get_hz(clk_sys) / (800000 * CYCLES_PER_BIT);  // ws2812 needs precise 800KHz timing
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);                      // Set ws2812 PIO state machine running
    led_state = BLUE;                                     // initially show yellow to indicate initialisation
    pio_sm_put(pio, sm, led_state);                         // Clear all LED colours to off
}