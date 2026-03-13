
// PIO addressing on-board RGB LED (GPIO 16) for on-board indications
#define WS2812_PIN 16           // chinese pico boards have ws2812 on pin 16
#define WS2812_PIO pio0         // pio unit for ws2812 RGB LED
#define WS2812_SM 2             // state machine for LED output
#define GREEN 0x0F000000        // send this to ws2812 for GREEN LED
#define RED   0x00180000        // send this to ws2812 for RED LED
#define BLUE  0x00001F00        // send this to ws2812 for BLUE LED
#define WHITE 0x0F0F0F00        // send this to ws2812 for WHITE LED
#define CYAN (RED|BLUE)         // above primary colour intensities are tuned for
#define MAGENTA (GREEN|BLUE)    // best colour mix and even-ness
#define YELLOW (GREEN|RED)
#define BLACK 0                 // turn off all LEDs in module

#define put_pixel(pixel) pio_sm_put(WS2812_PIO, WS2812_SM, (pixel))

extern volatile uint32_t led_state;                                // current value last sent to WS2812 LED
extern void ws2812_pio_init(PIO pio, uint sm, uint pin);           // Set up PIO SM for ws2812 LED module
