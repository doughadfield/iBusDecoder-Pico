
// Hardware UART-based iBus receiver for Raspberry Pi Pico (RP2040)
// Reads iBus packets from a UART, validates them, and makes channel data available to the main application.
// Core 1 is dedicated to receiving and parsing iBus data, while Core 0 can run the main application logic without blocking on UART reads.
// ─────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────
#include "pico/multicore.h"
#include "hardware/uart.h"

#define IBUS_NUM_CHANNELS 14
// UART defines
// UART0 for Rx from receiver, UART1 for Tx to receiver (telemetry)
#define iBus_UART uart0
#define Telemetry_UART uart1
#define iBus_BAUDRATE 115200    // Baud rate the same for both iBus and telemetry
// Use pin 29 for UART0 RX (iBus receiver)
#define iBus_RX_PIN 29
// Use pins 4 and 5 for UART1 (telemetry to receiver)
#define Telemetry_TX_PIN 4
#define Telemetry_RX_PIN 5


// Initialise UART and start iBus receiver on Core 1
extern void Ibus_Init(void);

// Global array holding the latest valid channel values (1000–2000 μs typical)
extern volatile uint16_t ibus_channels[IBUS_NUM_CHANNELS];

// Flag to indicate new valid data is available (optional - for main core polling)
extern volatile uint8_t ibus_new_data_flag;

// send telemetry response if we receive a valid query on the telemetry UART
extern void telemetry_send(void);
