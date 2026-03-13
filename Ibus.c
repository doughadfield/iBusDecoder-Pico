
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "Ibus.h"

#define IBUS_BAUDRATE       115200

// iBus packet is always 32 bytes
#define IBUS_PACKET_LENGTH  32

// Global array holding the latest valid channel values (1000–2000 μs typical)
volatile uint16_t ibus_channels[14] = {1500};

// Flag to indicate new valid data is available (optional - for main core polling)
volatile bool ibus_new_data_flag = false;

// ─────────────────────────────────────────────
// iBus packet validation & parsing
// ─────────────────────────────────────────────

static bool ibus_parse_packet(const uint8_t *buf, uint16_t *channels_out) {
    // Byte 0–1 must be header: 0x20 0x40
    if (buf[0] != 0x20 || buf[1] != 0x40) {
        return false;
    }

    // Calculate checksum (sum of bytes 0..29 should == bytes 30..31 little-endian)
    uint16_t sum = 0;
    for (int i = 0; i < 30; i++) {
        sum += buf[i];
    }

    uint16_t checksum = (buf[31] << 8) | buf[30];
    if (sum + checksum != 0xFFFF) {
        printf("Checksum failed: %u != %u\n", sum, checksum);
    }

    // Extract 14 channels (each 16-bit little-endian)
    for (int i = 0; i < 14; i++) {
//        channels_out[i] = (buf[3 + i*2 + 1] << 8) | buf[3 + i*2];
        int offset = 2 + i * 2;
        channels_out[i] = (buf[offset + 1] << 8) | buf[offset];
    }

    return true;
}

// ─────────────────────────────────────────────
// Core 1 – iBus receiver loop (no interrupts)
// ─────────────────────────────────────────────

void core1_entry(void) {
    uint8_t rx_buffer[IBUS_PACKET_LENGTH];
    uint rx_idx = 0;
    bool in_packet = false;

    while (true) {
        // Wait for at least one byte
        if (uart_is_readable(iBus_UART)) {
            uint8_t byte = uart_getc(iBus_UART);
            if (byte == 0x20) {
                // Possible start of new packet
                rx_buffer[0] = byte;
                rx_idx = 1;
                in_packet = true;
                continue;
            }

            if (!in_packet) {
                continue;
            }

            rx_buffer[rx_idx++] = byte;

            // We have a full packet?
            if (rx_idx >= IBUS_PACKET_LENGTH) {
                uint16_t temp_channels[14];
                if (ibus_parse_packet(rx_buffer, temp_channels)) {
                    // Copy to global array (volatile for safe access from core 0)
                    for (int i = 0; i < 14; i++) {
                        ibus_channels[i] = temp_channels[i];
                    }
                    ibus_new_data_flag = true;
                }
                // Reset for next packet
                rx_idx = 0;
                in_packet = false;
            }
        }
    }
}

// ─────────────────────────────────────────────
// Initialization and startup of iBus receiver process
// ─────────────────────────────────────────────

void Ibus_Init() {
    // Initialize UART0 for iBus (115200 8N1)
    uart_init(iBus_UART, iBus_BAUDRATE);
    gpio_set_function(iBus_RX_PIN, GPIO_FUNC_UART);

    // Optional: enable UART FIFO if you want slightly less CPU usage
    uart_set_fifo_enabled(iBus_UART, true);

    // Initialize UART1 for telemetry (TX and RX for queries)
    uart_init(Telemetry_UART, iBus_BAUDRATE);
    gpio_set_function(Telemetry_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(Telemetry_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(Telemetry_UART, true);

    ibus_channels[2] = 1000;  // Set throttle channel to minimum for safety on startup

    // Start iBus receiver on Core 1
    multicore_launch_core1(core1_entry);

    printf("Core 1 running iBus receiver - polling channels...\n");
}

// ─────────────────────────────────────────────
// Telemetry (Sensor) Functions
// ─────────────────────────────────────────────

// Sensor address (arbitrary, start with 0x01 for first sensor)
#define SENSOR_ADDR         0x07

// Sensor type for external voltage (0x02)
#define SENSOR_TYPE_VOLTAGE 0x02

// Send voltage telemetry response (e.g., 12.6 V → 1260 in 0.01V units, little-endian)
static void send_telemetry_voltage(float voltage) {
    uint16_t voltage_int = (uint16_t)(voltage * 100);  // e.g., 12.6 → 1260

    // Response packet structure (length 6 for voltage)
    uint8_t packet[8];  // 6 data + 2 checksum
    packet[0] = 0x06;  // Length (including length byte)
    packet[1] = 0x90 | SENSOR_ADDR;  // Command: response | addr
    packet[2] = SENSOR_TYPE_VOLTAGE; // Type
    packet[3] = 0x02;  // Data length (2 bytes for voltage)
    packet[4] = voltage_int & 0xFF;  // Low byte
    packet[5] = (voltage_int >> 8) & 0xFF;  // High byte

    // Checksum: 0xFFFF - sum of first 6 bytes
    uint16_t sum = 0;
    for (int i = 0; i < 6; i++) {
        sum += packet[i];
    }
    uint16_t checksum = 0xFFFF - sum;
    packet[6] = checksum & 0xFF;  // Low
    packet[7] = (checksum >> 8) & 0xFF;  // High

    // Send on telemetry UART
    uart_write_blocking(Telemetry_UART, packet, 8);
    printf("Sent telemetry voltage response: %.2f V\n", voltage);
}


// Telemetry loop: Listen for queries and respond (half-duplex style)
// Assumes diode/resistor hack on wiring to allow RX/TX on same line if needed
void telemetry_send(void) {
    if (uart_is_readable(Telemetry_UART)) {
        uint8_t byte = uart_getc(Telemetry_UART);

        printf("Received telemetry query byte: 0x%02X\n", byte);
        // iBus telemetry query is 2 bytes: 0x04, (0x80 | addr), checksum (0xFFFF - sum)
        // For addr 1: 0x04 0x81 checksum=0xFF7A (but we check simple pattern)
        if (byte == 0x04) {
            // Read next byte (addr byte)
            if (uart_is_readable_within_us(Telemetry_UART, 1000)) {
                uint8_t addr_byte = uart_getc(Telemetry_UART);

                if (addr_byte == (0x80 | SENSOR_ADDR)) {
                    // Read checksum (2 bytes)
                    uint8_t csum_l = uart_getc(Telemetry_UART);
                    uint8_t csum_h = uart_getc(Telemetry_UART);

                    uint16_t rx_csum = (csum_h << 8) | csum_l;
                    uint16_t calc_sum = 0x04 + addr_byte;
                    if (0xFFFF - calc_sum == rx_csum) {
                        // Valid query for our sensor – respond with voltage
                        // Replace 12.6f with your actual ADC-read voltage
                        send_telemetry_voltage(12.6f);
                    }
                }
            }
        }
    }
}
