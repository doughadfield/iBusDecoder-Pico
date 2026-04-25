/**
 * @file crsf.cpp
 * @brief Crossfire (CRSF) protocol parsing library implementation.
 *
 * This file implements the core logic for parsing CRSF frames,
 * specifically focusing on RC channel data.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "crsf.h"
#include <stdbool.h> // Added for bool type

#define TIMEOUT_US 100 // 100µs is plenty safe for 420k baud (byte = 24µs)


// Global array holding the latest valid channel values (1000–2000 μs typical)
volatile uint16_t RC_Channels[CRSF_NUM_CHANNELS] = {[0 ... CRSF_NUM_CHANNELS-1] = 1500};  // initialise all channels to 1500 (neutral)

// Flag to indicate new valid data is available (optional - for main core polling)
volatile uint8_t RC_new_data_flag = -1;


uint8_t crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc <<= 1;
        }
    }
    return crc;
}


void _crsf_decode_channels(const uint8_t *p)
{
    static uint16_t ch[CRSF_NUM_CHANNELS];

    ch[0]  = (((uint16_t)p[3] >> 0) | ((uint16_t)p[4] << 8)) & 0x07FF;
    ch[1]  = (((uint16_t)p[4] >> 3) | ((uint16_t)p[5] << 5)) & 0x07FF;
    ch[2]  = ((((uint16_t)p[5] >> 6) | ((uint16_t)p[6] << 2)) | ((uint16_t)p[7] << 10)) & 0x07FF;
    ch[3]  = (((uint16_t)p[7] >> 1) | ((uint16_t)p[8] << 7)) & 0x07FF;
    ch[4]  = (((uint16_t)p[8] >> 4) | ((uint16_t)p[9] << 4)) & 0x07FF;
    ch[5]  = ((((uint16_t)p[9] >> 7) | ((uint16_t)p[10] << 1)) | ((uint16_t)p[11] << 9)) & 0x07FF;
    ch[6]  = (((uint16_t)p[11] >> 2) | ((uint16_t)p[12] << 6)) & 0x07FF;
    ch[7]  = (((uint16_t)p[12] >> 5) | ((uint16_t)p[13] << 3)) & 0x07FF;

    ch[8]  = (((uint16_t)p[14] >> 0) | ((uint16_t)p[15] << 8)) & 0x07FF;
    ch[9]  = (((uint16_t)p[15] >> 3) | ((uint16_t)p[16] << 5)) & 0x07FF;
    ch[10] = ((((uint16_t)p[16] >> 6) | ((uint16_t)p[17] << 2)) | ((uint16_t)p[18] << 10)) & 0x07FF;
    ch[11] = (((uint16_t)p[18] >> 1) | ((uint16_t)p[19] << 7)) & 0x07FF;
    ch[12] = (((uint16_t)p[19] >> 4) | ((uint16_t)p[20] << 4)) & 0x07FF;
    ch[13] = ((((uint16_t)p[20] >> 7) | ((uint16_t)p[21] << 1)) | ((uint16_t)p[22] << 9)) & 0x07FF;
    ch[14] = (((uint16_t)p[22] >> 2) | ((uint16_t)p[23] << 6)) & 0x07FF;
    ch[15] = (((uint16_t)p[23] >> 5) | ((uint16_t)p[24] << 3)) & 0x07FF;

    // convert 0-2047 range to 1000-2000 typical RC channel range (assuming 11-bit resolution in CRSF)
    for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; i++)
    {        RC_Channels[i] = (uint16_t)(((uint32_t)ch[i] * 1000) / 2047 + 1000);  // Scale to 1000-2000 range
    }  
    RC_new_data_flag = 0; // Set flag to indicate new data is available for main core
}

void crsf_init(void){
    // Set up our UART
    uart_init(CRSF_UART_RX, CRSF_BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    // gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(CRSF_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(CRSF_UART_RX, true);
}

// CRSF packet structure for 16 channels
// Frame: [Sync][Len][Type][Payload...][CRC]

void crsf_decode_loop(void) {
    static uint8_t buffer[26];
    static uint8_t idx = 0; 
    uint64_t last_byte_time = 0;

    while (true) {
        if (uart_is_readable(CRSF_UART_RX)) {
            uint8_t byte = uart_getc(CRSF_UART_RX);

            last_byte_time = time_us_64();

            // Sync byte for CRSF is 0xC8
            if (idx == 0 && byte != 0xC8) continue;     // Wait for sync byte at the start of a packet
            if (idx == 1 && byte != 24) { // Length of channel data should be 22 bytes + 1 type + 1 CRC = 24 total, so length byte should be <= 25
                idx = 0; // Invalid length, reset to wait for next sync
                continue;
                }
            if(idx == 2 && byte != 0x16) { // We expect type 0x16 for RC Channels Packed
                idx = 0; // Invalid type, reset to wait for next sync
                continue;
            }
        
            buffer[idx++] = byte;   // We're in a RC data packet, store it in buffer and move index
        
            if (idx >= 26) { // Packet complete
                uint8_t calculated_crc = crc8(&buffer[2], idx - 3);
                if (calculated_crc != buffer[idx - 1]) {
                    printf("\nCRC Mismatch! Calculated: 0x%02X, Received: 0x%02X\n", calculated_crc, buffer[idx - 1]);
                    idx = 0; // Reset for next packet
                    continue;
                }
                _crsf_decode_channels(buffer);
                idx = 0;
#ifdef NEVER
                for (int i = 0; i < 8; i++) {
                    printf("CH%2d: %4d ", i + 1, channel[i]);
                }
                printf("\n");
                for (int i = 8; i < 16; i++) {
                    printf("CH%2d: %4d ", i + 1, channel[i]);
                }
                printf("\n");
                sleep_ms(500); // Throttle output for readability
#endif // NEVER
            }
        }
        else {  //  no char in fifo buffer
            if ((time_us_64() - last_byte_time) > TIMEOUT_US) {
                // We are in the inter-frame gap. 
                idx = 0;
            }
        }
    }
}
