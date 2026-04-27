/*
 * This file implements the core logic for parsing CRSF frames
 * Tested from ELRS Nano receiver running ELRS 2.0.0-rc7 firmware
 * using CRSF channel data in packed format (type 0x16)
 * CRSF received values are converted from 0-2047 range to 1000-2000
 */

#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include "crsf.h"
#ifdef TELEMETRY
#include "adc.h"
#endif  // TELEMETRY

#define TIMEOUT_US 100  // 100µs is plenty safe for 420k baud (byte = 24µs)

// Global array holding the latest valid channel values (1000–2000 μs typical)
volatile uint16_t RC_Channels[CRSF_NUM_CHANNELS] = {[0 ... CRSF_NUM_CHANNELS - 1] = 1500};  // initialise all channels to 1500 (neutral)

uint8_t crc8(const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= ptr[i];  // XOR the next byte into the CRC
        for (uint8_t j = 0; j < 8; j++)
        {
            // Check if the most significant bit is set
            if (crc & 0x80)
            {
                // Shift and XOR with the CRSF polynomial 0xD5
                crc = (crc << 1) ^ 0xD5;
            }
            else
            {
                // Just shift
                crc <<= 1;
            }
        }
    }
    return crc;
}

void _crsf_decode_channels(const uint8_t *p)
{
    static uint16_t ch[CRSF_NUM_CHANNELS];

    ch[0] = (((uint16_t)p[3] >> 0) | ((uint16_t)p[4] << 8)) & 0x07FF;
    ch[1] = (((uint16_t)p[4] >> 3) | ((uint16_t)p[5] << 5)) & 0x07FF;
    ch[2] = ((((uint16_t)p[5] >> 6) | ((uint16_t)p[6] << 2)) | ((uint16_t)p[7] << 10)) & 0x07FF;
    ch[3] = (((uint16_t)p[7] >> 1) | ((uint16_t)p[8] << 7)) & 0x07FF;
    ch[4] = (((uint16_t)p[8] >> 4) | ((uint16_t)p[9] << 4)) & 0x07FF;
    ch[5] = ((((uint16_t)p[9] >> 7) | ((uint16_t)p[10] << 1)) | ((uint16_t)p[11] << 9)) & 0x07FF;
    ch[6] = (((uint16_t)p[11] >> 2) | ((uint16_t)p[12] << 6)) & 0x07FF;
    ch[7] = (((uint16_t)p[12] >> 5) | ((uint16_t)p[13] << 3)) & 0x07FF;

    ch[8] = (((uint16_t)p[14] >> 0) | ((uint16_t)p[15] << 8)) & 0x07FF;
    ch[9] = (((uint16_t)p[15] >> 3) | ((uint16_t)p[16] << 5)) & 0x07FF;
    ch[10] = ((((uint16_t)p[16] >> 6) | ((uint16_t)p[17] << 2)) | ((uint16_t)p[18] << 10)) & 0x07FF;
    ch[11] = (((uint16_t)p[18] >> 1) | ((uint16_t)p[19] << 7)) & 0x07FF;
    ch[12] = (((uint16_t)p[19] >> 4) | ((uint16_t)p[20] << 4)) & 0x07FF;
    ch[13] = ((((uint16_t)p[20] >> 7) | ((uint16_t)p[21] << 1)) | ((uint16_t)p[22] << 9)) & 0x07FF;
    ch[14] = (((uint16_t)p[22] >> 2) | ((uint16_t)p[23] << 6)) & 0x07FF;
    ch[15] = (((uint16_t)p[23] >> 5) | ((uint16_t)p[24] << 3)) & 0x07FF;

    // convert 0-2047 range to 1000-2000 typical RC channel range (assuming 11-bit resolution in CRSF)
    for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; i++)
    {
        RC_Channels[i] = (uint16_t)(((uint32_t)ch[i] * 1000) / 1600 + 880);  // Scale to 1000-2000 range
    }
    RC_new_data_flag = 0;                                                    // Set flag to indicate new data is available for main core
}

void crsf_init(void)
{
    // Set up our RX UART for incoming CRSF data
    uart_init(CRSF_UART_RX, CRSF_BAUD_RATE);
    gpio_set_function(CRSF_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(CRSF_UART_RX, true);
#ifdef TELEMETRY
    init_adc();                                          // Initialize ADC for battery voltage monitoring
    uart_init(CRSF_UART_TX, CRSF_BAUD_RATE);            // Set up our TX UART for outgoing CRSF telemetry (battery voltage)
    gpio_set_function(CRSF_TX_PIN, GPIO_FUNC_UART);         // Set TX pin function to UART
    uart_set_fifo_enabled(CRSF_UART_TX, true);
#endif  // TELEMETRY
}

#ifdef TELEMETRY
uint8_t crsf_packet[CRSF_BATTERY_FRAME_SIZE];

void crsf_telemetry_send(uint8_t *packet_buffer)
{
    uint32_t mv = get_smoothed_mv();
    crsf_battery_packet(crsf_packet, mv);  // Send voltage in mV to the packet function
    uart_write_blocking(CRSF_UART_TX, packet_buffer, CRSF_BATTERY_FRAME_SIZE);
}
#endif  // TELEMETRY

// CRSF packet structure for 16 channels
// Frame: [Sync][Len][Type][Payload...][CRC]

void crsf_decode_loop(void)
{
    static uint8_t buffer[26];
    static uint8_t idx = 0;
    uint64_t last_byte_time = 0;

    while (true)
    {
        if (uart_is_readable(CRSF_UART_RX))
        {
            uint8_t byte = uart_getc(CRSF_UART_RX);

            last_byte_time = time_us_64();

            // Sync byte for CRSF is 0xC8
            if (idx == 0 && byte != 0xC8)
                continue;  // Wait for sync byte at the start of a packet
            if (idx == 1 && byte != 24)
            {              // Length of channel data should be 22 bytes + 1 type + 1 CRC = 24 total, so length byte should be <= 25
                idx = 0;   // Invalid length, reset to wait for next sync
                continue;
            }
            if (idx == 2 && byte != 0x16)
            {             // We expect type 0x16 for RC Channels Packed
                idx = 0;  // Invalid type, reset to wait for next sync
                continue;
            }

            buffer[idx++] = byte;  // We're in a RC data packet, store it in buffer and move index

            if (idx >= 26)
            {                      // Packet complete
                uint8_t calculated_crc = crc8(&buffer[2], idx - 3);
                if (calculated_crc != buffer[idx - 1])
                {
                    printf("\nCRC Mismatch! Calculated: 0x%02X, Received: 0x%02X\n", calculated_crc, buffer[idx - 1]);
                    idx = 0;  // Reset for next packet
                    continue;
                }
                _crsf_decode_channels(buffer);
                idx = 0;
            }
        }
        else
        {  //  no char in fifo buffer
            if ((time_us_64() - last_byte_time) > TIMEOUT_US)
            {
                // We are in the inter-frame gap.
                idx = 0;
            }
        }
    }
}

/**
 * Constructs a CRSF Battery Frame
 * @param buffer: Pointer to a uint8_t array of at least 12 bytes
 * @param voltage_mv: Calibrated battery voltage in millivolts
 */
void crsf_battery_packet(uint8_t *buffer, uint32_t voltage_mv)
{
    // 1. Set Header
    buffer[0] = CRSF_SYNC_BYTE;
    buffer[1] = 10;  // Length: Type(1) + Payload(8) + CRC(1)
    buffer[2] = CRSF_TYPE_BATTERY;

    // 2. Voltage (Big Endian)
    // CRSF expects units of 0.1V. (e.g., 11.1V = 111)
    uint16_t v = (uint16_t)(voltage_mv / 100);
    buffer[3] = (v >> 8) & 0xFF;
    buffer[4] = v & 0xFF;

    // 3. Current (0.1A steps) - Set to 0 if not monitored
    buffer[5] = 0;
    buffer[6] = 0;

    // 4. Capacity (1mAh steps) - Set to 0 if not monitored (3 bytes)
    buffer[7] = 0;
    buffer[8] = 0;
    buffer[9] = 0;

    // 5. Remaining Percentage (0-100)
    buffer[10] = 100;

    // 6. Calculate CRC (Starts from Type byte at index 2, length 9)
    buffer[11] = crc8(&buffer[2], 9);
}
