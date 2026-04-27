#ifndef CRSF_H
#define CRSF_H

#include <stdlib.h>

#define TELEMETRY  // Enable telemetry output

// UART defines
#define CRSF_UART_RX uart0
#define CRSF_UART_TX uart1
#define CRSF_BAUD_RATE 420000
#define CRSF_TX_PIN 4   // UART1 is used for telemetry output
#define CRSF_RX_PIN 29  // UART0 RX pin for CRSF input
#define ARM_CHANNEL 5  // Channel number for motor arming switch

// CRSF defines
#define CRSF_MAX_LENGTH 64    // CRSF Maximum packet Length
#define CRSF_NUM_CHANNELS 16  // Maximum number of channels in CRSF payload

#define CRSF_SYNC_BYTE 0xC8
#define CRSF_PAYLOAD_TYPE 0x16

#define CRSF_TYPE_BATTERY 0x08      // CRSF frame type for battery sensor data
#define CRSF_BATTERY_FRAME_SIZE 12  // size of buffer to hold a CRSF battery frame (header + length + type + payload + CRC)

void crsf_init(void);
void crsf_decode_loop(void);
void crsf_battery_packet(uint8_t *buffer, uint32_t voltage_mv);
void crsf_telemetry_send(uint8_t *packet_buffer);

// Global array holding the latest valid channel values (1000–2000 μs typical)
extern volatile uint16_t RC_Channels[CRSF_NUM_CHANNELS];

// Flag to indicate new valid data is available (optional - for main core polling)
extern volatile uint8_t RC_new_data_flag;

extern uint8_t crsf_packet[CRSF_BATTERY_FRAME_SIZE];
#endif  // CRSF_H
