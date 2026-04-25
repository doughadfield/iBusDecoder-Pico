#ifndef CRSF_H
#define CRSF_H

#include <stdlib.h>

// UART defines
#define CRSF_UART_RX uart0
#define CRSF_BAUD_RATE 420000
#define CRSF_TX_PIN 28      // TX pin is not used in this example, but defined for completeness
#define CRSF_RX_PIN 29

// CRSF defines
#define CRSF_MAX_LENGTH 64 // CRSF Maximum packet Length
#define CRSF_NUM_CHANNELS 16   // Maximum number of channels in CRSF payload

#define CRSF_SYNC_BYTE      0xC8
#define CRSF_PAYLOAD_TYPE   0x16

void crsf_init(void);
void crsf_decode_loop(void);

// Global array holding the latest valid channel values (1000–2000 μs typical)
extern volatile uint16_t RC_Channels[CRSF_NUM_CHANNELS];

// Flag to indicate new valid data is available (optional - for main core polling)
extern volatile uint8_t RC_new_data_flag;

#endif // CRSF_H

