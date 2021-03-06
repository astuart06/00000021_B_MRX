// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef GLOBALS_H
#define	GLOBALS_H

#include "p24F16KA302.h"
#include "xc.h"

#include "protocol.h"

// Instruction cycle frequency, Hz - required for __delayXXX() to work
#define  FCY    8000000UL
#include "libpic30.h"

/*******************************************************************************
* DEFINITIONS
*******************************************************************************/
#define TRIGGER_ADC     PORTBbits.RB7
#define SLAVE_STATE     LATBbits.LATB3
#define DEBUG_RB2       LATBbits.LATB2

// These are opposite to the master as the open collector transistor on the
// output inverts the logic.
#define SLAVE_ACTIVE    1
#define SLAVE_IDLE      0

#define ADC_START_VALUE 0xAA;

/*******************************************************************************
* VARIABLES
*******************************************************************************/
extern unsigned char spi_data_rx[USB_PACKET_SIZE];
extern unsigned char spi_data_tx[USB_PACKET_MAX];
extern unsigned char spi_data_dummy[USB_PACKET_MAX];

extern unsigned int dummy_adc_value;

/*******************************************************************************
* ENUMS
*******************************************************************************/
typedef enum{
    EV_CMD_NONE,
    EV_CMD_POT  = 0x10,
    EV_CMD_SRAM = 0x11,
    EV_CMD_ADC  = 0x12,
    EV_CMD_ID   = 0x13
} device_event_t;

typedef enum{
    ST_SPI_RX,
    ST_SPI_TX,
    ST_DIGIPOT_RW,
    ST_SRAM_READ,
    ST_ADC_EN,
    ST_SLAVE_ID
} device_state_t;

extern device_event_t next_event;
extern device_state_t next_state;
    
#endif // GLOBALS_H