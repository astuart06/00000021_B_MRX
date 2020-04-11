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

#define DEBUG_SLAVE     // To debug slave SLAVE_STATE cannot be used.

/*******************************************************************************
* DEFINITIONS
*******************************************************************************/
#define TRIGGER_ADC     PORTBbits.RB7
#define SLAVE_STATE     LATBbits.LATB1   

// These are opposite to the master as the open collector transistor on the
// output inverts the logic.
#define SLAVE_ACTIVE    1
#define SLAVE_IDLE      0

/*******************************************************************************
* VARIABLES
*******************************************************************************/
extern unsigned char spi_data_rx[USB_PACKET_SIZE];
extern unsigned char spi_data_tx[USB_PACKET_MAX];
extern unsigned char spi_data_dummy[USB_PACKET_MAX];

/*******************************************************************************
* ENUMS
*******************************************************************************/
typedef enum{
    EV_CMD_NONE,
    EV_CMD_POT  = 0x10,
    EV_CMD_SRAM = 0x11,
    EV_CMD_ADC  = 0x12
} device_event_t;

typedef enum{
    ST_SPI_RX,
    ST_SPI_TX,
    ST_DIGIPOT_RW,
    ST_SRAM_READ,
    ST_ADC_EN
} device_state_t;

extern device_event_t next_event;
extern device_state_t next_state;
    
#endif // GLOBALS_H