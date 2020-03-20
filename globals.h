// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef GLOBALS_H
#define	GLOBALS_H

#include "p24F16KA302.h"
#include "xc.h"

/*******************************************************************************
* DEFINITIONS
*******************************************************************************/
#define TRIGGER_ADC     PORTBbits.PORTB7
#define SLAVE_BUSY      LATBbits.LATB1
#define SRAM_IO_5       LATBbits.LATB5

// These are opposite to the master as the open collector transistor on the
// output inverts the logic.
#define SLAVE_ACTIVE    1
#define SLAVE_IDLE      0

/*******************************************************************************
* ENUMS
*******************************************************************************/
enum device_state{
    fsm_spi_msg_rx,
    fsm_spi_msg_tx,
    fsm_spi_msg_decode,
    fsm_spi_msg_error,
    fsm_i2c_pot_inc,
    fsm_i2c_pot_dec,
    fsm_i2c_pot_read,
    fsm_i2c_pot_write,
    fsm_adc_aquisition,
    fsm_adc_read,
    };
    
extern enum device_state fsm_state; 

extern unsigned char spi_data_rx[8];
extern unsigned char spi_data_tx[64];



#endif // GLOBALS_H