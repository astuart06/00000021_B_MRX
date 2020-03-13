// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef GLOBALS_H
#define	GLOBALS_H

#include "p24F16KA302.h"
#include "xc.h"

/*******************************************************************************
* DEFINITIONS
*******************************************************************************/

/*******************************************************************************
* ENUMS
*******************************************************************************/
enum device_state{
    fsm_spi_rx_msg,
    fsm_spi_tx_msg,
    fsm_spi_msg_decode,
    fsm_spi_error,
    fsm_i2c_pot_inc,
    fsm_i2c_pot_dec,
    fsm_i2c_pot_read,
    fsm_i2c_pot_write,
    fsm_adc_aquisition,
    fsm_adc_read,
    } fsm_state;


#endif // GLOBALS_H