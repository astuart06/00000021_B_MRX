// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef RECEIVER_I2C_H
#define	RECEIVER_I2C_H

#include "p24F16KA302.h"
#include "xc.h"

/*******************************************************************************
* DEFINITIONS
*******************************************************************************/
#define I2C_ADDR_DIGIPOT_WRITE  0x5C
#define I2C_ADDR_DIGIPOT_READ   0x5D

/*******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void peripheral_i2c_init(void);

void digipot_init(void);
void digipot_handler(void);

unsigned char digipot_write(unsigned char value);
unsigned char digipot_read(void);
/******************************************************************************
* i2c_wait_for_idle
* 
* Description: Wait until SEN, PEN, RCEN, ACKEN and TRSTAT bits are clear. This
*   indicates that the bus is idle.
*
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
void i2c_wait_for_idle(void);

/******************************************************************************
* i2c_start
* 
* Description: Generate a 'start' condition on the bus, then wait for the module
*   to clear the bit. A 'start' condition is a high-to-low transition on SDA
*   when SCL is high.
* 
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/

void i2c_start(void);

/******************************************************************************
* i2c_stop
* 
* Description: Generate a 'stop' condition on the bus, then wait for the module
*   to clear the bit. A 'stop' condition is a low-to-high transition on SDA
*   when SCL is high. This is the opposite of a start condition.
*
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
void i2c_stop(void);

/******************************************************************************
* i2c_write
* 
* Description: A write to the I2CxTRN register will initiate data transmission.
*   The TBF bit will clear when the transmission has finished. A 'start
*   condition must be asserted on the bus before calling this function.
* 
* Inputs:
*   data -  8-bit value to transfer.      
* Returns:
* None
 ******************************************************************************/
void i2c_write(int data);

#endif // RECEIVER_I2C_H