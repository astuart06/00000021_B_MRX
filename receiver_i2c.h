// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef RECEIVER_I2C_H
#define	RECEIVER_I2C_H

#include "p24F16KA302.h"
#include "xc.h"

/*******************************************************************************
* DEFINITIONS
*******************************************************************************/
#define I2C_ADDR_DIGIPOT    0x5C

/*******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void peripheral_i2c_init(void);

void pot_read_init();    

void i2c_test_write(int addr_device);
int i2c_test_read(void);
void i2c_wait_for_idle(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(int data);

#endif // RECEIVER_I2C_H