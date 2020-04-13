/******************************************************************************
 * File:   receiver_i2c.c
 * Author: Andrew Stuart
 *
 ******************************************************************************/

/*******************************************************************************
* INCLUDES
*******************************************************************************/
#include "receiver_i2c.h"
#include "p24F16KA302.h"

#include "stdio.h"
#include <stdlib.h>
#include "xc.h"

#include "globals.h"
#include "receiver_spi.h"

/******************************************************************************
* peripheral_i2c_init
* 
* Description:
* 
*
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
void peripheral_i2c_init(){   
    ODCBbits.ODB2 = 1;                  // Set the SDA and SCL pins as open 
    ODCBbits.ODB3 = 1;                  // drain outputs.
       
    I2C2CONbits.IPMIEN = 0;             // Disable IPMI mode
    I2C2CONbits.A10M = 0;               // 7-bit slave address
    I2C2CONbits.I2CEN = 1;              // Enable the I2C module
    I2C2BRG = 0x9D;                     // Set I2C clock frequency to 100kHz
}

void digipot_handler(void){


    spi_tx_wait_init(spi_data_rx, 8);
}

/******************************************************************************
* pot_read_init
* 
* Description:
* 
*
* Inputs:
* None      
* Returns:
* None
******************************************************************************/
//void pot_read_init(void){
//    while(TRIGGER_ADC == 0);
//    SLAVE_STATE = SLAVE_ACTIVE;
//    fsm_state = fsm_i2c_pot_read;
//}


/******************************************************************************
* i2c_test_write
* 
* Description:
* 
*
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
void i2c_test_write(int addr_device){
    i2c_wait_for_idle();
    i2c_start();
    i2c_write(addr_device);
    i2c_wait_for_idle();
    i2c_write(0x00);
    i2c_wait_for_idle();
    i2c_write(0x80);
    i2c_wait_for_idle();
    i2c_stop();
}

/******************************************************************************
* i2c_test_read
* 
* Description:
* 
*
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
int i2c_test_read(){
    int rx_data_0, rx_data_1;
    
    i2c_wait_for_idle();
    i2c_start();
    i2c_write(0x5F);            // R/W bit set to read 
    i2c_wait_for_idle();
    I2C2CONbits.RCEN = 1;
    // Wait for RCEN bit to clear (8th falling clk edge occured).
    while(I2C2CONbits.RCEN);
    // Wait until the receive buffer is full.
    while(!I2C2STATbits.RBF);
    rx_data_0 = I2C2RCV;
    // ACK the data we have just received.
    I2C2CONbits.ACKEN = 1;
    i2c_wait_for_idle();
    // Repeat for the next byte
    I2C2CONbits.RCEN = 1;
    while(I2C2CONbits.RCEN);
    while(!I2C2STATbits.RBF);
    rx_data_1 = I2C2RCV;
    i2c_wait_for_idle();
    i2c_stop();
    
    return rx_data_1;
}

/******************************************************************************
* i2c_wait_for_idle
* 
* Description:
* 
*
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
void i2c_wait_for_idle(){               // When these bits are set...
        while(  I2C2CONbits.SEN ||      // Start condition 
                I2C2CONbits.PEN ||      // Stop condition
                I2C2CONbits.RCEN ||     // Receiving sequence           
                I2C2CONbits.ACKEN ||    // Acknowledge sequence
                I2C2STATbits.TRSTAT);   // Master transmit
}
/******************************************************************************
* i2c_start
* 
* Description:
* 
*
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
void i2c_start(){
    // Generate a 'start' condition on the bus, then wait for the
    // module to clear the bit.
    I2C2CONbits.SEN = 1;
    while(I2C2CONbits.SEN);
}
/******************************************************************************
* i2c_stop
* 
* Description:
* 
*
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
void i2c_stop(){
    // Generate a 'stop' condition on the bus, then wait for the
    // module to clear the bit
    I2C2CONbits.PEN = 1;
    while(I2C2CONbits.PEN);
}
/******************************************************************************
* i2c_write
* 
* Description:
* 
*
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
void i2c_write(int data){
    // Load the data into tx buffer
    I2C2TRN = data;
    // Wait until the tx buffer is empty
    while(I2C2STATbits.TBF);    
}

