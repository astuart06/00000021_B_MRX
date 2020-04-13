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

void digipot_init(void){
    next_state = ST_DIGIPOT_RW;
}

void digipot_handler(void){
    unsigned char data_buffer[8];
    unsigned char result;
    unsigned char ack_status;
    
    ack_status = digipot_write(spi_data_rx[USB_PACKET_DATA_0]);
    __delay_us(100);
    result = digipot_read();
    
    data_buffer[0] = ack_status;
    data_buffer[1] = result;
    data_buffer[2] = 0x55;      // Temp test data.
    data_buffer[3] = 0xAA;
    
    
    spi_tx_wait_init(data_buffer, 8);
}

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
unsigned char digipot_write(unsigned char value){
    unsigned char ack_status_0, ack_status_1, ack_status_2;
    
    i2c_wait_for_idle();
    i2c_start();
    
    i2c_write(I2C_ADDR_DIGIPOT_WRITE);
    i2c_wait_for_idle();
    ack_status_0 = I2C2STATbits.ACKSTAT;        
    
    i2c_write(0x00);            // Addr: b0000 (wiper 0). Cmd: b00 (Write Data). D8: b0 (Data MSb).
    i2c_wait_for_idle();
    ack_status_1 = I2C2STATbits.ACKSTAT;        
    
    i2c_write(value);            // D7-D0
    i2c_wait_for_idle();
    ack_status_2 = I2C2STATbits.ACKSTAT;        
    i2c_stop();
    
    return (ack_status_2 << 2) | (ack_status_1 << 1) | ack_status_0;
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
unsigned char digipot_read(void){
    int rx_data_msb, rx_data_lsb;
    unsigned char ack_status;
    
    i2c_wait_for_idle();
    i2c_start();
    
    i2c_write(I2C_ADDR_DIGIPOT_READ);
    i2c_wait_for_idle();
    ack_status = I2C2STATbits.ACKSTAT;        
    
    I2C2CONbits.RCEN = 1;       // Gen clock pulse to read data from slave.
    while(I2C2CONbits.RCEN);    // Wait for RCEN bit to clear (8th falling clk edge occurred).
    while(!I2C2STATbits.RBF);   // Wait until the receive buffer is full.
    rx_data_msb = I2C2RCV;
    I2C2CONbits.ACKEN = 1;      // Master ACK the data we have just received.
    i2c_wait_for_idle();
    
    I2C2CONbits.RCEN = 1;       // Repeat for the next byte
    while(I2C2CONbits.RCEN);
    while(!I2C2STATbits.RBF);
    rx_data_lsb = I2C2RCV;        // Note that a Master ACK is not generated.
    i2c_wait_for_idle();
    i2c_stop();
    
   // return (rx_data_msb << 8) | rx_data_lsb;
    return ack_status;
}

void i2c_wait_for_idle(){               // Wait when these bits are set...
        while(  I2C2CONbits.SEN     ||  // Start condition 
                I2C2CONbits.PEN     ||  // Stop condition
                I2C2CONbits.RCEN    ||  // Repeat start condition
                I2C2CONbits.RCEN    ||  // Receiving sequence           
                I2C2CONbits.ACKEN   ||  // Acknowledge sequence
                I2C2STATbits.TRSTAT);   // Master transmit
}

void i2c_start(void){

    I2C2CONbits.SEN = 1;
    while(I2C2CONbits.SEN);
}

void i2c_stop(){
    I2C2CONbits.PEN = 1;
    while(I2C2CONbits.PEN);
}

void i2c_write(int data){
    // Load the data into tx buffer
    I2C2TRN = data;
    // Wait until the tx buffer is empty
    while(I2C2STATbits.TBF);    
}

