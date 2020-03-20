/******************************************************************************
 * File:   receiver_spi.c
 * Author: Andrew Stuart
 *
 ******************************************************************************/

/*******************************************************************************
* INCLUDES
*******************************************************************************/
#include "receiver_spi.h"
#include "p24F16KA302.h"

#include "stdio.h"
#include <stdlib.h>

#include "protocol.h"
#include "globals.h"
#include "receiver_i2c.h"


/******************************************************************************
* peripheral_spi_init
* 
* Description:
* 
*
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
void peripheral_spi_init(){  
    SPI1CON1bits.SPRE = 0b110;  // Set the SPI clock frequency to 2MHz.
    SPI1CON1bits.PPRE = 0b01;   
    
    SPI1CON1bits.MSTEN = 0;     // Slave mode.
    SPI1CON1bits.SSEN = 0;      // CS manually monitored.
    SPI1CON1bits.MODE16 = 0;    // 8-bit mode.
    SPI1CON2bits.SPIBEN = 1;    // Enable the enhanced buffer.
        
    SPI1STATbits.SPIROV = 0;    // Clear overflow flag.
    
    SPI1STATbits.SISEL = 0b011; // Interrupt when data available is RX buffer.
    IPC2bits.SPI1IP = 0b111;    // Highest interrupt priority.
    IFS0bits.SPI1IF = 0;        // Clear the Interrupt flag.
    IEC0bits.SPI1IE = 0;        // Ensure interrupt is disabled.
    
    SPI1STATbits.SPIEN = 1;     // Enable the SPI module. 
}

/*******************************************************************************
* spi_msg_rx_init
* 
* Description:
* 
*
* Inputs:
*      
* Returns:
* 
 ******************************************************************************/
void spi_msg_rx_init(void){
    int i;
    
    if(SPI1STATbits.SPIROV)    return;      // Overflow condition?
    if(!SPI1STATbits.SPIRBF)   return;      // Buffer full yet, quit if not?
    
    for(i = 0; i < 8; i++){                 // Grab all 8 bytes of packet.
        spi_data_rx[i] = SPI1BUF;
    }
    
    if(spi_data_rx[7] != 0x65)  return;     // Simple error check.
    
    SLAVE_BUSY = SLAVE_ACTIVE;              // Tell the master slave is busy.
    master_cmd = spi_data_rx[0];            // First byte is cmd.  
    
    switch(master_cmd){
        case cmd_pot_read:
            pot_read_init();
            break;
            
        case cmd_pot_write:
            break;

        case cmd_pot_inc:
            break;
            
        case cmd_pot_dec:
            break;
            
        case cmd_mem_read:
            break;
                        
        case cmd_mem_write:
            break;
            
        case cmd_data_acq:
            break; 
    }
}

/*******************************************************************************
* spi_msg_tx_init
* 
* Description:
* 
*
* Inputs:
*      
* Returns:
* 
 ******************************************************************************/
void spi_msg_tx_init(void){
    fsm_state = fsm_spi_msg_tx;
}

/*******************************************************************************
* spi_msg_tx_handler
* 
* Description:
* 
*
* Inputs:
*      
* Returns:
* 
 ******************************************************************************/
void spi_msg_tx_handler(void){
    spi_data_tx[0] = 0x20;
    spi_data_tx[1] = 0x21;
    spi_data_tx[2] = 0x22;
    spi_data_tx[3] = 0x23;
    spi_data_tx[4] = 0x24;
    spi_data_tx[5] = 0x25;
    spi_data_tx[6] = 0x26;
    spi_data_tx[7] = 0x27;
    
    SLAVE_BUSY = SLAVE_IDLE;     // Ready to tx, tell the master to clock SPI bus.
    spi_tx_buffer_write(spi_data_tx, 8);   
    fsm_state = fsm_spi_msg_rx;
}

/*******************************************************************************
* spi_tx_buffer_write
* 
* Description:
* 
*
* Inputs:
*      
* Returns:
* 
 ******************************************************************************/
void spi_tx_buffer_write(unsigned char *tx_data, unsigned int size_bytes){
    unsigned int i;
    
    i = 0;
    while(i < size_bytes){
        if(!SPI1STATbits.SPITBF){       // Any space in tx buffer?
            SPI1BUF = *tx_data;                   
            tx_data++;
            i++;
        }
    }                                       
}

/*******************************************************************************
* spi_fill_tx_buffer
* 
* Description:
* 
*
* Inputs:
*      
* Returns:
* 
 ******************************************************************************/
void spi_fill_tx_buffer(int value){
    int i = 0;
    
    while(i < 8){        
        if(!SPI1STATbits.SPITBF){
            SPI1BUF = value;
            i++;
        }
    }
}   

