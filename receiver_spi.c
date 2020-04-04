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
#include <string.h>

#include "protocol.h"
#include "globals.h"
#include "receiver_i2c.h"
#include "receiver_sram.h"


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
    
    memset(spi_data_dummy, 0, USB_PACKET_MAX);
                                // Write some know value to the dummy buffer.    
    SPI1STATbits.SPIEN = 1;     // Enable the SPI module.    
}

unsigned char spi_rx_wait(void){   
    spi_transfer(spi_data_dummy, spi_data_rx, 8);
    return spi_data_rx[USB_PACKET_CMD];  // Return command byte.
}

void spi_tx_wait(void){
    while(TRIGGER_ADC == 0);    // Wait until master sets trigger high.
    SLAVE_STATE = SLAVE_ACTIVE; // Tell master we are active.   
    Nop();
    SLAVE_STATE = SLAVE_IDLE;     // Once set to idle, master will initiate SPI transfer.
    
    spi_transfer(spi_data_tx, spi_data_dummy, spi_data_rx[USB_PACKET_RXBYTES]);
}

/*******************************************************************************
* spi_transfer
* 
* Description:
* 
*
* Inputs:
*      
* Returns:
* 
 ******************************************************************************/
void spi_transfer(unsigned char *tx_data, unsigned char *rx_data, unsigned int size_bytes){
    unsigned int bytes_txd, bytes_rxd;
    
    bytes_txd = 0;
    bytes_rxd = 0;
      
    while(1){
        // SPITBF bit is clear when buffer available for write instruction.
        if(!SPI1STATbits.SPITBF && (bytes_txd < size_bytes)){       
            SPI1BUF = *tx_data;                   
            tx_data++;
            bytes_txd++;
        }
        // SPXMPT bit clear when the rx FIFO is not empty.
        if(!SPI1STATbits.SRXMPT && (bytes_rxd < size_bytes)){
            *rx_data = SPI1BUF;
            rx_data++;
            bytes_rxd++;
        }
        if((bytes_txd >= size_bytes) && (bytes_rxd >= size_bytes)){
            break;
        }
    }
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
    
    SLAVE_STATE = SLAVE_IDLE;     // Ready to tx, tell the master to clock SPI bus.
    spi_transfer(spi_data_tx, spi_data_dummy, 8);
}