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
    SPI1CON1bits.SMP = 0;       // Must be cleared in slave mode.
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

spi_rx_wait_init(void){
    next_state = ST_SPI_RX;
}

void spi_rx_wait_handler(void){   
    // spi_transfer is a blocking function. It does not return until 8 bytes are rx'd.
    spi_transfer(spi_data_dummy, spi_data_rx, 8);
    next_event = spi_data_rx[USB_PACKET_CMD];  // Return command byte.
    
    if(next_event == EV_CMD_POT)    digipot_init();          
    if(next_event == EV_CMD_SRAM)   sram_read_init();
    if(next_event == EV_CMD_ADC)    adc_en_init();
    if(next_event == EV_CMD_ID)     slave_id_init();

}

void spi_tx_wait_init(unsigned char * data_buffer, int length){
    memcpy(spi_data_tx, data_buffer, length);
    next_state = ST_SPI_TX;
}

void spi_tx_wait_handler(void){      
    spi_transfer(spi_data_tx, spi_data_dummy, spi_data_rx[USB_PACKET_RXBYTES]);
    spi_rx_wait_init();
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
void spi_transfer(unsigned char *tx_data, unsigned char *rx_data, unsigned char size_bytes){
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