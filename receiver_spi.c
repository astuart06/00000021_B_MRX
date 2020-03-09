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

/******************************************************************************
* spi_init
* 
* Description:
* 
*
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
void spi_init(){  
    SPI1CON1bits.PPRE = 0b11;   // Set the SPI clock frequency to 500kHz.
    SPI1CON1bits.SPRE = 0b000;
    
    SPI1CON1bits.MSTEN = 0;     // Slave mode
    SPI1CON1bits.SSEN = 0;      // CS manually monitored
    SPI1CON1bits.MODE16 = 0;    // 8-bit mode
    SPI1CON2bits.SPIBEN = 1;    // Enable the enhanced buffer.
        
    SPI1STATbits.SPIROV = 0;    // Clear overflow flag.
    
    SPI1STATbits.SISEL = 0b001; // Interrupt when data available is RX buffer.
    IPC2bits.SPI1IP = 0b111;    // Highest interrupt priority
    IFS0bits.SPI1IF = 0;        // Clear the Interrupt flag
    IEC0bits.SPI1IE = 1;        // Enable the interrupt    
    
    SPI1STATbits.SPIEN = 1;     // Enable the SPI module 
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
void spi_tx_buffer_write(unsigned int *tx_data, unsigned int size_bytes){
    unsigned int i;
    
    i = 0;
    while(i < size_bytes){
        if(!SPI1STATbits.SPITBF){       // Any space in tx buffer?
            SPI1BUF = *tx_data;                   
            tx_data++;
            i++;
        }
/*        
        while(!SPI1STATbits.SRXMPT){    // Read and discard any data in the
            SPI1BUF;                    // rx FIFO.
        }                              
*/
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
    
    while(!SPI1STATbits.SPITBF){
        SPI1BUF = value;
        i++;
    }
}   

