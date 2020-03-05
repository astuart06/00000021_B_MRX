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
    
    SPI1STATbits.SPIROV = 0;    // Clear overflow flag.
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
int spi_transfer(int data){
    while(SPI1STATbits.SPITBF); 
    SPI1BUF = data;
    SPI1STATbits.SPIROV = 0;
    while(!SPI1STATbits.SPIRBF);
    return SPI1BUF;
}
