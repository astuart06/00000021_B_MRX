/******************************************************************************
 * Project: Sonic screwdriver - Receiver Micro
 * File:    main.c
 * Author:  Andrew Stuart
 ******************************************************************************/

/*******************************************************************************
* INCLUDES
*******************************************************************************/
#include "xc.h"
#include "p24F16KA302.h"
/*
#include "GenericTypeDefs.h"

#include "stdio.h"
#include "stdlib.h"
#include "time.h"
*/
#include "config.h"

// Instruction cycle frequency, Hz - required for __delayXXX() to work
#define  FCY    16000000UL
#include "libpic30.h"

#include "receiver_adc.h"
#include "receiver_i2c.h"
#include "receiver_spi.h"

/*******************************************************************************
* DEFINITIONS
*******************************************************************************/
#define TRIGGER_ADC     PORTBbits.PORTB7
#define SRAM_IO_5       LATBbits.LATB5

#define SPI_PACKET_SIZE 8

enum device_state{
    spi_request,
    spi_reply} state;

/*******************************************************************************
* PROTOTYPES
*******************************************************************************/
void __attribute__((__interrupt__, auto_psv)) _SPI1Interrupt();
void __attribute__((__interrupt__, auto_psv)) _INT0Interrupt();

/*******************************************************************************c
* GLOBAL VARIABLES
*******************************************************************************/
unsigned int spi_data_rx[8];
unsigned int spi_data_tx[64];

/*******************************************************************************
* INTERRUPTS
*******************************************************************************/

void __attribute__((__interrupt__, auto_psv)) _SPI1Interrupt(){
    static int i = 0; 
   
    if(state == spi_request){
        spi_data_rx[i++] = SPI1BUF;  
    }
    else{
        SPI1BUF;                    // Throw the data away if not in requeset
    }                               // mode.    
   
   if(i > (SPI_PACKET_SIZE - 1)){   // Full packet rx'd from master, set the
       state = spi_reply;           // mode to reply.
       i = 0;                       
   }                               
   
   IFS0bits.SPI1IF = 0;          // Clear the Interrupt flag.   
}

void __attribute__((__interrupt__, auto_psv)) _INT0Interrupt(){    
    unsigned int adc_value;
    
    IFS0bits.INT0IF = 0;        // Clear the interrupt flag
    adc_value = adc_read();
}

/*******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
int main() {
    
    int i;
    unsigned char j;
    int toggle_state;

    ANSBbits.ANSB2 = 0;
    ANSBbits.ANSB4 = 0;
    ANSBbits.ANSB15 = 0;
    ANSBbits.ANSB3 = 0;

    ANSBbits.ANSB4 = 0;
    ODCB = 0;    
    
    //TRISBbits.TRISB13 = 0;
    toggle_state = 1;
    
    TRISBbits.TRISB4 = 1;           // CS as an input
    
    spi_data_tx[0] = 0x60;
    spi_data_tx[1] = 0x61;
    spi_data_tx[2] = 0x62;
    spi_data_tx[3] = 0x63;
    spi_data_tx[4] = 0x64;
    spi_data_tx[5] = 0x65;
    spi_data_tx[6] = 0x66;
    spi_data_tx[7] = 0x67;    
    
    adc_init();
    spi_init();                     
    state = spi_request;        // First packet is always a request.
    spi_fill_tx_buffer(0xF1);   // Dummy data for first request packet from
                                // master.
    while(1){
        if(state == spi_reply){
            spi_tx_buffer_write(spi_data_tx, 8);
            spi_fill_tx_buffer(0xF1); 
            state = spi_request;
        }
    }
    
/*
        for(i = 0; i < 4; i++){
            SPI1STATbits.SPIROV = 0;
            while (SPI1STATbits.SPITBF); 
            SPI1BUF = send_data[i];
        }
        
        toggle_state = 0;
        toggle_state = 1;
*/
    
    return 0;
}


