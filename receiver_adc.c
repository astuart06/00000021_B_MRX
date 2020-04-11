/******************************************************************************
 * File:   receiver_adc.c
 * Author: Andrew Stuart
 *
 ******************************************************************************/

/*******************************************************************************
* INCLUDES
*******************************************************************************/
#include "receiver_adc.h"
#include "p24F16KA302.h"

#include "stdio.h"
#include <stdlib.h>
#include "xc.h"

#include "protocol.h"
#include "globals.h"

#include "receiver_sram.h"
#include "receiver_spi.h"

/******************************************************************************
* adc_init
* 
* Description:
* 
*
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
void peripheral_adc_init(){
    
    AD1CON1bits.MODE12 = 1;     // Set the resolution to 12-bits.
    AD1CON1bits.FORM = 0b00;    // Unsigned result, right-justified.
    
    AD1CON1bits.SSRC = 0b0001;  // INT0 event ends sampling and begins conv.
    AD1CON3bits.SAMC = 0b11111; // Sampling time (SAMC * Tad).
    AD1CON3bits.ADCS = 2;      // sample conversion clock(ADCS * Tcy).
    AD1CON1bits.ASAM = 1;       // Start sampling automatically after last conv.
    
    AD1CON2bits.PVCFG = 0;      // Set the ADC Vref+ to Vdd and Vref- to Vss.
    AD1CON2bits.NVCFG = 0;
    
    TRISBbits.TRISB12 = 1;      // Set B12 as input
    ANSBbits.ANSB12 = 1;        //  and disable digital input buffer.
    
    AD1CHSbits.CH0SA = 12;      // Using MUX A set +ve input to AN12 and 
    AD1CHSbits.CH0NA = 0;       // -ve input to Vss and always use MUX A.
    AD1CON2bits.ALTS = 0;
    
    IEC0bits.AD1IE = 1;         // Enable conversion complete interrupt.
    AD1CON2bits.SMPI = 0;       // Set interrupt flag for every conversion.
    IFS0bits.AD1IF = 0;         // Clear A/D conversion interrupt flag.
   
    AD1CON1bits.ADON = 0;       // Module disabled, cmd from Host to turn it on.
}

void adc_en_handler(void){
    unsigned char state;
    
    state = spi_data_rx[USB_PACKET_DATA_0];     // Grab the enable bit.
    state &= 1;                                 // Check it is just one bit.
    
    if(state == 1){
        hardware_sram_init(SRAM_WRITE);         // Select SRAM write mode if the ADC is running.
    }  
    else{
        hardware_sram_init(SRAM_READ);  
    }
    AD1CON1bits.ADON = state;                   // Enable/disable the ADC module.
    
    spi_tx_wait_init(spi_data_rx, 8);
}