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
    
    IEC0bits.AD1IE = 1;         // Disable conversion complete interrupt.
    AD1CON2bits.SMPI = 0;       // Set interrupt flag for every conversion.
    IFS0bits.AD1IF = 0;         // Clear A/D conversion interrupt flag.
   
    AD1CON1bits.ADON = 1;       // Turn on the ADC module.
}

/******************************************************************************
* adc_read
* 
* Description:
* 
*
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
int adc_read(){
    int adc_value;
    
    // Set the SAMP bit to start sampling
    AD1CON1bits.SAMP = 1;
    // Wait for the sampling and conversion  to finish.
    while(!AD1CON1bits.DONE){};
    adc_value = ADC1BUF0;
    
    return adc_value;
}