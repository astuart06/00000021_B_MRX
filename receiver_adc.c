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
void adc_init(){
    // Set the resolution to 12-bits.
    AD1CON1bits.MODE12 = 1;
    // Unsigned result, right-justified.
    AD1CON1bits.FORM = 0b00;
    // Sample clock source select bits (0111 = internal counter)
    AD1CON1bits.SSRC = 0b0111;
    // Set the sample time before conversion starts (0b11111 = 31 Tad)
    AD1CON3bits.SAMC = 0b11111;
    // Set the ADC Vref+ to Vdd and Vref- to Vss.
    AD1CON2bits.PVCFG = 0;
    AD1CON2bits.NVCFG = 0;
    // Set the sample conversion rate to 16 * Tcy (Tad = 1us).
    AD1CON3bits.ADCS = 16;
    // Set B12 as input and disable digital input buffer.
    TRISBbits.TRISB12 = 1;
    ANSBbits.ANSB12 = 1;
    // Using MUX A set +ve input to AN12 and -ve input to Vss and always use
    // MUX A.
    AD1CHSbits.CH0SA = 12;
    AD1CHSbits.CH0NA = 0;
    AD1CON2bits.ALTS = 0;
    // Clear A/D conversion interrupt.
    IFS0bits.AD1IF = 0;  
    
    // Turn on the ADC module
    AD1CON1bits.ADON = 1;
    
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