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


/*******************************************************************************
* PROTOTYPES
*******************************************************************************/
void change_note_init(void);
void __attribute__((__interrupt__, auto_psv)) _CNInterrupt();
void __attribute__((__interrupt__, auto_psv)) _SPI1Interrupt();

/*******************************************************************************c
* GLOBAL VARIABLES
*******************************************************************************/

int int_count;
int int_ignore;

/*******************************************************************************
* INTERRUPTS
*******************************************************************************/

void __attribute__((__interrupt__, auto_psv)) _SPI1Interrupt(){
    unsigned int spi_rx_val;
    
    IFS0bits.SPI1IF = 0;        // Clear the Interrupt flag

    spi_rx_val = SPI1BUF;
    SPI1BUF = 0x10 + spi_rx_val;
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
    int temp;
    int toggle_state;
    int result[4];
    int send_data[4];

    ANSBbits.ANSB2 = 0;
    ANSBbits.ANSB4 = 0;
    ANSBbits.ANSB15 = 0;
    ANSBbits.ANSB3 = 0;

    ANSBbits.ANSB4 = 0;
    ODCB = 0;    
    
    //TRISBbits.TRISB13 = 0;
    toggle_state = 1;
    
    TRISBbits.TRISB4 = 1;           // CS as an input
    
    send_data[1] = 0x11;
    send_data[2] = 0x12;
    send_data[3] = 0x13;
    send_data[4] = 0x14;
    
    int_count = 0;
    int_ignore = 0;
    //spi_tx_val = 0x50;
    
    adc_init();
    spi_init();
    
    while(1);
    
    while(1){                  // Wait until CS goes low
        for(i = 0; i < 8; i++){
            result[i] = spi_transfer(i);
            //while(!SPI1STATbits.SPIRBF);
            //result[i] = SPI1BUF;
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
      }
    
    return 0;
}

void change_note_init(void){
    TRISBbits.TRISB4 = 1;           // RB12, pin23 is an input.
    CNPD1bits.CN1PDE = 0;           // Pull down resistor disabled.
    CNPU1bits.CN1PUE = 0;           // Pull up resistor disabled.
    CNEN1bits.CN1IE = 1;            // Enable change notification.
    IFS1bits.CNIF = 0;              // Clear interrupt flag.
    IPC4bits.CNIP = 0b111;          // CN interrupt to highest priority.
    IEC1bits.CNIE = 0;              // Enable CN interrupts.
}


