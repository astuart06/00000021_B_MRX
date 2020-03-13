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
#include "config.h"

// Instruction cycle frequency, Hz - required for __delayXXX() to work
#define  FCY    8000000UL
#include "libpic30.h"

#include "receiver_adc.h"
#include "receiver_i2c.h"
#include "receiver_spi.h"

#include "protocol.h"
#include "globals.h"

/*******************************************************************************
* DEFINITIONS
*******************************************************************************/
#define TRIGGER_ADC     PORTBbits.PORTB7
#define ADC_DONE        PORTBbits.RB1
#define SRAM_IO_5       LATBbits.LATB5

#define SPI_PACKET_SIZE 8

/*******************************************************************************
* PROTOTYPES
*******************************************************************************/
void __attribute__((__interrupt__, auto_psv)) _SPI1Interrupt();
void __attribute__((__interrupt__, auto_psv)) _INT0Interrupt();

void int0_init(void);
void peripheral_io_init(void);


/*******************************************************************************c
* GLOBAL VARIABLES
*******************************************************************************/
unsigned int spi_data_tx[64];

/*******************************************************************************
* INTERRUPTS
*******************************************************************************/
/*
void __attribute__((__interrupt__, auto_psv)) _SPI1Interrupt(){
    static int i = 0; 
   
    for(i = 0; i < 8; i++){
        spi_data_rx[i] = SPI1BUF;
    }
    state = spi_reply;                                
   
   IFS0bits.SPI1IF = 0;          // Clear the Interrupt flag.   
}
*/

void __attribute__((__interrupt__, auto_psv)) _INT0Interrupt(){    
    // ADC conversion started automatically on +ve edge on INT0 (TRIGGER_ADC).
    ADC_DONE = 1;           // Tell master we are busy, cleared in main loop.
    
    IFS0bits.INT0IF = 0;        // Clear the interrupt flag    
}

/******************************************************************************
* int0_init
* 
* Description:
* 
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
void int0_init(void){
    INTCON2bits.INT0EP = 0;     // +ve edge trigger
    IFS0bits.INT0IF = 0;        // Clear interrupt flag
    IPC0bits.INT0IP = 7;        // Highest priority
}

/******************************************************************************
* peripheral_io_init
* 
* Description:
* 
*
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
void peripheral_io_init(){
    ANSBbits.ANSB2 = 0;
    ANSBbits.ANSB4 = 0;
    ANSBbits.ANSB15 = 0;
    ANSBbits.ANSB3 = 0;
    ANSBbits.ANSB4 = 0;
    ODCB = 0;    
    
    TRISBbits.TRISB4 = 1;           // CS as an input
    TRISBbits.TRISB1 = 0;           // ADC_DONE as an output
}
/*******************************************************************************
* MAIN
*******************************************************************************/
int main() {        
    spi_data_tx[0] = 0x60;
    spi_data_tx[1] = 0x61;
    spi_data_tx[2] = 0x62;
    spi_data_tx[3] = 0x63;
    spi_data_tx[4] = 0x64;
    spi_data_tx[5] = 0x65;
    spi_data_tx[6] = 0x66;
    spi_data_tx[7] = 0x67;    
    
    peripheral_io_init();
    peripheral_adc_init();
    peripheral_spi_init();         
    
    spi_fill_tx_buffer(0xF1);   // Dummy data for first request packet from
                                // master.
    
    fsm_state = fsm_spi_rx_msg;         // Initial state.
    while(1){
        switch(fsm_state){
            case fsm_spi_rx_msg:
                spi_rx_msg_init();
                break;
            
            case fsm_spi_tx_msg:
                break;
    
            case fsm_spi_msg_decode:
                break;
                
            case fsm_spi_error:
                break;
    
            case fsm_i2c_pot_inc:
                break;
                
            case fsm_i2c_pot_dec:
                break;
                
            case fsm_i2c_pot_read:
                break;
                
            case fsm_i2c_pot_write:
                break;
                
            case fsm_adc_aquisition:
                break;
    
            case fsm_adc_read:
                break;
        }
    }
    return 0;
}


