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
#include "receiver_sram.h"

#include "protocol.h"
#include "globals.h"

/*******************************************************************************
* DEFINITIONS
*******************************************************************************/
#define SPI_PACKET_SIZE 8

/*******************************************************************************
* PROTOTYPES
*******************************************************************************/
void __attribute__((__interrupt__, auto_psv)) _INT0Interrupt();
void __attribute__((__interrupt__, auto_psv)) _ADC1Interrupt();

void int0_init(void);
void peripheral_io_init(void);

/*******************************************************************************c
* GLOBAL VARIABLES
*******************************************************************************/
device_state_t      fsm_state; 
byte_cmd_t          host_cmd;

unsigned char       spi_data_tx[64];
unsigned char       spi_data_rx[8];
/*******************************************************************************
* INTERRUPTS
*******************************************************************************/
void __attribute__((__interrupt__, auto_psv)) _INT0Interrupt(){
    // ADC conversion started automatically on +ve edge on INT0 (TRIGGER_ADC).
    // So just tell the master we are 'busy' and clear the flag.
    // ***NOT CALLED AT THE MOMENT***
    SLAVE_STATE = SLAVE_ACTIVE;
    
    IFS0bits.INT0IF = 0;        // Clear the interrupt flag    
}

void __attribute__((__interrupt__, auto_psv)) _ADC1Interrupt(){
    unsigned int adc_value;
    
    SLAVE_STATE = SLAVE_ACTIVE;
    
    adc_value = ADC1BUF0;
    sram_write(adc_value);
    
    SLAVE_STATE = SLAVE_IDLE;
    IFS0bits.AD1IF = 0;        // Clear the interrupt flag    
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
    
    TRISBbits.TRISB4 = 1;           // CS as an input.
    
    TRISBbits.TRISB1 = 0;           // SLAVE_BUSY as an output.
}
/*******************************************************************************
* MAIN
*******************************************************************************/
int main(){
    peripheral_io_init();
    peripheral_adc_init();
    peripheral_spi_init();
    
    hardware_sram_init(SRAM_WRITE);
    
    spi_fill_tx_buffer(0xF1);   // Dummy data for first request packet from
                                // master.
    
    SLAVE_STATE = SLAVE_IDLE;        
    
    fsm_state = fsm_spi_msg_rx; // Initial state (waiting for spi msg).
    while(1){
        switch(fsm_state){
            case fsm_spi_msg_rx:
                spi_msg_rx_init();
                break;
            
            case fsm_spi_msg_tx:
                //__delay_us(100);
                spi_msg_tx_handler();                
                break;
    
            case fsm_spi_msg_decode:        // Remove, done in spi_msg_rx??
                break;
                
            case fsm_spi_msg_error:
                break;
    
            case fsm_i2c_pot_inc:
                break;
                
            case fsm_i2c_pot_dec:
                break;
                
            case fsm_i2c_pot_read:
                pot_read_handler();
                break;
                
            case fsm_i2c_pot_write:
                break;
                
            case fsm_adc_aquisition:
                break;
    
            case fsm_adc_read:
                break;
                
            case fsm_sram_read:
                sram_read_handler();
                break;
        }
    }
    return 0;
}


