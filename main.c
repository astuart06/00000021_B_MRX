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

#include "receiver_adc.h"
#include "receiver_i2c.h"
#include "receiver_spi.h"
#include "receiver_sram.h"
#include "receiver_misc.h"

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
device_state_t          next_state;
device_event_t          next_event;

unsigned char           spi_data_tx[USB_PACKET_MAX];
unsigned char           spi_data_rx[8];
unsigned char           spi_data_dummy[USB_PACKET_MAX];

unsigned int            dummy_adc_value;
unsigned int            array[20];

/*******************************************************************************
* INTERRUPTS
*******************************************************************************/
void __attribute__((__interrupt__, auto_psv)) _INT0Interrupt(){
    // ADC conversion started automatically on +ve edge on INT0 (TRIGGER_ADC).
    // So just tell the master we are 'busy' and clear the flag.
    // ***NOT CALLED AT THE MOMENT***
    
    IFS0bits.INT0IF = 0;        // Clear the interrupt flag    
}

// ADC1 interrupt occurs after every conversion is complete (SMPI = 0).
void __attribute__((__interrupt__, auto_psv)) _ADC1Interrupt(){
    unsigned int adc_value;
    static int i = 0;
    static unsigned int adc_data[512];

    SLAVE_STATE = SLAVE_ACTIVE;
    adc_value = ADC1BUF0;
    adc_data[i] = adc_value;
    if(i > 200){
        Nop();
    }
    else{
        i++;
    }
    //sram_write(i);
        
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
    // Disable analog inputs on the following pins...
    ANSAbits.ANSA0 = 0;     // IO 0
    ANSAbits.ANSA1 = 0;     // IO 1
    ANSAbits.ANSA2 = 0;     // IO 2
    ANSAbits.ANSA3 = 0;     // IO 3
    
    ANSBbits.ANSB2 = 0;     // SDA2
    ANSBbits.ANSB3 = 0;     // SCL2
    ANSBbits.ANSB4 = 0;     // SPI CS
    ANSBbits.ANSB14 = 0;    // IO 10
    ANSBbits.ANSB15 = 0;    // IO 11
        
    ODCA = 0;
    ODCB = 0;    
    
    TRISBbits.TRISB2 = 0;   // DEBUG_RB2 is an output.
    TRISBbits.TRISB3 = 0;   // SLAVE_STATE as an output.
    TRISBbits.TRISB4 = 1;   // SPI CS as an input. 
    TRISBbits.TRISB7 = 1;   // TRIGGER_ADC as an input.
    
    CM1CONbits.CON = 0;
    REFOCONbits.ROEN = 0;
    
    RCFGCALbits.RTCEN = 0;
    I2C1CONbits.I2CEN = 0;
    
    DEBUG_RB2 = 0;          // Default state of debug ouput.
}
/*******************************************************************************
* MAIN
*******************************************************************************/
int main(){
    peripheral_io_init();
    peripheral_adc_init();
    peripheral_spi_init();
    //peripheral_i2c_init();        // i2c hardware NF.
    
    hardware_sram_init(SRAM_WRITE); // Default to write mode. (remove when done by handler).
    
    // Disable devices on RA4/6, just a test...
    CM2CONbits.CON = 0;
    CM2CONbits.COE = 0;
    CTMUCON1bits.CTMUEN = 0;
            
    next_state = ST_SPI_RX;
    while(1){      
        switch(next_state){
            case ST_SPI_RX:
                spi_rx_wait_handler();      // spi_rx_wait_handler() is blocking so it waits for an event.
                break;

            case ST_SPI_TX:
                spi_tx_wait_handler();
                break;                
                
            case ST_DIGIPOT_RW:
                digipot_handler();
                break;
                
            case ST_SRAM_READ:  
                sram_read_handler();
                break;
                
            case ST_ADC_EN:                
                adc_en_handler();                
                break;
                
            case ST_SLAVE_ID:
                slave_id_handler();
                break;
        }
    }
    return 0;
}


