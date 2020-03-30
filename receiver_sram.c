 /******************************************************************************
 * File:   receiver_sram.c
 * Author: Andrew Stuart
 *
 ******************************************************************************/

/*******************************************************************************
* INCLUDES
*******************************************************************************/
#include "receiver_sram.h"
#include "p24F16KA302.h"

#include "stdio.h"
#include <stdlib.h>
#include "xc.h"

#include "globals.h"

#define TRIS_INPUT  1
#define TRIS_OUTPUT 0


void sram_read_init(void){
    fsm_state = fsm_sram_read;    
}

void sram_read_handler(void){
    unsigned int value;
    
    if(TRIGGER_ADC){
        SLAVE_STATE = SLAVE_ACTIVE;
        value = sram_read();
                
    }
}

/******************************************************************************
* hardware_sram_init
* 
* Description:
* 
*
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
void hardware_sram_init(int sram_mode){
    TRISBbits.TRISB6 = 0;       // SRAM CS pin as an output 
    SRAM_CS1 = 1;               // and default to high (inactive state),
                                // also sets IO pins to high-Z.
    if(sram_mode == SRAM_WRITE){
        
    }
    else if(sram_mode == SRAM_READ){
        
    }
}

/******************************************************************************
* sram_write
* 
* Description:
* 
* The SRAM IO is connected to RA0-RA4, RA6-RA7, RB5, RB8-RB9 and RB14-RB15.
* Remember that the master needs to sets the address and !WE first.
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
void sram_write(unsigned int data){   
    LATAbits.LATA0  = data & (1U << 0);     // Bit 0 -  IO 0
    LATAbits.LATA1  = data & (1U << 1);     // Bit 1 -  IO 1
    LATAbits.LATA2  = data & (1U << 2);     // Bit 2 -  IO 2
    LATAbits.LATA3  = data & (1U << 3);     // Bit 3 -  IO 3
    LATAbits.LATA4  = data & (1U << 4);     // Bit 4 -  IO 4
    LATBbits.LATB5  = data & (1U << 5);     // Bit 5 -  IO 5
    LATAbits.LATA6  = data & (1U << 6);     // Bit 6 -  IO 6
    LATAbits.LATA7  = data & (1U << 7);     // Bit 7 -  IO 7    
    LATBbits.LATB8  = data & (1U << 8);     // Bit 8 -  IO 8
    LATBbits.LATB9  = data & (1U << 9);     // Bit 9 -  IO 9
    LATBbits.LATB14 = data & (1U << 10);    // Bit 10 - IO 10
    LATBbits.LATB15 = data & (1U << 11);    // Bit 11 - IO 11
    
    Nop();
    SRAM_CS1 = 0;   // Latch the data into memory using the CS signal.
    Nop();
    SRAM_CS1 = 1;
}

unsigned int sram_read(void){
    
}


void sram_IO_state(int state){
    TRISAbits.TRISA0    = state;
    TRISAbits.TRISA1    = state;   
    TRISAbits.TRISA2    = state;   
    TRISAbits.TRISA3    = state;   
    TRISAbits.TRISA4    = state;   
    TRISBbits.TRISB5    = state;
    TRISAbits.TRISA6    = state;   
    TRISAbits.TRISA7    = state;   
    TRISBbits.TRISB8    = state;   
    TRISBbits.TRISB9    = state;   
    TRISBbits.TRISB14   = state;  
    TRISBbits.TRISB15   = state;  
}