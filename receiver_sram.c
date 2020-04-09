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
#include "protocol.h"
#include "receiver_spi.h"

void sram_read_handler(void){
    int i;
    int count;
    unsigned int result;
    unsigned char data_buffer[USB_PACKET_MAX];
    
    count = spi_data_rx[USB_PACKET_RXBYTES];
    hardware_sram_init(SRAM_READ);
    SRAM_CS1 = 0;
    
    for(i = 0; i < (count / 2); i++){
        while(TRIGGER_ADC == 0);        // Wait for trigger before beginning read.
        SLAVE_STATE = SLAVE_ACTIVE;
        
        result = sram_read();
        data_buffer[2*i]        = result & 0xFF00;  // MSB
        data_buffer[(2*i)+1]    = result & 0x00FF;  // LSB
        
        SLAVE_STATE = SLAVE_IDLE;
    }
    SRAM_CS1 = 1;
    spi_tx_wait_init(data_buffer, count);
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
    Nop();
    Nop();
    
    TRISAbits.TRISA0    = sram_mode;
    TRISAbits.TRISA1    = sram_mode;   
    TRISAbits.TRISA2    = sram_mode;   
    TRISAbits.TRISA3    = sram_mode;   
    TRISAbits.TRISA4    = sram_mode;   
    TRISBbits.TRISB5    = sram_mode;
    TRISAbits.TRISA6    = sram_mode;   
    TRISAbits.TRISA7    = sram_mode;   
    TRISBbits.TRISB8    = sram_mode;   
    TRISBbits.TRISB9    = sram_mode;   
    TRISBbits.TRISB14   = sram_mode;  
    TRISBbits.TRISB15   = sram_mode;
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
    LATAbits.LATA0  = (data >> 0) & 1U;     // Bit 0 -  IO 0
    LATAbits.LATA1  = (data >> 1) & 1U;     // Bit 1 -  IO 1
    LATAbits.LATA2  = (data >> 2) & 1U;     // Bit 2 -  IO 2
    LATAbits.LATA3  = (data >> 3) & 1U;     // Bit 3 -  IO 3
    LATAbits.LATA4  = (data >> 4) & 1U;     // Bit 4 -  IO 4
    
    LATBbits.LATB5  = (data >> 5) & 1U;     // Bit 5 -  IO 5
    
    LATAbits.LATA6  = (data >> 6) & 1U;     // Bit 6 -  IO 6
    LATAbits.LATA7  = (data >> 7) & 1U;     // Bit 7 -  IO 7    
    
    LATBbits.LATB8  = (data >> 8) & 1U;     // Bit 8 -  IO 8
    LATBbits.LATB9  = (data >> 9) & 1U;     // Bit 9 -  IO 9
    
    LATBbits.LATB14 = (data >> 10) & 1U;    // Bit 10 - IO 10
    LATBbits.LATB15 = (data >> 11) & 1U;    // Bit 11 - IO 11
    
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    SRAM_CS1 = 0;   // Latch the data into memory using the CS signal.
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    SRAM_CS1 = 1;
}

unsigned int sram_read(void){
    unsigned long result, port_a_bits, port_b_bits;
    unsigned long mask_a;
    unsigned long mask_b_u, mask_b_l;
    
    mask_a      = 0x00DF;
    mask_b_u    = 0xC000;   // IO10/IO11 on RB14/RB15
    mask_b_l    = 0x0320;
    
    port_a_bits = (PORTA & mask_a);
    
    port_b_bits = (PORTB & mask_b_u) >> 4;
    port_b_bits |=(PORTB & mask_b_l);
    
    result = port_a_bits | port_b_bits;
    
    return result;
}