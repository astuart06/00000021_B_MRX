// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef RECEIVER_SRAM_H
#define	RECEIVER_SRAM_H

#include "p24F16KA302.h"


/*******************************************************************************
* DEFINITIONS
*******************************************************************************/
#define SRAM_CS1    PORTBbits.RB6

/*******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void sram_init(void);
void sram_write(unsigned int data);
unsigned int sram_read(void);


#endif // RECEIVER_SRAM_H