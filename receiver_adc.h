// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef RECEIVER_ADC_H
#define	RECEIVER_ADC_H

#include "p24F16KA302.h"
#include "xc.h"

/*******************************************************************************
* DEFINITIONS
*******************************************************************************/

/*******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void peripheral_adc_init(void);
void adc_en_handler(void);
int adc_read(void);

#endif // RECEIVER_ADC_H