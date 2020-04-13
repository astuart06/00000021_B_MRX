// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef RECEIVER_MISC_H
#define	RECEIVER_MISC_H

#include "p24F16KA302.h"
#include "xc.h"

/*******************************************************************************
* DEFINITIONS
*******************************************************************************/

/*******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void slave_id_init(void);
void slave_id_handler(void);

#endif // RECEIVER_MISC_H