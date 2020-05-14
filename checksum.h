// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef CHECKSUM_H
#define	CHECKSUM_H

#include "p24F16KA302.h"

/*******************************************************************************
* DEFINITIONS
*******************************************************************************/

/*******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
/******************************************************************************
* checksum
* 
* Description: 
*   
* 
* Inputs:
* None      
* Returns:
* None
 ******************************************************************************/
unsigned int checksum(char *data, int size, int m);

#endif // CHECKSUM_H