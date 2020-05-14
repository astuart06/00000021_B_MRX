/******************************************************************************
 * File:   receiver_misc.c
 * Author: Andrew Stuart
 *
 ******************************************************************************/

/*******************************************************************************
* INCLUDES
*******************************************************************************/
#include "receiver_misc.h"
#include "p24F16KA302.h"

#include "stdio.h"
#include <stdlib.h>
#include "xc.h"

#include "protocol.h"
#include "globals.h"

#include "receiver_spi.h"
#include "checksum.h"

void slave_id_init(void){
    next_state = ST_SLAVE_ID;
}
void slave_id_handler(void){
    unsigned char data_buffer[8];
    
    // Currently the version string must not be longer than 7 bytes.
    strcpy(data_buffer, 'V2.2.8d');
    data_buffer[7] = checksum(data_buffer, 7, 256);
            
   spi_tx_wait_init(data_buffer, 8);
}