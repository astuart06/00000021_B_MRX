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

void slave_id_init(void){
    next_state = ST_SLAVE_ID;
}
void slave_id_handler(void){
    unsigned char data_buffer[8];
    
    data_buffer[0] = 'V';
    data_buffer[1] = '2';
    data_buffer[2] = '.';
    data_buffer[3] = '2';
    data_buffer[4] = '.';
    data_buffer[5] = '5';
    data_buffer[6] = 'r';
    data_buffer[7] = 'c';
            
   spi_tx_wait_init(data_buffer, 8);
}