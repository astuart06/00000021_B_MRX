/******************************************************************************
 * File:   checksum.c
 * Author: Andrew Stuart
 *
 ******************************************************************************/

/*******************************************************************************
* INCLUDES
*******************************************************************************/
#include "checksum.h"
#include "p24F16KA302.h"

#include "stdio.h"
#include <stdlib.h>
#include "xc.h"

#include "globals.h"
#include "protocol.h"

unsigned int checksum(char *data, int size, int m){
    int i;
    unsigned int a, b;

    a = *data;                  // Initial value for a.

    for(i = 1; i < size; i++){
        b = *(data + i);
        if(b != 0){             // if b = 0 the a will not change.            
            b = m - b;
            if ( a>=b ){
                a = a - b;
            }
            else{
                a = m - b + a;
            }
        }
    }
    return ~a & 0xff;
}
