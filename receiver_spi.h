// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef RECEIVER_SPI_H
#define	RECEIVER_SPI_H

#include "p24F16KA302.h"


/*******************************************************************************
* DEFINITIONS
*******************************************************************************/
#define SPI_CS  PORTBbits.RB4

/*******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void peripheral_spi_init(void);

void spi_transfer(unsigned char *tx_data, unsigned char *rx_data, unsigned int size_bytes);

unsigned char spi_rx_wait(void);

void spi_tx_wait_init(unsigned char * data_buffer, int length);
void spi_tx_wait_handler(void);

void spi_msg_tx_init(void);
void spi_msg_tx_handler(void);

void spi_fill_tx_buffer(int value);
void spi_tx_buffer_write(char *tx_data, unsigned int size_bytes);

#endif // RECEIVER_SPI_H

