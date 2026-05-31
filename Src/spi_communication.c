#include "spi_communication.h"
void SPI2_Transfer(uint16_t data)
{
	//check  Transmit buffer empty
    while(!(SPI2->SR & (1 << 1)));
    //send data
    SPI2->DR = data;
    //check busy flag
    while (SPI2->SR & (1 << 7));
}
