#include "dma_communicaiton.h"
/*void dmaI2CTransfer(uint8_t* rxBuff,uint8_t size)
{
	//disable dma
	DMA1_Stream0->CR&= ~(1<<0);
	//wait dma untill disable
	while(DMA1_Stream0->CR & (1<<0));
	//dma1 Stream 0  LIFCR clear
	DMA1->LIFCR = (0x3D << 0);
	//dma mem address
	DMA1_Stream0->M0AR=(uint32_t)rxBuff;
	//number of data
	DMA1_Stream0->NDTR=(uint32_t)size;
	// DMA EN + LAST
	 I2C1->CR2 |= (1 << 11) | (1 << 12);
	//start dma
	DMA1_Stream0->CR|=(1<<0);
}*/
/*void dmaI2CTransfer(uint8_t* rxBuff,uint8_t size)
{
	//disable dma
	DMA1_Stream0->CR&= ~(1<<0);
	//wait dma untill disable
	while(DMA1_Stream0->CR & (1<<0));
	//dma1 Stream 0  LIFCR clear
	DMA1->LIFCR = (0x3D << 0);
	//dma mem address
	DMA1_Stream0->M0AR=(uint32_t)rxBuff;
	//number of data
	DMA1_Stream0->NDTR=(uint32_t)size;
	// DMA EN + LAST
	 I2C1->CR2 |= (1 << 11) | (1 << 12);
	//start dma
	DMA1_Stream0->CR|=(1<<0);
}*/
