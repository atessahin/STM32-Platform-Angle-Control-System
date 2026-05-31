#include "I2C_driver.h"
I2C_Status i2cWrite(uint8_t deviceAddr ,uint8_t regAddr,uint8_t* data,uint16_t length)
{
    volatile uint32_t timeout;
    timeout=50000;
	//check busy
    while(I2C1->SR2 & (1<<1))
    {
        if(--timeout == 0) return I2C_ERROR_BUSY ;
    }

    I2C1->CR1 |= (1 << 8);

    timeout=50000;
   //wait  start bit
    while(!(I2C1->SR1 & (1<<0)))
    {
        if(--timeout == 0) return I2C_ERROR_TIMEOUT ;
    }

    //send slave addr <<1 for write mode
    I2C1->DR = deviceAddr << 1;

    timeout=50000;
    //check addr sent
    while(!(I2C1->SR1 & (1<<1)))
    {
    	//check ack failure
    	if (I2C1->SR1 & (1 << 10))
    	{
    		//af clear
    	   I2C1->SR1 &= ~(1 << 10);
    	   //send stop
    	   I2C1->CR1 |= (1 << 9);
    	   return I2C_ERROR_AF_OR_ADDRESS_MISMATCHED_OR_NOT_RECEIVED;
    	}


        if(--timeout == 0) return I2C_ERROR_TIMEOUT;
    }

    //we have to read this SRs data sheet saying like this
    volatile uint8_t temp;
    temp=I2C1->SR1;
    temp=I2C1->SR2;

    I2C1->DR = regAddr;
    timeout=50000;
    //check TxE empty
    while(!(I2C1->SR1 & (1<<7)))
    {
        if(--timeout == 0) return I2C_DATA_REG_NOT_EMPTY;
    }
    //write section
    for(uint16_t i=0;i<length;i++)
    {
    	I2C1->DR=data[i];
    	timeout=50000;
        while(!(I2C1->SR1 & (1<<7)))
        {
            if(--timeout == 0) return I2C_DATA_REG_NOT_EMPTY;
        }
    }
    timeout=50000;
    //check byte transfer finish
    while(!(I2C1->SR1 & (1<<2)))
    {
        if(--timeout == 0) return I2C_BYTE_TRANSFER_NOT_FINISH;
    }


    //stop bit
    I2C1->CR1 |= (1 << 9);

    return I2C_OK;
}
I2C_Status i2cRead(uint8_t deviceAddr ,uint8_t regAddr,uint8_t* dataBuffer,uint16_t length)
{
    volatile uint32_t timeout;
    timeout=50000;
	//check busy
    while(I2C1->SR2 & (1<<1))
    {
        if(--timeout == 0) return I2C_ERROR_BUSY ;
    }

    I2C1->CR1 |= (1 << 8);

    timeout=50000;
   //wait  start bit
    while(!(I2C1->SR1 & (1<<0)))
    {
        if(--timeout == 0) return I2C_ERROR_TIMEOUT ;
    }

    //send slave addr <<1 for write mode
    I2C1->DR = deviceAddr << 1;

    timeout=50000;
    //check addr sent
    while(!(I2C1->SR1 & (1<<1)))
    {
    	//check ack failure
    	if (I2C1->SR1 & (1 << 10))
    	{
    		//af clear
    	   I2C1->SR1 &= ~(1 << 10);
    	   //send stop
    	   I2C1->CR1 |= (1 << 9);
    	   return I2C_ERROR_AF_OR_ADDRESS_MISMATCHED_OR_NOT_RECEIVED;
    	}


        if(--timeout == 0) return I2C_ERROR_TIMEOUT;
    }

    //we have to read this SRs data sheet saying like this
    volatile uint8_t temp;
    temp=I2C1->SR1;
    temp=I2C1->SR2;

    I2C1->DR = regAddr;
    timeout=50000;
    //check TxE empty
    while(!(I2C1->SR1 & (1<<7)))
    {
        if(--timeout == 0) return I2C_DATA_REG_NOT_EMPTY;
    }


    //-----------------------
    //re-start section
    //-----------------------

    I2C1->CR1 |= (1 << 8);

    timeout=50000;
   //wait  start bit
    while(!(I2C1->SR1 & (1<<0)))
    {
        if(--timeout == 0) return I2C_ERROR_TIMEOUT ;
    }

    //send slave addr << 1|1; for read mode
    I2C1->DR = deviceAddr << 1|1;

    timeout=50000;
    //check addr sent
    while(!(I2C1->SR1 & (1<<1)))
    {
    	//check ack failure
    	if (I2C1->SR1 & (1 << 10))
    	{
    		//af clear
    	   I2C1->SR1 &= ~(1 << 10);
    	   //send stop
    	   I2C1->CR1 |= (1 << 9);
    	   return I2C_ERROR_AF_OR_ADDRESS_MISMATCHED_OR_NOT_RECEIVED;
    	}


        if(--timeout == 0) return I2C_ERROR_TIMEOUT;
    }

    //we have to read this SRs data sheet saying like this
    temp = I2C1->SR1;
    temp = I2C1->SR2;


	for(uint16_t i=0;i<length;i++)
	{
		/*dma fonk*/
		if(length-1==i)
		{
			//send nack
			I2C1->CR1 &= ~(1 << 10);
			I2C1->CR1 |= (1 << 9);
		}
		else
		{
			//keep send ack
			I2C1->CR1 |=(1 << 10);
		}

		timeout=50000;
		while(!(I2C1->SR1 & (1<<6)))
		{
			if(--timeout == 0) return I2C_DATA_REG_NOT_EMPTY;
		}

		dataBuffer[i]=I2C1->DR;

	}


    return I2C_OK;
}
