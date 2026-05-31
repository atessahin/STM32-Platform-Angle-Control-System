#include "stm32f4xx.h"
#include "main.h"
#include "dma_communicaiton.h"
typedef enum {
    I2C_OK = 0,
    I2C_ERROR_BUSY,
    I2C_ERROR_TIMEOUT,
    I2C_ERROR_OVR,
    I2C_ERROR_AF_OR_ADDRESS_MISMATCHED_OR_NOT_RECEIVED,
    I2C_DATA_REG_NOT_EMPTY,
	I2C_BYTE_TRANSFER_NOT_FINISH
} I2C_Status;


I2C_Status i2cWrite(uint8_t deviceAddr ,uint8_t regAddr,uint8_t* data,uint16_t length);
I2C_Status i2cRead(uint8_t deviceAddr ,uint8_t regAddr,uint8_t* dataBuffer,uint16_t length);
