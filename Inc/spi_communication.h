#ifndef SPI_COMMUNICATION_H
#define SPI_COMMUNICATION_H

#include "stm32f4xx.h"

void SPI2_Transfer(uint16_t data);
void MAX7219_Write(uint8_t address, uint8_t value);
void MAX7219_Init(void);
void Display_Angle(int16_t angle);

/* MPU açısını göstermek için yeni eklenen fonksiyon prototipi */
void Display_MPU_Angle(int16_t mpu_angle);

#endif /* SPI_COMMUNICATION_H */
