#include "stm32f4xx.h"
#include "spi_communication.h"

/* cs pin control macros */
#define CS_LOW()  (GPIOB->BSRR = (1 << 28))
#define CS_HIGH() (GPIOB->BSRR = (1 << 12))

/* send command or data to max7219 */
void MAX7219_Write(uint8_t address, uint8_t value) {
    uint16_t package = (address << 8) | value;

    /* activate chip */
    CS_LOW();
    /* transfer data via spi */
    SPI2_Transfer(package);
    for(volatile int i=0; i<100; i++);
    /* deactivate chip */
    CS_HIGH();
}

/* initialize max7219 */
void MAX7219_Init(void) {
    /* set decode mode to code-b (Sayılar için en iyi mod) */
    MAX7219_Write(0x09, 0xFF);
    for(volatile int i=0; i<1000; i++);
    /* set brightness intensity */
    MAX7219_Write(0x0A, 0x04);
    /* set scan limit to all digits */
    MAX7219_Write(0x0B, 0x07);
    /* exit shutdown mode */
    MAX7219_Write(0x0C, 0x01);
    /* disable display test */
    MAX7219_Write(0x0F, 0x00);

    /* clear all digits (0x0F Code-B modunda boşluk demektir) */
    for(int i = 1; i <= 8; i++) {
        MAX7219_Write(i, 0x0F);
    }
}

/* show KEYPAD target angle value on RIGHT side (Digits 1-3) */
void Display_Angle(int16_t angle) {
    /* limit angle range */
    if (angle > 60) angle = 60;
    if (angle < -60) angle = -60;

    /* check sign and convert to positive */
    uint8_t isNegative = 0;
    if (angle < 0) {
        isNegative = 1;
        angle = -angle;
    }

    /* separate digits */
    uint8_t birler = angle % 10;
    uint8_t onlar = (angle / 10) % 10;

    /* write ones digit */
    MAX7219_Write(1, birler);

    /* handle tens digit or blank */
    if (onlar == 0 && angle < 10) {
         MAX7219_Write(2, 0x0F); // 0x0F = Boşluk
    } else {
         MAX7219_Write(2, onlar);
    }

    /* write negative sign or blank */
    if (isNegative) {
        MAX7219_Write(3, 0x0A); // 0x0A = Eksi (-) işareti
    } else {
        MAX7219_Write(3, 0x0F); // 0x0F = Boşluk
    }
}

/* show MPU current angle value on LEFT side (Digits 6-8) */
void Display_MPU_Angle(int16_t mpu_angle) {
    /* Limit angle range for 2 digits (Max 99, Min -99) */
    if (mpu_angle > 99) mpu_angle = 99;
    if (mpu_angle < -99) mpu_angle = -99;

    /* check sign and convert to positive */
    uint8_t isNegative = 0;
    if (mpu_angle < 0) {
        isNegative = 1;
        mpu_angle = -mpu_angle;
    }

    /* separate digits */
    uint8_t birler = mpu_angle % 10;
    uint8_t onlar = (mpu_angle / 10) % 10;

    /* write ones digit to Digit 6 */
    MAX7219_Write(6, birler);

    /* handle tens digit or blank to Digit 7 */
    if (onlar == 0 && mpu_angle < 10) {
         MAX7219_Write(7, 0x0F);
    } else {
         MAX7219_Write(7, onlar);
    }

    /* write negative sign or blank to Digit 8 (En sol hane) */
    if (isNegative) {
        MAX7219_Write(8, 0x0A);
    } else {
        MAX7219_Write(8, 0x0F);
    }
}
