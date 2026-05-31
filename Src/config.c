#include "stm32f4xx.h"


void gpioConfig()
{
	RCC->AHB1ENR |= (1 << 1);// Enable GPIOB
	RCC->AHB1ENR |= (1<<0); // Enable GPIOA
	RCC->AHB1ENR |= (1 << 2); // enable gpioc clock
	// gpio for i2c (pb6-scl/pb7-sda)
	//set altarneta function
	GPIOB->MODER|=(2<<12);
	GPIOB->MODER|=(2<<14);
	//set open drain cause i2c have to work with open drain
	GPIOB->OTYPER|=(1<<6);
	GPIOB->OTYPER|=(1<<7);
	//fast speed
	GPIOB->OSPEEDR|=(2<<12);
	GPIOB->OSPEEDR|=(2<<14);
	//set pull-up
	GPIOB->PUPDR|=(1<<12);
	GPIOB->PUPDR|=(1<<14);
	//af func AF4
	GPIOB->AFR[0]|=(4<<24);
	GPIOB->AFR[0]|=(4<<28);



	// gpio for spi2 (pb13-sck / pb15-mosi / pb12-cs)
	//set altarneta function
	GPIOB->MODER|=(2<<26);
	GPIOB->MODER|=(2<<30);
	//fast speed
	GPIOB->OSPEEDR|=(2<<26);
	GPIOB->OSPEEDR|=(2<<30);
	//af func AF5
	GPIOB->AFR[1]|=(5<<20);
	GPIOB->AFR[1]|=(5<<28);

	//cs high
	GPIOB->MODER |=(1 << 24);
	GPIOB->BSRR=(1 << 12);

	//keypad


	// rows (pc0, pc1, pc2, pc3) output (01)
	GPIOC->MODER &= ~((3<<0) | (3<<2) | (3<<4) | (3<<6));
	GPIOC->MODER |= ((1<<0) | (1<<2) | (1<<4) | (1<<6));

	// cols (pc4, pc5, pc6, pc7) input (00)
	GPIOC->MODER &= ~((3<<8) | (3<<10) | (3<<12) | (3<<14));

	// pull-up for cols (01)
	GPIOC->PUPDR &= ~((3<<8) | (3<<10) | (3<<12) | (3<<14));
	GPIOC->PUPDR |= ((1<<8) | (1<<10) | (1<<12) | (1<<14));

	// all rows high initially
	GPIOC->BSRR = (1<<0) | (1<<1) | (1<<2) | (1<<3);

	//for motor timer config
	//pb4(ch1)-pb5(ch2)
	//set altarnate func
	GPIOB->MODER|=(2<<8);
	GPIOB->MODER|=(2<<10);
	//fast speed
	GPIOB->OSPEEDR|=(2<<8);
	GPIOB->OSPEEDR|=(2<<10);
	//af func AF5
	GPIOB->AFR[0]|=(2<<16);
	GPIOB->AFR[0]|=(2<<20);
}

void i2cConfig()
{
	//enable i2c1
	RCC->APB1ENR|=(1<<21);

	//Software Reset
	I2C1->CR1 |= (1<<15);
	//delay gelcek
	I2C1->CR1 &= ~(1<<15);
	//i2c disable
	I2C1->CR1&= ~(1<<0);
	//for 100 khz
	/*
	 APB1=42MHZ
	 CCR=fPCLK1 / 2*fSCL
	 CCR=42*10^6/2*100.000
	 CCR=210​

	TRISE=​(fPCLK1/1.000.000)+1
	(42*10^6/1*10^6)+1=43​
	 */
	I2C1->CCR=210;
	I2C1->CR2=42;
	I2C1->TRISE = 43;

	//analog filter enable
	I2C1->FLTR&= ~(1<<4);
	//ack enable
	I2C1->CR1|=(1<<10);
	//enable
	I2C1->CR1|=(1<<0);
}

/*dma 1 stream 0 channel 1*/
void dmaConfig()
{
	//dma1 enable
	RCC->AHB1ENR|=(1<<21);
	//disable dma
	DMA1_Stream4->CR&= ~(1<<0);
	//direction perhipal->mem
	DMA1_Stream4->CR &= ~(3 << 6);
	//cinc disable
	DMA1_Stream4->CR&= ~(1<<8);
	//pinc disable
	DMA1_Stream4->CR&= ~(1<<9);
	//minc enable
	DMA1_Stream4->CR|=(1<<10);
	//transfer complete interrupt enable
	DMA1_Stream4->CR|=(1<<4);
	//perhip data size and mem data size
	DMA1_Stream4->CR&= ~(1<<11|1<<13);
	//priorty lvl high
	DMA1_Stream4->CR|= (2<<17);
	//chanel select 1
	DMA1_Stream4->CR&= ~(7<<25);
	//perhipal addr
	DMA1_Stream4->PAR=(uint32_t)&(SPI2->DR);

	NVIC_SetPriority(DMA1_Stream4_IRQn, 3);
	NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

void systemClockConfig()
{
	RCC->CR |=(1<<16);
	while(!(RCC->CR & (1<<17)));

	FLASH->ACR|=(3<<1)|(1<<8)|(1<<9)|(1<<10);

	RCC->APB1ENR|=(1<<28);
	PWR->CR |=(3U<<14);


	RCC->CR &=~(1<<24);
	while(RCC->CR & (1<<25))
		;

	//PLL: HSE/M * N/P = 8MHz/8 * 168/2 = 84mhz
	RCC->PLLCFGR = (8 << 0) |       // PLLM = 8
                   (168 << 6) |     // PLLN = 168
                   (0 << 16) |      // PLLP = 2
                   (1 << 22);       // PLLSRC = HSE


	RCC->CR |= (1 << 24);           // PLL ON
	while(!(RCC->CR & (1 << 25)));  // Wait until PLL is ready


	RCC->CFGR&=~(15U<<4);           // AHB prescaler = 1, 84MHz
	RCC->CFGR|=(4U<<10);            // APB1 prescaler = 2, 84MHz/2 = 42MHz (max)
	RCC->CFGR&=~(7U<<13);           // APB2 prescaler = 1, 84MHz


	// Select PLL as system clock
	RCC->CFGR |= (2 << 0);          // SW = PLL
	while((RCC->CFGR & (3 << 2)) != (2 << 2))  // Wait until PLL is system clock
		;

	SystemCoreClockUpdate();
}
void spiConfig()
{
	//spi1 clock enable
	RCC->APB1ENR|=(1<<14);
	//spi1 disable
	SPI2->CR1&= ~(1<<6);
	//baud rate control
	SPI2->CR1|= (3<<3);
	//data format 16bit
	SPI2->CR1|= (1<<11);
	//master selection
	SPI2->CR1|= (1<<2);
	//MSB format
	SPI2->CR1&= ~(1<<7);
	//software slave man and nss pind and ıo val of the nss pin ignore
	SPI2->CR1 |= (1<<8) | (1<<9);
	//cpol=0 cpha=0
	SPI2->CR1&= ~(1<<1);
	SPI2->CR1&= ~(1<<0);
	//start
	SPI2->CR1 |= (1 << 6);
}
void tim3Config()
{
	//tim3 enable
	RCC->APB1ENR|=(1<<1);
	//counter direction
	//TIM3->CR1|=(1<<4);
	//preseclar
	TIM3->PSC=0;
	//arr
	TIM3->ARR=4199;
	//clear chanel 1
	TIM3->CCMR1 &= ~(7 << 4);
	//select pwm mode for channel 1
	TIM3->CCMR1 |= (6U << 4);
	//preload Enable ch1
	TIM3->CCMR1 |= (1U << 3);
	//clear channel 2
	TIM3->CCMR1 &= ~(7 << 12);
	//select pwm mode for channel 2
	TIM3->CCMR1 |= (6U << 12);
	//preload Enable ch2
	TIM3->CCMR1 |= (1U << 11);
	//output enable for ch1 ch2
	TIM3->CCER |= (1 << 0) | (1 << 4);
	//arpe enable cause of glitch pwm
	TIM3->CR1|=(1<<7);
	//counter enable
	TIM3->CR1|=(1<<0);
}
