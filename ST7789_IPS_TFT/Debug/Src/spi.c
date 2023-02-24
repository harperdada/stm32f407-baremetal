/*
 * spi.c
 *
 *  Created on: Sep 20, 2022
 *      Author: hussamaldean
 */


#include "spi.h"
#include "stm32f4xx.h"

#define AF05 0x05


void st7789_spi_init()
{

	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;
	/*SPI pins*/
	GPIOA->MODER|=GPIO_MODER_MODE5_1|GPIO_MODER_MODE6_1|GPIO_MODER_MODE7_1;
	GPIOA->MODER&=~(GPIO_MODER_MODE5_0|GPIO_MODER_MODE6_0|GPIO_MODER_MODE7_0);
	GPIOA->AFR[0]|=(AF05<<GPIO_AFRL_AFSEL5_Pos)|(AF05<<GPIO_AFRL_AFSEL6_Pos)|(AF05<<GPIO_AFRL_AFSEL7_Pos);

	/*DC and RST pins*/
	GPIOA->MODER|=GPIO_MODER_MODE0_0|GPIO_MODER_MODE1_0;
	GPIOA->MODER&=~(GPIO_MODER_MODE0_1|GPIO_MODER_MODE1_1);

	/*SPI configuration*/

	RCC->APB2ENR|=RCC_APB2ENR_SPI1EN;
	SPI1->CR1|=SPI_CR1_CPHA|SPI_CR1_CPOL|SPI_CR1_SSM|SPI_CR1_BR_0|SPI_CR1_SSI|SPI_CR1_MSTR|SPI_CR1_SPE;
}

void st7789_spi_transmit(uint8_t *data,uint32_t size)
{
	uint32_t i=0;
	uint8_t temp;

	while(i<size)
	{
		/*Wait until TXE is set*/
		while(!(SPI1->SR & (SPI_SR_TXE))){}

		/*Write the data to the data register*/
		SPI1->DR=data[i];
		//*spidr = data[i];
		i++;
	}
	/*Wait until TXE is set*/
	while(!(SPI1->SR & (SPI_SR_TXE))){}

	/*Wait for BUSY flag to reset*/
	while((SPI1->SR & (SPI_SR_BSY))){}

	/*Clear OVR flag*/
	temp = SPI1->DR;
	temp = SPI1->SR;
}
