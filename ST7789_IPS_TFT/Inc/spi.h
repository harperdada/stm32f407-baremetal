/*
 * spi.h
 *
 *  Created on: Sep 20, 2022
 *      Author: hussamaldean
 */

#ifndef SPI_H_
#define SPI_H_

#include "stdint.h"
#include "stm32f4xx.h"


#define LCD_RST1 GPIOA->BSRR=GPIO_BSRR_BS0
#define LCD_RST0 GPIOA->BSRR=GPIO_BSRR_BR0

#define	LCD_DC1	GPIOA->BSRR=GPIO_BSRR_BS1
#define	LCD_DC0 GPIOA->BSRR=GPIO_BSRR_BR1



void st7789_spi_init();
void st7789_spi_transmit(uint8_t *data,uint32_t size);


#endif /* SPI_H_ */
