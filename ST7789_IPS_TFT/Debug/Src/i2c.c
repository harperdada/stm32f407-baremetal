/**

 * original author: Husamuldeen <https://github.com/hussamaldean>

   ----------------------------------------------------------------------
   	Copyright (C) husamuldeen, 2020

    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.
     
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
 */


#include "i2c.h"

uint8_t inited=0;

void i2c_init(void){
if(inited==0){
RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN; //enable gpiob clock
RCC->APB1ENR|=RCC_APB1ENR_I2C1EN; //enable i2c1 clock
GPIOB->MODER|=0xA0000; //set pb8and9 to alternative function
GPIOB->AFR[1]|=0x44;
GPIOB->OTYPER|=GPIO_OTYPER_OT8|GPIO_OTYPER_OT9; //set pb8 and pb9 as open drain
I2C1->CR1=I2C_CR1_SWRST;
I2C1->CR1&=~I2C_CR1_SWRST;	
I2C1->CR2|=50;
I2C1->CCR|=0x2|(1<<15)|(1<<14); 
I2C1->TRISE=20; //output max rise 
I2C1->CR1|=I2C_CR1_PE;
inited=1;
}
}
char i2c_readByte(char saddr,char maddr, char *data)
{

volatile int tmp;
while(I2C1->SR2&I2C_SR2_BUSY){;}
I2C1->CR1|=I2C_CR1_START;
while(!(I2C1->SR1&I2C_SR1_SB)){;}
I2C1->DR=saddr<<1;
while(!(I2C1->SR1&I2C_SR1_ADDR)){;}
tmp=I2C1->SR2;
while(!(I2C1->SR1&I2C_SR1_TXE)){;}
I2C1->DR=maddr;
while(!(I2C1->SR1&I2C_SR1_TXE)){;}
I2C1->CR1|=I2C_CR1_START;
while(!(I2C1->SR1&I2C_SR1_SB)){;}	
I2C1->DR=saddr<<1|1;
while(!(I2C1->SR1&I2C_SR1_ADDR)){;}
I2C1->CR1&=~I2C_CR1_ACK;
tmp =I2C1->SR2;
I2C1->CR1|=I2C_CR1_STOP;
while(!(I2C1->SR1&I2C_SR1_RXNE)){;}
*data++=I2C1->DR;
return 0;
}

void i2c_writeByte(char saddr,char maddr,char data){


volatile int Temp;
while(I2C1->SR2&I2C_SR2_BUSY){;}          /*wait until bus not busy*/
I2C1->CR1|=I2C_CR1_START;                 /*generate start*/
while(!(I2C1->SR1&I2C_SR1_SB)){;}         /*wait until start bit is set*/
I2C1->DR = saddr<< 1;                 	 /* Send slave address*/
while(!(I2C1->SR1&I2C_SR1_ADDR)){;}      /*wait until address flag is set*/
Temp = I2C1->SR2; 											 /*clear SR2 by reading it */
while(!(I2C1->SR1&I2C_SR1_TXE)){;}       /*Wait until Data register empty*/
I2C1->DR = maddr;                        /* send memory address*/
while(!(I2C1->SR1&I2C_SR1_TXE)){;}       /*wait until data register empty*/
I2C1->DR = data; 	
while (!(I2C1->SR1 & I2C_SR1_BTF));      /*wait until transfer finished*/
I2C1->CR1 |=I2C_CR1_STOP;								 /*Generate Stop*/	
	
}

void i2c_WriteMulti(char saddr,char maddr,char *buffer, uint8_t length){

while (I2C1->SR2 & I2C_SR2_BUSY);           //wait until bus not busy
I2C1->CR1 |= I2C_CR1_START;                   //generate start
while (!(I2C1->SR1 & I2C_SR1_SB)){;}					//wait until start is generated
volatile int Temp;														
I2C1->DR = saddr<< 1;                 	 			// Send slave address
while (!(I2C1->SR1 & I2C_SR1_ADDR)){;}        //wait until address flag is set
Temp = I2C1->SR2; 						      //Clear SR2
while (!(I2C1->SR1 & I2C_SR1_TXE));           //Wait until Data register empty
I2C1->DR = maddr;                      				// send memory address
while (!(I2C1->SR1 & I2C_SR1_TXE));           //wait until data register empty
//sending the data
for (uint8_t i=0;i<length;i++)
 { 
 I2C1->DR=buffer[i]; 													//filling buffer with command or data
	while (!(I2C1->SR1 & I2C_SR1_BTF));
 }	
                             
I2C1->CR1 |= I2C_CR1_STOP;										//wait until transfer finished
}


void i2c_ReadMulti(char saddr,char maddr, int n, char* data)
{
	volatile int temp;
	while (I2C1->SR2 & I2C_SR2_BUSY){;}
	I2C1->CR1|=I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){;}
	I2C1->DR=saddr<<1;
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){;}
	temp=I2C1->SR2;
	while(!(I2C1->SR1&I2C_SR1_TXE)){;}
	I2C1->DR = maddr;
	while(!(I2C1->SR1&I2C_SR1_TXE)){;}
	I2C1->CR1|=I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){;}
	I2C1->DR=saddr<<1|1;
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){;}
	temp=I2C1->SR2;
	I2C1->CR1|=I2C_CR1_ACK;
	while(n>0U)
		{
		if(n==1U)
				{
				I2C1->CR1&=~I2C_CR1_ACK;
					I2C1->CR1|=I2C_CR1_STOP;
					while(!(I2C1->SR1&I2C_SR1_RXNE)){;}
					*data++=I2C1->DR;
						break;
				}
			else
					{

					while(!(I2C1->SR1&I2C_SR1_RXNE)){;}
						(*data++)=I2C1->DR;
							n--;
				}
}
}
