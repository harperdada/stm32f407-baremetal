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

#include "delay.h"
volatile uint64_t x;


void delayuS(int ms)
{

	SysTick->LOAD=100-1;
	SysTick->VAL=0;
	SysTick->CTRL=0x5;
		for (int i=0;i<ms;i++)
		{
			while(!(SysTick->CTRL &0x10000)){}
		}
	SysTick->CTRL=0;	
	

			 
	}

void delay(int ms)
{

	SysTick->LOAD=100000-1;
	SysTick->VAL=0;
	SysTick->CTRL=0x5;
		for (int i=0;i<ms;i++)
		{
			while(!(SysTick->CTRL &0x10000)){}
		}
	SysTick->CTRL=0;	

}
