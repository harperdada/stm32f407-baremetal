#include "st7789.h"
#include "delay.h"
extern void SysClockConfig(void);


int main(void)
{

	SysClockConfig(); /*Change core frequency to 100MHz*/
	ST7789_Init();

	while(1)
		{
		ST7789_Test();
		delay(500);
		}
}
