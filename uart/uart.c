/*
 * uart.c
 *
 * description: STM32F407 example uart project
 *   UART3 example without any interrupts
 *   uses USART3 PD8/PB11 pins to transmit and receive data
 *   connect a Serial to USB adapter to see the
 *   data on PC
 *
 * setup:
 *   1. enable usart clock from RCC
 *   2. enable gpioD clock
 *   3. set PD8 and PB11 as af7
 *   4. set uart word length and parity
 *   5. enable transmit and receive (TE/RE bits)
 *   6. calculate baud rate and set BRR
 *   7. enable uart
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

/*************************************************
* function declarations
*************************************************/
int main(void);

volatile uint8_t msg[] = "I see what you mean, bro!\n";

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    /* set system clock to 168 Mhz */
    set_sysclk_to_168();

    // enable USART3 clock, bit 18 on APB1ENR
    RCC->APB1ENR |= (1 << 18);

    // enable GPIOD clock, bit 3 on AHB1ENR
    RCC->AHB1ENR |= (1 << 3);

    // set pin modes as alternate mode 7 (pins 2 and 3)
    // USART3 TX and RX pins are PD8 and PB11 respectively
    //GPIOD->MODER &= ~(0xFU << 4); // Reset bits 4:5 for PD8 and 6:7 for PB11

    GPIOD->MODER &= ~(3U << (8 * 2));
    GPIOD->MODER |= (2U) << (8 * 2);
    //GPIOD->MODER |=  (0xAU << 4); // Set   bits 16,17 for PD8 and 23:24 for PB11 to alternate mode (10)

    GPIOB->MODER &= ~(3U << (11 * 2));
    GPIOB->MODER |= (2U) << (11 * 2);

    // set pin modes as high speed
    //GPIOD->OSPEEDR |= 0x000000A0; // Set pin 2/3 to high speed mode (0b10)
    GPIOD->OSPEEDR |= 0x00020000; // Set pin 8 to high speed mode (0b10)
    GPIOB->OSPEEDR |= 0x00800000; // Set pin 11 to high speed mode (0b10)

    // choose AF7 for USART3 in Alternate Function registers
    GPIOD->AFR[1] |= (0x7 << 0); // for pin D8
    GPIOB->AFR[1] |= (0x7 << 12); // for pin B11

    // USART2 word length M, bit 12
    //USART2->CR1 |= (0 << 12); // 0 - 1,8,n

    // USART2 parity control, bit 9
    //USART2->CR1 |= (0 << 9); // 0 - no parity

    // USART2 TX enable, TE bit 3
    USART3->CR1 |= (1 << 3);

    // USART2 rx enable, RE bit 2
    USART3->CR1 |= (1 << 2);

    // baud rate = fCK / (8 * (2 - OVER8) * USARTDIV)
    //   for fCK = 42 Mhz, baud = 115200, OVER8 = 0
    //   USARTDIV = 42Mhz / 115200 / 16
    //   = 22.7864 22.8125
    // we can also look at the table in RM0090
    //   for 42 Mhz PCLK, OVER8 = 0 and 115.2 KBps baud
    //   we need to program 22.8125
    // Fraction : 16*0.8125 = 13 (multiply fraction with 16)
    // Mantissa : 22
    // 12-bit mantissa and 4-bit fraction
    USART3->BRR |= (22 << 4);
    USART3->BRR |= 13;

    // enable usart2 - UE, bit 13
    USART3->CR1 |= (1 << 13);

    while(1)
    {
        for (uint32_t i=0; i<sizeof(msg); i++){
            // send character
            USART3->DR = msg[i];
            // wait for transmit complete
            while(!(USART3->SR & (1 << 6)));

        }
        // slow down
        for(int i=0; i<10000000; i++);
    }

    return 0;
}
