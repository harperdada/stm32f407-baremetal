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
#include "gpio.h"

#define FREQ 16000000  // CPU frequency, 16 Mhz

/*************************************************
* function declarations
*************************************************/
void _init(void) {;}

int main(void);

volatile uint8_t msg[] = "Dude! I am interrupt driven.\n";

volatile int bufpos = 0;

void USART3_IRQHandler(void)
{   
    // check if the source is transmit interrupt
    // waiting for the "transmit buffer empty" bit to be set.
    // USART_IT_TXE : specifies the interrupt source for the Tx buffer empty 
    //                   interrupt. 
    // #define USART_FLAG_TXE                       ((uint16_t)0x0080)
   // This is bit number 7 and 1 << 7 is equal to 0x0080
   // another way to write it: while(!(USART3->SR & 0x0080)) {}
    if (USART3->SR & (1 << 7)) {
        
        if (bufpos == sizeof(msg)) {
            // buffer is flushed out, disable tx interrupt
            bufpos = 0; 
            USART3->CR1 &= ~(1U << 7);
        }
        else {
            // flush ot the next char in the buffer
            USART3->DR = msg[bufpos++];
        }
    }
}

static volatile uint32_t s_ticks;

void SysTick_Handler(void) {
   s_ticks++;
}

// t: expiration time, prd: period, now: current time. Return true if expired
static inline bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}

/*************************************************
* initialize SysTick
*************************************************/
static inline void systick_init(uint32_t s)
{   
    // Clear CTRL register
    SysTick->CTRL = 0x00000;
    // Main clock source is running with HSI by default which is at 8 Mhz.
    // SysTick clock source can be set with CTRL register (Bit 2)
    // 0: Processor clock/8 (AHB/8)
    // 1: Processor clock (AHB)
    SysTick->CTRL |= (1 << 2);
    // Enable callback (bit 1)
    SysTick->CTRL |= (1 << 1);
    // Load the value
    SysTick->LOAD = (uint32_t)(s-1);
    // Set the current value to 0
    SysTick->VAL = 0;
    // Enable SysTick (bit 0)
    SysTick->CTRL |= (1 << 0);
    //RCC->APB1ENR |= 1<<28;
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    uint16_t led1 = PIN('D', 15);            // Blue LED
    /* set system clock to 168 Mhz */
    set_sysclk_to_168();

    systick_init(FREQ / 1000);             // Tick every 1 ms

    gpio_set_mode(led1, GPIO_MODE_OUTPUT);

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

    // enable usart3 - UE, bit 13
    USART3->CR1 |= (1 << 13);

    NVIC_SetPriority(USART3_IRQn, 1); // Priority level 1
    NVIC_EnableIRQ(USART3_IRQn);

    // now that everything is ready,
    // enable tx interrupt and let it push out
    USART3->CR1 |= (1 << 7);

    uint32_t timer = 0, period = 4000;      // Declare timer and 1000ms period
    while(1)
    {
#if 0
        for (uint32_t i=0; i<sizeof(msg); i++){
            // send character
            USART3->DR = msg[i];
            // wait for transmit complete
            while(!(USART3->SR & (1 << 6)));

        }
#endif
     if (timer_expired(&timer, period, s_ticks)) {
         static bool on;       // This block is executed
         gpio_write(led1, on);  // Every `period` milliseconds
         on = !on;             // Toggle LED state
     }
        // slow down
        for(int i=0; i<10000000; i++); // a long wait. should be more than enough the push out the buffer
       // restart transmission by enabling usart3 tx interrupt
        printf("ticks: %lu\n", s_ticks);
        USART3->CR1 |= (1 << 7);
    }

    return 0;
}
