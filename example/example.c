 /*
 * USART3 transmit is on DMA1 Stream3 Channel 4.
 * See Reference manual Table 28 RM0390 July 2017.
 *
 * USART3 Tx signal is connected to pin PD8. To see the output of USART3
 * on a PC, you need to use a USB-serial module. Connect the Rx pin of
 * the module to the PB11 pin of the STM32F407 Discovery board. Make sure
 * the USB-serial module you use has a 3.3V interface.
 *
 * By default, the system clock is running at 16 MHz.
 * The UART is configured for 9600 Baud.
 * PD8 - USART3 TX (AF7)
 *
 * This program was tested with Keil uVision v5.24a with DFP v2.11.0
 */
#include <inttypes.h>
#include <stdbool.h>
#include "stm32f4xx.h"

#define FREQ 16000000  // CPU frequency, 16 Mhz
#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};

#define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank)))

// Enum values are per datasheet: 0, 1, 2, 3
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };



static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                      // Pin number
  RCC->AHB1ENR |= BIT(PINBANK(pin));       // Enable GPIO clock
  gpio->MODER &= ~(3U << (n * 2));         // Clear existing setting
  gpio->MODER |= (mode & 3U) << (n * 2);   // Set new mode
}

static inline void gpio_set_af(uint16_t pin, uint8_t af_num) {
  struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                      // Pin number
  gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4));
  gpio->AFR[n >> 3] |= ((uint32_t) af_num) << ((n & 7) * 4);
}

static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio->BSRR |= (1U << PINNO(pin)) << (val ? 0 : 16);
}

// t: expiration time, prd: period, now: current time. Return true if expired
static inline bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}


void USART3_init(unsigned long baud);
void DMA1_init(void);
void DMA1_Stream3_setup(unsigned int src, unsigned int dst, int len);

int done = 1;

void _init(void) {;}

static volatile uint32_t s_ticks;
void SysTick_Handler(void) {
  s_ticks++;
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


int main (void) {
    uint16_t led1 = PIN('D', 15);            // Blue LED
    gpio_set_mode(led1, GPIO_MODE_OUTPUT);  // Set blue LED to output mode

    systick_init(FREQ / 1000);             // Tick every 1 ms    
    USART3_init(115200);
    DMA1_init();

#if 1
    char alphabets[] = "abcdefghijklmnopqrstuvwxyz";
    char message[80];
    int i;
    int size = sizeof(alphabets);
        /* prepare the message for transfer */
        for (i = 0; i < size; i++)
            message[i] = alphabets[i];
        
        /* send the message out by USART3 using DMA */
        while (done == 0) {}    /* wait until DMA data transfer is done */
        done = 0;               /* clear done flag */
        DMA1_Stream3_setup((unsigned int)message, (unsigned int)&USART3->DR, size);
#endif
    
    uint32_t timer = 0, period = 2000;      // Declare timer and 1000ms period
    while (1) {
        if (timer_expired(&timer, period, s_ticks)) {
            static bool on;       // This block is executed
            gpio_write(led1, on);  // Every `period` milliseconds
            on = !on;             // Toggle LED state
        }
    }
}
/*  Initialize UART pins, Baudrate
    The USART3 is configured to send output to pin PD8 at 115200 Baud
 */
void USART3_init (unsigned long baud) {
    uint8_t af = 0;           // Alternate function
    uint16_t rx = 0, tx = 0;  // pins

    af = 7; tx = PIN('D', 8); rx = PIN('B', 11); // UART3
    gpio_set_mode(tx, GPIO_MODE_AF);
    gpio_set_af(tx, af);
    gpio_set_mode(rx, GPIO_MODE_AF);
    gpio_set_af(rx, af);


    //RCC->AHB1ENR |= 8;     /* it will be done above in gpio_set_mode enable GPIOD clock */
    RCC->APB1ENR |= BIT(18); // enable USART3 clock, bit 18 on APB1EN

    /* Configure PD8 for USART3_TX */
    //GPIOD->AFR[0] &= ~0x0F00;
    //GPIOD->AFR[0] |=  0x0700;   /* alt7 for USART3 */
    //GPIOD->MODER  &= ~0x0030;
    //GPIOD->MODER  |=  0x0020;   /* enable alternate function for PD8 */

    //USART3->BRR = 0x0683;       /* 9600 baud @ 16 MHz */
    USART3->BRR = FREQ / baud;                 // FREQ is a CPU frequency
    //USART3->BRR = baud;                 // FREQ is a CPU frequency
    USART3->CR1 = 0x0008;       /* enable Tx, 8-bit data */
    USART3->CR2 = 0x0000;       /* 1 stop bit */
    USART3->CR3 = 0x0000;       /* no flow control */
    USART3->CR1 |= 0x2000;      /* enable USART3 */
      
    USART3->SR = ~0x40;         /* clear TC flag */
    USART3->CR1 |= 0x0040;      /* enable transmit complete interrupt */

    NVIC_EnableIRQ(USART3_IRQn);    /* USART3 interrupt enable at NVIC */
}


/*  Initialize DMA1 controller
 *  DMA1 controller's clock is enabled and also the DMA interrupt is 
 *  enabled in NVIC.
 */
void DMA1_init(void) {
    RCC->AHB1ENR |= 0x00200000;     /* DMA controller clock enable */
    DMA1->LIFCR = 0x0f400000;       /* clear all interrupt flags of Stream 3 */
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);  /* DMA interrupt enable at NVIC */
}

/*  Set up a DMA transfer for USART 3
 *  The USART3 is connected to DMA1 Stream 2. This function sets up the 
 *  peripheral register address, memory address, number of transfers,
 *  data size, transfer direction, and DMA interrupts are enabled.
 *  At the end, the DMA controller is enabled and the USART3 transmit
 *  DMA is enabled.
 */
#define DMA_Mode_Circular                 ((uint32_t)0x00000100)

void DMA1_Stream3_setup(unsigned int src, unsigned int dst, int len) {
    DMA1_Stream3->CR &= ~1;         /* disable DMA1 Stream 3 */
    while (DMA1_Stream3->CR & 1) {} /* wait until DMA1 Stream 3 is disabled */
    DMA1->LIFCR = 0x0f400000;       /* clear all interrupt flags of Stream 3 */
    DMA1_Stream3->PAR = dst;
    DMA1_Stream3->M0AR = src;
    DMA1_Stream3->NDTR = len;
    DMA1_Stream3->CR = 0x08000000;  /* USART3_TX on DMA1 Stream3 Channel 4 */
    DMA1_Stream3->CR |= 0x00000440; /* data size byte, mem incr, mem-to-peripheral */
    DMA1_Stream3->CR |= 0x16;       /* enable interrupts DMA_IT_TC | DMA_IT_TE | DMA_IT_DME */
    DMA1_Stream3->FCR  = 0;         /* direct mode, no FIFO */
    DMA1_Stream3->CR |= DMA_Mode_Circular;
    DMA1_Stream3->CR |= 1;          /* enable DMA1 Stream 3 */

    USART3->SR &= ~0x0040;          /* clear UART transmit complete interrupt flag */
    USART3->CR3 |= 0x80;            /* enable USART3 transmitter DMA */
}

/*  DMA1 Stream3 interrupt handler
    This function handles the interrupts from DMA1 controller Stream6. The error interrupts 
    have a placeholder for error handling code. If the interrupt is from DMA data
    transfer complete, the DMA controller is disabled, the interrupt flags are
    cleared.
 */
void DMA1_Stream3_IRQHandler(void)
{
    if (DMA1->HISR & 0x000C0000)    /* if an error occurred */
        while(1) {}                 /* substitute this by error handling */
    DMA1->LIFCR = 0x0f400000;       /* clear all interrupt flags of Stream 3 */
    DMA1_Stream3->CR &= ~0x10;      /* disable DMA1 Stream 3 TCIE */
}

/*  USART3 interrupt handler
 *  USART3 transmit complete interrupt is used to set the done flag to signal
 *  the other part of the program that the data transfer is done.
 */
void USART3_IRQHandler(void)
{
    USART3->SR &= ~0x0040;          /* clear transmit complete interrupt flag */
    done = 1;                       /* set the done flag */
}
