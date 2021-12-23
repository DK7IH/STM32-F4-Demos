///////////////////////////////////////////////////////////////////  
//                    UART Demo for STM32F4                      //
//                    Transmitter and Receiver                   //
//                    Receiver on interrupt                      //
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         F411E Discovery board by STM               */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////

//This demo code transmits signals at PA2. The receiver is waiting 
//for signals at PA3. When PA2 and PA3 are connected the test 
//characters are received by the software. Each time a character 
//is dtected at the RX pin the correspnding interrup is fired.
//The red LED toggles each time this event occurs.

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <stdlib.h> 
#include <string.h> 

  ////////////
 //  MISC  //
////////////
int main(void);
uint8_t rxdata;
char *msg = (char*)"CQ CQ CQ DE DK7IH"; 
extern "C" void USART2_IRQHandler(void);

// USART2 interrupt handler
//Declare as 'extern "C"' when compiler in 
//c++ mode to get correct name of int function
extern "C" void USART2_IRQHandler(void)
{
	rxdata = USART2->DR;
    //Do something with received character 
    GPIOD->ODR ^= (1 << 13); //Toggle red LED
}

  ///////////////////
 // Main programm //
///////////////////
int main(void)
{
	uint16_t t1;
	 
    //Turn on the GPIOC peripheral for LED
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; //RCC->AHB1ENR |= (1<<0);
    GPIOD->MODER |= (1 << (13 << 1))|(1 << (15 << 1));	//red and blue
    GPIOD->ODR &= ~(1 << 13); //LED OFF
    GPIOD->ODR &= ~(1 << 15); //LED OFF
    
    //////////////////////////////////////////////
    // Set SystemClock to 48 MHz with 8 MHz HSE
    //////////////////////////////////////////////
    FLASH->ACR |= FLASH_ACR_LATENCY_1WS;        //1 wait state for 48 MHz
    RCC->CR |= RCC_CR_HSEON;                    //Activate external clock (HSE: 8 MHz)
    while ((RCC->CR & RCC_CR_HSERDY) == 0);     //Wait until HSE is ready
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;
    RCC->PLLCFGR |= 4 << RCC_PLLCFGR_PLLQ_Pos;  //PLL-Q: /4
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;     //PLL source is HSE
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;          //PLL-P: /2
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
    RCC->PLLCFGR |= 96 << RCC_PLLCFGR_PLLN_Pos; //PLL-N: x96
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
    RCC->PLLCFGR |= 4 << RCC_PLLCFGR_PLLM_Pos;  //PLL-M: /4
    RCC->CR |= RCC_CR_PLLON;                    //Activate PLL (Output: 96 MHz)
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);     //Wait until PLL is ready
    
    //Division by 2 of clk signal       
    RCC->CFGR |= RCC_CFGR_HPRE_DIV2             //AHB divider:  /2 = 48MHz
              | RCC_CFGR_PPRE1_DIV2             //APB1 divider: /2 = 24MHz
              | RCC_CFGR_PPRE2_DIV2;            //APB2 divider: /2 = 24MHz
               
    RCC->CFGR |= RCC_CFGR_SW_PLL;               //Switching to PLL clock source
 
    //////////////////////////////////////////////
    // Setup UART2
    //////////////////////////////////////////////
    ///////////////
    // TX Section  
    //////////////  
    RCC->APB1ENR |= (1 << 17);                //Enable USART2 clock, bit 17 on APB1ENR
    RCC->AHB1ENR |= (1 << 0);                 //Eenable GPIOA clock, bit 0 on AHB1ENR

    //Set pin modes as alternate mode 7 (pins 2 and 3)
    //USART2 TX and RX pins are PA2 and PA3 respectively
    GPIOA->MODER &= ~(0xFU << 4); //Reset bits 4:5 for PA2 and 6:7 for PA3
    GPIOA->MODER |=  (0xAU << 4); //Set   bits 4:5 for PA2 and 6:7 for PA3 to alternate mode (10)

    //Set pin modes as fast speed
    GPIOA->OSPEEDR |= 0xA0 << 4; //Set pin 2/3 to fast speed mode (0b10 << 4)

    //Choose AF7 for USART2 in Alternate Function registers
    GPIOA->AFR[0] |= (0x7 << 8); // for pin A2
    GPIOA->AFR[0] |= (0x7 << 12); // for pin A3

    //USART2 word length M, bit 12
    USART2->CR1 |= (0 << 12); // 0 - 1,8,n

    //USART2 parity control, bit 9
    USART2->CR1 |= (0 << 9); // 0 - no parity

    //USART2 TX enable, TE bit 3 
    USART2->CR1 |= (1 << 3);

    // USART2 rx enable, RE bit 2
    USART2->CR1 |= (1 << 2);

    //Baud = fCK / (8 * (2 - OVER8) * USARTDIV)
    //-> USARTDIV = fCK / ((8 * (2 - OVER8) * Baud)
    //-> USARTDIV = 24000000 / ( 16 * 9600)= 156.25
    //Mantissa : 156
    //Fraction : 16*.25 = 4 (multiply fraction with 16)
    
    USART2->BRR |= (156 << 4); // 12-bit mantissa
    USART2->BRR |= 4;          // 4-bit fraction 
    
    ////////////////
    // RX Section
    ///////////////
    //Setup the NVIC to enable interrupts.
    //Use 4 bits for 'priority' and 0 bits for 'subpriority'.
    NVIC_SetPriorityGrouping(0);
    //UART receive interrupts should be high priority.
    uint32_t uart_pri_encoding = NVIC_EncodePriority(0, 1, 0);
    NVIC_SetPriority(USART2_IRQn, uart_pri_encoding);
    NVIC_EnableIRQ(USART2_IRQn);
    
    //TX and RX enable
    USART2->CR1 |= (USART_CR1_RXNEIE);
    USART2->CR1 |= (1 << 13);  //Enable USART2 - UE, bit 13
    
    while(1)
    {
		GPIOD->ODR |= (1 << 15); 
		for (t1=0; t1 < strlen(msg); t1++)
        {
	        USART2->DR = msg[t1]; //Transmit character
            while(!(USART2->SR & (1 << 6))); //Wait for transmission to be completed

        }
        GPIOD->ODR &= ~(1 << 15); 
        //Delay
        for(int i=0; i<10000000; i++);
        
    }
	return 0;
}
