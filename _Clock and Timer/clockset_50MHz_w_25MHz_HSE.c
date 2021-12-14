///////////////////////////////////////////////////////////////////  
/*                    Clock set for STM32F4                      */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         WeAct BlackPill board                       */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
//Purpose: This code snippet set the STM32F4 MCU to SYSCLK=50 MHz
//by using 25MHz externel clock (aka HSE) on BlackPill baord

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

  ////////////
 //  MISC  //
////////////
int main(void);
static void delay(unsigned int); 

  /////////////////////////////
 //  Quick and esay delay   //
/////////////////////////////
static void delay(unsigned int time) 
{
    for (unsigned int i = 0; i < time; i++)
    {
        for (volatile unsigned int j = 0; j < 2000; j++);
    }    
}

  ///////////////////
 // Main programm //
///////////////////
int main(void)
{
	//////////////////////////////////////////
    // Setup LED
    //////////////////////////////////////////
    //Turn on the GPIOA peripheral for LED
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //RCC->AHB1ENR |= (1<<0);
	GPIOC->MODER |= (1 << (13 << 1));
	
	//Turn on the GPIOA peripheral 
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //RCC->AHB1ENR |= (1<<0);
    
    /////////////////////////////////////////////
    // Set SystemClock to 50 MHz with 25 MHz HSE
    //////////////////////////////////////////////
    FLASH->ACR |= (1 << 1);                     //2 wait states for 96+ MHz
    RCC->CR |= (1 << 16);                       //Activate external clock (HSE: 25 MHz)
    while ((RCC->CR & (1 << 17)) == 0);         //Wait until HSE is ready
    
    RCC->PLLCFGR |= (1 << 22);                  //PLL source is HSE
    
    RCC->PLLCFGR &= ~0b11111;                   //Reset bits 4..0, then set PLL-M: VCO input frequency = PLL input clock frequency / PLLM with 2 ≤ PLLM ≤ 63
    RCC->PLLCFGR |= 20;                         // -> f.VCO.in = 25MHz / 20 = 1.25MHz
                                                
    RCC->PLLCFGR &= ~(32704 << 6);
    RCC->PLLCFGR |= 160 << 6;                   //PLL-N: f.VCO.out = f.VCO.in * 160 = 250MHz
    
    RCC->PLLCFGR &= ~(0b11 << 16);              //Reset - PLL-P: Main PLL (PLL) division factor for main system clock
    RCC->PLLCFGR &= ~(0b01 << 16);              //f.PLL.output.clock = f.VCO.out / 4 = 50MHz
                                                
    RCC->PLLCFGR &= ~(0b111 << 24);             //Main PLL (PLL) division factor for USB OTG FS, SDIO and 
                                                //random number generator clocks (f<=48MHz for RNG!, 48MHz for USB)
    RCC->PLLCFGR |= (4 << 24);                  //PLL-Q: f.VCO.out / 4 = 25MHz
        
    RCC->CR |= (1 << 24);                       //Activate PLL (Output: 100 MHz)
    while ((RCC->CR & (1 << 25)) == 0);         //Wait until PLL is ready
    
    //Division by 2 of clk signal       
    RCC->CFGR |= (0b1000 << 4)                  //AHB divider:  /2
              | (0b100 << 10)                   //APB1 divider: /2
              | (0b100 << 13);                  //APB2 divider: /2
               
    RCC->CFGR |= (1 << 1);                      //Switching to PLL clock source
    
    while(1)
    {
        GPIOC->ODR |= (1 << 13);
        delay(100);
        GPIOC->ODR &= ~(1 << 13);
        delay(100);    
    }
	
	return 0;
}

