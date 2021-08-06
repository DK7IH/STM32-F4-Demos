///////////////////////////////////////////////////////////////////  
/*                    Clock set for STM32F4                      */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         F411E Discovery board by STM               */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
//Purpose: This code snippet set the STM32F4 MCU to SYSCLK=96 Mhz
//by using 8MHz externel clock (aka HSE)

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <stdlib.h> 

  ////////////
 //  MISC  //
////////////
int main(void);
static void delay_ms(unsigned int); 

  /////////////////////////////
 //  Quick and esay delay   //
/////////////////////////////
static void delay_ms(unsigned int time) 
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
	//Turn on the GPIOA peripheral
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //RCC->AHB1ENR |= (1<<0);
    
    //Turn on the GPIOD peripheral for LED
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; //RCC->AHB1ENR |= (1<<0);
    GPIOD->MODER |= (1 << (13 << 1));	
    GPIOD->ODR |= (1 << 13);
    
    //////////////////////////////////////////////
    // Set SystemClock to 168 MHz with 8 MHz HSE
    //////////////////////////////////////////////
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;        //2 wait state for 168 MHz
    RCC->CR |= RCC_CR_HSEON;                    //Activate external clock (HSE: 8 MHz)
    while ((RCC->CR & RCC_CR_HSERDY) == 0);     //Wait until HSE is ready
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;     //PLL source is HSE
    
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;          //PLL-M: VCO input frequency = PLL input clock frequency / PLLM with 2 ≤ PLLM ≤ 63
    RCC->PLLCFGR |= 4 << RCC_PLLCFGR_PLLM_Pos;  // -> f.VCO.in = 8MHz / 4 = 2MHz
                                                
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
    RCC->PLLCFGR |= 168 << RCC_PLLCFGR_PLLN_Pos; //PLL-N: f.VCO.out = f.VCO.in * 100 = 336MHz
    
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;          //PLL-P: Main PLL (PLL) division factor for main system clock
                                                //f.PLL.output.clock = f.VCO.out / 2 = 168MHz
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;          //Main PLL (PLL) division factor for USB OTG FS, SDIO and random number generator clocks
    RCC->PLLCFGR |= 7 << RCC_PLLCFGR_PLLQ_Pos;  //PLL-Q: f.VCO.out / 8 = 24MHz
        
    RCC->CR |= RCC_CR_PLLON;                    //Activate PLL (Output: 168 MHz)
    while((RCC->CR & RCC_CR_PLLRDY) == 0);      //Wait until PLL is ready
    
    //Division by 2 of clk signal       
    RCC->CFGR |= RCC_CFGR_HPRE_DIV2             //AHB divider:  /2
              | RCC_CFGR_PPRE1_DIV2             //APB1 divider: /2
              | RCC_CFGR_PPRE2_DIV2;            //APB2 divider: /2
               
    RCC->CFGR |= RCC_CFGR_SW_PLL;               //Switching to PLL clock source
  
    while(1)
    {
		GPIOD->ODR |= (1 << 13); //Blink LED
        delay_ms(500); 
        
        
        GPIOD->ODR &= ~(1 << 13);
        delay_ms(500);
    }
	
	return 0;
}

