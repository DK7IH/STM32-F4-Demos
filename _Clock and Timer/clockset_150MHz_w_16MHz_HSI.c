///////////////////////////////////////////////////////////////////  
/*                    Clock set for STM32F4                      */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         WeAct BlackPill board                       */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
//Purpose: This code snippet set the STM32F4 MCU to SYSCLK=96 MHz
//by using 16MHz internel clock (aka HSI) on BlackPill baord

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
	    
    //////////////////////////////////////////////
    // Set SystemClock to 150 MHz with 8 MHz HSI
    //////////////////////////////////////////////
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;        //2 wait state for 96 MHz
    RCC->CR |= RCC_CR_HSION;                    //Activate external clock (HSE: 8 MHz)
    while ((RCC->CR & RCC_CR_HSIRDY) == 0);     //Wait until HSE is ready
    
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;     //PLL source is HSE
    
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;          //PLL-M: VCO input frequency = PLL input clock frequency / PLLM with 2 ≤ PLLM ≤ 63
    RCC->PLLCFGR |= 8 << RCC_PLLCFGR_PLLM_Pos;  // -> f.VCO.in = 16MHz / 8 = 2MHz
                                                
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
    RCC->PLLCFGR |= 150 << RCC_PLLCFGR_PLLN_Pos; //PLL-N: f.VCO.out = f.VCO.in * 150 = 300MHz
    
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;          //PLL-P: Main PLL (PLL) division factor for main system clock
                                                //f.PLL.output.clock = f.VCO.out / 2 = 150MHz
                                                
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;          //Main PLL (PLL) division factor for USB OTG FS, SDIO and 
                                                //random number generator clocks (f<=48MHz for RNG!, 48MHz for USB)
    RCC->PLLCFGR |= 5 << RCC_PLLCFGR_PLLQ_Pos;  //PLL-Q: f.VCO.out / 6 = 30MHz
        
    RCC->CR |= RCC_CR_PLLON;                    //Activate PLL (Output: 100 MHz)
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);     //Wait until PLL is ready
    
    //Division by 2 of clk signal       
    RCC->CFGR |= RCC_CFGR_HPRE_DIV2             //AHB divider:  /2
              | RCC_CFGR_PPRE1_DIV2             //APB1 divider: /2
              | RCC_CFGR_PPRE2_DIV2;            //APB2 divider: /2
               
    RCC->CFGR |= RCC_CFGR_SW_PLL;               //Switching to PLL clock source
    
    while(1)
    {
        GPIOC->ODR |= (1 << 13);
        delay(100);
        GPIOC->ODR &= ~(1 << 13);
        delay(100);    
    }
	
	return 0;
}

