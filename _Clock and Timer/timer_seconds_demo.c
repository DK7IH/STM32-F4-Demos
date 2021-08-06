///////////////////////////////////////////////////////////////////  
//                    Clock set for STM32F4 with Timer2 as       //
//                    counter for seconds                        //  
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         F411E Discovery board by STM               */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <stdlib.h> 

  ////////////
 //  MISC  //
////////////
int main(void);
extern "C" void TIM2_IRQHandler(void);      // IRQ-Handler timer2

  /////////////////////////////
 //     TIM2 INT Handler    //
/////////////////////////////
extern "C" void TIM2_IRQHandler(void)  // IRQ-Handler timer2
{
    if (TIM2->SR & TIM_SR_UIF)       // flash on update event
    {
		GPIOD->ODR ^= (1 << 13);
        GPIOD->ODR ^= (1 << 15);   // Turn GPIOD pin15 (blue LED) ON in GPIO port output data register
    }
    TIM2->SR = 0x0;              // reset the status register
}

  ///////////////////
 // Main programm //
///////////////////
int main(void)
{
    //Turn on the GPIOC peripheral for LED
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; //RCC->AHB1ENR |= (1<<0);
    GPIOD->MODER |= (1 << (13 << 1))|(1 << (15 << 1));	//red and blue
    GPIOD->ODR |= (1 << 13);
    GPIOD->ODR &= ~(1 << 15);
    
    //////////////////////////////////////////////
    // Set SystemClock to 48 MHz with 8 MHz HSE
    //////////////////////////////////////////////
    FLASH->ACR |= FLASH_ACR_LATENCY_1WS;      //1 wait state for 48 MHz
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

   // enable TIM2 clock (bit0)
    RCC->APB1ENR |= (1 << 0);
    
    //Timer calculation
    //Timer updade frequency = TIM_CLK/(TIM_PSC+1)/(TIM_ARR + 1) 
    TIM2->PSC = 2400;
    TIM2->ARR = 10000;

    //Update Interrupt Enable
    TIM2->DIER |= (1 << 0);

    NVIC_SetPriority(TIM2_IRQn, 2); //Priority level 2
    NVIC_EnableIRQ(TIM2_IRQn);      //Enable TIM2 IRQ from NVIC

    
    TIM2->CR1 |= (1 << 0);           //Enable Timer 2 module (CEN, bit0)

    while(1)
    {
    }
	
	return 0;
}

