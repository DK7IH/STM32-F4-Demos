///////////////////////////////////////////////////////////////////  
//                    Timer demo for STM32F4                     //
///////////////////////////////////////////////////////////////////
//Purpose: This demonstration reads analogue voltage from pin PB0//
//and displays its numeric value to an LCD DOT matrix module     //
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         WeAct Blackpill board                      */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
//Purpose: This demonstration uses Timer2 to generate a 1kHz signal
//used for a delay in Milliseconds

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <math.h>

  ////////////
 //  MISC  //
////////////
int main(void);

  ///////////////////
 // Main programm //
///////////////////
int main(void)
{
	uint32_t ti = 1000; //Delay time in Milliseconds
	
	//////////////////////////////////////////
    // Setup LED w. GPIOC
    //////////////////////////////////////////
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //RCC->AHB1ENR |= (1<<0);
    GPIOC->MODER |= (1 << (13 << 1));
	
	GPIOC->ODR |= (1 << 13);                      //LED off
    
    //////////////////////////////////////////////
    // Set SystemClock to 50 MHz with 16 MHz HSI
    //////////////////////////////////////////////
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;        //2 wait states
    RCC->CR |= RCC_CR_HSION;                    //Activate internal clock (HSI: 16 MHz)
    while ((RCC->CR & RCC_CR_HSIRDY) == 0);     //Wait until HSE is ready
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;
    RCC->PLLCFGR |= 4 << RCC_PLLCFGR_PLLQ_Pos;  //PLL-Q: /4
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;     //PLL source is HSI
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;          //PLL-P: /2: f.PLL = f.VCO.out / 2 = 50MHz
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
    RCC->PLLCFGR |= 50 << RCC_PLLCFGR_PLLN_Pos; //PLL-N: x50: f.VCO.out = 50 x f.VCO.in = 100MHz
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
    RCC->PLLCFGR |= 8 << RCC_PLLCFGR_PLLM_Pos;  //PLL-M: /8 -> f.VCO.in = f.HSI / 8 = 2 MHZ
    RCC->CR |= RCC_CR_PLLON;                    //Activate PLL (Output: 50 MHz)
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);     //Wait until PLL is ready
    
    //Division by 2 of clk signal       
    RCC->CFGR |= RCC_CFGR_HPRE_DIV2             //AHB divider:  /2 =25MHz
              | RCC_CFGR_PPRE1_DIV2             //APB1 divider: /2 =25MHz
              | RCC_CFGR_PPRE2_DIV2;            //APB2 divider: /2 =25MHz
               
    RCC->CFGR |= RCC_CFGR_SW_PLL;               //Switching to PLL clock source

	//////////////////////////////////////////
    // Setup TIMER2
    //////////////////////////////////////////
    RCC->APB1ENR |= (1 << 0);         //Enable TIM2 clock (Bit0)
 	TIM2->CR1 = 0;                    //Reset register
	TIM2->SR = 0;                     //Clear the update event flag
	TIM2->PSC = 25000;                //Set prescaler to 1kHz
    TIM2->ARR = 0xFFFF;               //Set delay
    TIM2->CR1 |= (1 << 0);            //Start the timer counter 
        
	GPIOC->ODR &= ~(1 << 13);                      //LED on
            
    while(1)
    { 
		TIM2->EGR &= ~(1 << 0);            //Timer reset
        TIM2->CNT = 0x00;                 //Counter register reset
	    while(TIM2->CNT < ti - 100);
	    GPIOC->ODR &= ~(1 << 13);         //LED on
		
		TIM2->EGR &= ~(1 << 0);            //Timer reset
        TIM2->CNT = 0x00;                 //Counter register reset
	    while(TIM2->CNT < 100);
	    GPIOC->ODR |= (1 << 13);          //LED off
    
	 
    }
	return 0;
}
