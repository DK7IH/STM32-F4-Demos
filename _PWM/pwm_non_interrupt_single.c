///////////////////////////////////////////////////////////////////  
//                    PWM demo for STM32F4                       //
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         STMf407 discovery board                    */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier (DK7IH)                        */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
//Purpose: This demonstration sets the LED on PD12 in STM32F407
//discovery board by using pwm driving. Timer used: TIM4

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#define INPUT_FREQ 25000000
#define PWM_FREQ   20000

  ////////////////////
 //  declarations  //
////////////////////
int main(void);

  ///////////////////
 // Main programm //
///////////////////
int main(void)
{
	int16_t t1, t2;    
		
    //////////////////////////////////////////////
    // Set SystemClock to 96 MHz with 8 MHz HSE
    //////////////////////////////////////////////
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;        //2 wait state for 96 MHz
    RCC->CR |= RCC_CR_HSEON;                    //Activate external clock (HSE: 8 MHz)
    while ((RCC->CR & RCC_CR_HSERDY) == 0);     //Wait until HSE is ready
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;     //PLL source is HSE
    
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;          //PLL-M: VCO input frequency = PLL input clock frequency / PLLM with 2 ≤ PLLM ≤ 63
    RCC->PLLCFGR |= 4 << RCC_PLLCFGR_PLLM_Pos;  // -> f.VCO.in = 8MHz / 4 = 2MHz
                                                
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
    RCC->PLLCFGR |= 100 << RCC_PLLCFGR_PLLN_Pos; //PLL-N: f.VCO.out = f.VCO.in * 100 = 200MHz
    
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;          //PLL-P: Main PLL (PLL) division factor for main system clock
                                                //f.PLL.output.clock = f.VCO.out / 2 = 100MHz
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;          //Main PLL (PLL) division factor for USB OTG FS, SDIO and random number generator clocks
    RCC->PLLCFGR |= 8 << RCC_PLLCFGR_PLLQ_Pos;  //PLL-Q: f.VCO.out / 8 = 25MHz
        
    RCC->CR |= RCC_CR_PLLON;                    //Activate PLL (Output: 96 MHz)
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);     //Wait until PLL is ready
    
    //Division by 2 of clk signal       
    RCC->CFGR |= RCC_CFGR_HPRE_DIV2             //AHB divider:  /2
              | RCC_CFGR_PPRE1_DIV2             //APB1 divider: /2
              | RCC_CFGR_PPRE2_DIV2;            //APB2 divider: /2
               
    RCC->CFGR |= RCC_CFGR_SW_PLL;               //Switching to PLL clock source
    
    //////////////////////////////////////////
    //Setup TIMER
    //////////////////////////////////////////
    RCC->AHB1ENR |= (1 << 3); //Power up PORTD with LEDs
	GPIOD->MODER |= (2UL << (15 << 1)) + (2UL << (14 << 1)) + (2UL << (13 << 1)) + (2UL << (12 << 1)); // Turn on alternate function for BITS 12, 13, 14, 15
	
	//Select AF2 alternate function (Timer 4 output)
	GPIOD->AFR[1] &= 0x0000ffff;
	GPIOD->AFR[1] |= (0b0010 << (16)); //PD12 (green LED)
	
	//Power up Timer 4
	RCC->APB1ENR |= (1 << 2);
	TIM4->ARR = INPUT_FREQ/PWM_FREQ;
	TIM4->CCR1 = 0;
	
	//Channels 1 and 2
	//PWM mode 1: OCxM = 110 in TIM4->CCMRx = PWM mode 1
	//Set OCxPE bit in TIM4_CCMR1
	TIM4->CCMR1 = (3 << 5) + (3 << 2); 
	
    //OCx output is enabled by the CCxE bit in
	//TIM4->CCER register. 
	TIM4->CCER = (1 << 0);
	
	//Set ARPE in CCR1 and enable reload 
	TIM4->CR1 |= (1 << 7);
	
	//Set UG bit in TIM4->EGR
	TIM4->EGR |= (1 << 0);
	
	//Enable counter
	TIM4->CR1 |= (1 << 0);
	
    for(;;)
    { 
		for(t1 = 0; t1 <= 100; t1++)
		{
   		    TIM4->CCR1 = (int) (t1 * TIM4->ARR) / 100;
		    for(t2 = 0; t2 < 16384; t2++); //Delay cheap and dirty!
		 }	
		    
		 for(t1 = 100; t1 >= 0; t1--)
		 {
		    TIM4->CCR1 = (int) (t1 * TIM4->ARR) / 100;
		    for(t2 = 0; t2 < 16384; t2++); //Delay cheap and dirty!
		}       
    }
	return 0;
}
