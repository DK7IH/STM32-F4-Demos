///////////////////////////////////////////////////////////////////  
//                    PWM demo for STM32F4                       //
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         STMf407 discovery board                    */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier (DK7IH)                        */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
//Purpose: This demonstration sets the 4 LEDs on the STM32F407
//discovery board by using pwm driving. Timer used: TIM4

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#define INPUT_FREQ 25000000
#define PWM_FREQ   20000

  ////////////////////
 //  declarations  //
////////////////////
int main(void);
void sduty(int, int);

void sduty(int channel, int dutycycle)
{
	switch (channel)
	{
		case 0: TIM4->CCR1 = (int) (dutycycle * TIM4->ARR) / 100;
			    break;
		case 1: TIM4->CCR2 = (int)(dutycycle * TIM4->ARR) / 100;
			    break;
		case 2: TIM4->CCR3 = (int)(dutycycle * TIM4->ARR) / 100;
			    break;
		case 3: TIM4->CCR4 = (int)(dutycycle * TIM4->ARR) / 100;
			    break;
	}
}

  ///////////////////
 // Main programm //
///////////////////
int main(void)
{
	int16_t t0, t1, t2;    
	
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
    //Setup TIMER
    //////////////////////////////////////////
    RCC->AHB1ENR |= (1 << 3); //Power up PORTD
	GPIOD->MODER |= (2UL << (15 << 1)) + (2UL << (14 << 1)) + (2UL << (13 << 1)) + (2UL << (12 << 1)); // Turn on alternate function for BITS 12, 13, 14, 15
	
	// Select AF2 alternate function (Timer 4 output)
	GPIOD->AFR[1] &= 0x0000ffff;
	GPIOD->AFR[1] |= (0b0010 << (28))+ (0b0010 << (24)) + (0b0010 << (20)) + (0b0010 << (16));
	
	//Power up Timer 4
	RCC->APB1ENR |= (1 << 2);
	TIM4->ARR = INPUT_FREQ/PWM_FREQ;
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;
	TIM4->CCR4 = 0;
	
	//Channels 1 and 2
	//PWM mode 1: OCxM = 110 in TIM4->CCMRx = PWM mode 1
	//Set OCxPE bit in TIM4_CCMRx
	TIM4->CCMR1 = (3 << 13) + (3 << 10) + (3 << 5) + (3 << 2); 
	//Channels 3 and 4
	TIM4->CCMR2 = (3 << 13) + (3 << 10) + (3 << 5) + (3 << 2); 
    
    //OCx output is enabled by the CCxE bit in
	//TIM4->CCER register. 
	TIM4->CCER = (1 << 12) + (1 << 8) + (1 << 4) + (1 << 0);
	
	//Set ARPE in CCR1 and enable reload 
	TIM4->CR1 |= (1 << 7);
	
	//Set UG bit in TIM4->EGR
	TIM4->EGR |= (1 << 0);
	
	//Enable counter
	TIM4->CR1 |= (1 << 0);
	
    for(;;)
    { 
		for(t0 = 0; t0 < 4; t0++)
		{
		    for(t1 = 0; t1 <= 100; t1++)
		    {
			    sduty(t0, t1);
			    for(t2 = 0; t2 < 16384; t2++); //Delay cheap and dirty!
		    }	
		    for(t1 = 100; t1 >= 0; t1--)
		    {
			    sduty(t0, t1);
			    for(t2 = 0; t2 < 16384; t2++); //Delay cheap and dirty!
		    }	
		}       
    }
	return 0;
}
