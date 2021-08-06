///////////////////////////////////////////////////////////////////  
//                    PWM demo for STM32F4                       //
///////////////////////////////////////////////////////////////////
//Purpose: This demonstration sets an external LED (PA6 to VDD)  //
//with PWM driving derived from TIM3.                            //
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         WeAct Blackpill board                      */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
//Purpose: This demonstration sets an external LED to a given PWM 
//value

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#define PWMPERIOD 255
#define PWMMAX    255

  ////////////
 //  MISC  //
////////////
int main(void);
extern "C" void TIM3_IRQHandler(void);

extern "C" void TIM3_IRQHandler(void)
{
    static uint32_t dc = 0;
    
    //Clear interrupt status
    if((TIM3->DIER & 0x01) && (TIM3->SR & 0x01))
    {
        TIM3->SR &= ~(1U << 0);
    }

    if(dc >= PWMMAX)
    {
		dc = 0;
	}	
    
    TIM3->CCR1 = (PWMMAX - dc++); //Set new duty cycle
}

  ///////////////////
 // Main programm //
///////////////////
int main(void)
{
	    
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
    
    RCC->AHB1ENR |= (1 << 0);             //Enable GPIOA clock
    GPIOA->MODER |= (0x02 << 12);         //Set PA6 to AF mode
    
    GPIOA->AFR[0] |= (0b0010 << 24);      //Choose Timer3 as AF2 for Pin PA6 LED
    
    RCC->APB1ENR |= (1 << 1);             //Enable TIM3 clock (Bit1)

    TIM3->PSC = 499; //fCK_PSC / (PSC[15:0] + 1) -> 50 Mhz / (499 + 1) = 100 khz timer clock speed
    
    TIM3->ARR = PWMPERIOD; //Set period
    TIM3->CCR1 = 0;        //Set duty cycle on channel 1
    
    TIM3->CCMR1 |= (0x06 << 4); //Set OC1 mode as PWM (0b110 (0x06) in Bits 6:4)
    
    TIM3->CCMR1 |= (1 << 3); //Enable OC1 preload (Bit3)
    TIM3->CCER |= (1 << 0);  //Enable capture/compare CH1 output
    TIM3->DIER |= (1 << 0);  //Enable update interrupt

    NVIC_SetPriority(TIM3_IRQn, 2); // Priority level 2
    NVIC_EnableIRQ(TIM3_IRQn);      //Enable TIM3 IRQ via NVIC
    TIM3->CR1 |= (1 << 0);          //Switch on TIM3
    
    while(1)
    { 
    }
	return 0;
}
