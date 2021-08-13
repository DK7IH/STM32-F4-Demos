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
//TIM2 is configurated toggling the blue LED evry 500ms so
//that this produces a 1 second blink
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
extern "C" void TIM2_IRQHandler(void)//IRQ-Handler for TIM2
{
    if (TIM2->SR & TIM_SR_UIF)       //Toggle LED on update event every 500ms
    {
		GPIOD->ODR ^= (1 << 15);     //Blue LED
    }
    TIM2->SR = 0x0;                  //Reset status register
}

  ///////////////////
 // Main programm //
///////////////////
int main(void)
{
    //Turn on the GPIOD peripheral for LED
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; //RCC->AHB1ENR |= (1<<0);
    GPIOD->MODER |= (1 << (15 << 1));	//LED blue
    
    //////////////////////////////////////////////
    // Set SystemClock to 100 MHz with 8 MHz HSE
    //////////////////////////////////////////////
    FLASH->ACR |= 0b010;                         //2 wait state for 100 MHz
    RCC->CR |= (1 << 16);                        //Activate external clock (HSE: 8 MHz)
    while ((RCC->CR & (1 << 17)) == 0);          //Wait until HSE is ready
    
    //Configuration of PLL
    RCC->PLLCFGR |= (1 << 22);                  //PLL source is HSE
    
                                                //Set PLLM
    RCC->PLLCFGR &= ~0x3F;                      //1st Reset bits
    RCC->PLLCFGR |= 4;                          //2nd define VCO input frequency = PLL input clock frequency (f.HSE) / PLLM with 2 ≤ PLLM ≤ 63 
                                                //-> f.VCO.in = 8MHz / 4 = 2MHz
                                                
                                                //Set PLLN: PPLLN defines VCO out frequency
    RCC->PLLCFGR &= ~0x7FC0;                    //1st Reset bits 14:6
    RCC->PLLCFGR |= (100 << 6);                 //2nd define f.VCO.out = f.VCO.in * 100 = 200MHz
     
                                                //Set PLLP: Main PLL (PLL) division factor for main system clock; Reset Bits 17:16
    RCC->PLLCFGR &= ~(0b11 << 16);              //Reset bits 17:16
                                                //f.PLL.output.clock = f.VCO.out / 2 = 100MHz
                                                
                                                //Set PLLQ. PLLQ = division factor for USB OTG FS, SDIO and random number generator clocks
    RCC->PLLCFGR &= ~(0b1111 << 24);            //Reset bits 27:24
    RCC->PLLCFGR |= (8 << 24);                  //PLL-Q: f.VCO.out / 8 = 25MHz
        
    RCC->CR |= (1 << 24);                       //Activate PLL, Bit 24
    while ((RCC->CR & (1 << 25)) == 0);         //Wait until PLL is ready Bit 25
    
                                                //Division of clock signal for bus system
    RCC->CFGR |= (0b1001 << 4)                  //AHB divider:  100MHz / 4 = 25 MHz
              | (0b100 << 10)                   //APB1 divider: /2
              | (0b100 << 13);                  //APB2 divider: /2
               
    RCC->CFGR |= 0b10;                          //Switching to PLL clock source
        
    //Enable TIM2 clock (bit0)
    RCC->APB1ENR |= (1 << 0);
    
    //Timer calculation
    //Timer update frequency = TIM_CLK/(TIM_PSC+1)/(TIM_ARR + 1) 
    TIM2->PSC = 100000; //Dive system clock (f=100MHz) by 100000 -> update frequency = 1000/s
    TIM2->ARR = 500;    //Define overrun after 500ms

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

