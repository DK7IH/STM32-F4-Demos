///////////////////////////////////////////////////////////////////  
/*                    Clock set for STM32F4                      */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         Diymore STM32F4 (F407)                     */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
//Purpose: This code snippet set the STM32F4 MCU to SYSCLK=100 Mhz
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
	//Turn on the GPIOC peripheral
    //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //RCC->AHB1ENR |= (1<<0);
    
    //Turn on the GPIOD peripheral for LED
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; //RCC->AHB1ENR |= (1<<0);
    GPIOE->MODER |= (1 << (0 << 1));	
    GPIOE->ODR |= (1 << 0);
    
    
    //////////////////////////////////////////////
    // Set SystemClock to 100 MHz with 8 MHz HSE
    //////////////////////////////////////////////
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;        //2 wait state for 96 MHz
    RCC->CR |= (1 << 16);                        //Activate external clock (HSE: 8 MHz)
    while ((RCC->CR & (1 << 17)) == 0);          //Wait until HSE is ready
    
    //Configuration of PLL
    RCC->PLLCFGR |= (1 << 22);                  //PLL source is HSE
    
                                                //Set PLLM
    RCC->PLLCFGR &= ~0x3F;                      //1st Reset bits first
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
    
    while(1)
    {
		GPIOE->ODR |= (1 << 0); //Blink LED
        delay_ms(500); 
        
        GPIOE->ODR &= ~(1 << 0);
        delay_ms(500);
    }
	
	return 0;
}
