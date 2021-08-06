///////////////////////////////////////////////////////////////////
/*                    Measure SYSCLK on PA8 Demo                 */
/*  ************************************************************ */
/*  Mikrocontroller:  ARM Cortex M4 (STM32F407)                  */
/*  Board:                                                       */
/*  Compiler:         GCC (GNU ARM Toolchain C-Compiler)         */
/*  Autor:            Peter Rachow                               */
/*  Last change:      2021-07-06                                 */
///////////////////////////////////////////////////////////////////

//This demo measures SYSCLK speed on Pin PA8 divides by 2

#include "stm32f4xx.h"

int main(void);
static void delay (unsigned int);

  /////////////////////////////
 //  Cheap and dirty delay  //
/////////////////////////////
static void delay (unsigned int time) 
{
    for (unsigned int i = 0; i < time; i++)
    {
        for (volatile unsigned int j = 0; j < 2000; j++);
    }    
}

int main()
{

    //Turn on GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //RCC->AHB1ENR |= (1<<0);
    
    //Set PA8 as MCO pin
    RCC->CFGR |= (0b11 << 21);          //MCO1: Microcontroller clock output 1 11: PLL clock selected
    RCC->CFGR |= (0b100 <<24);          //Divide f.out by 2
    GPIOA->MODER |= (2 << (8 << 1));	//PA8 as AF
    GPIOA->OSPEEDR |= (3 << (8 << 1));	//HiSpeed
    GPIOA->AFR[1] = 0;                  //0b0000
    
    ///////////////////////////////////
    // Set up LEDs - GPIOD 12,13,14,15
    ///////////////////////////////////
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~(0xFFU << 24);
    GPIOD->MODER |= (0x55 << 24);
    GPIOD->ODR    = 0x0000;
    
    //////////////////////////////////////////////
    // Set SystemClock to 96 MHz with 8 MHz HSE
    //////////////////////////////////////////////
    
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;        //1 wait state for 96 MHz
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
    	        
    for(;;) 
	{
        GPIOD->ODR |= (1 << 12); //Green led off, blink shows successful init
        delay(100);
        
		GPIOD->ODR &= ~(1 << 12);
        delay(100);		
	}
	return 0;
}

