///////////////////////////////////////////////////////////////////  
/*                    LED driver software                        */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         F411E Discovery board by STM               */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"

//MISC
void main(void);
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

void main(void)
{

	///////////////////////////////////
    // Set up LEDs - GPIOD 12,13,14,15
    ///////////////////////////////////
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~(0xFFU << 24);
    GPIOD->MODER |= (0x55 << 24);
    GPIOD->ODR    = 0x0000;
	  		
	for(;;)
	{
		GPIOD->ODR |= (1 << 12); //Green led off, blink shows successful init
        delay(100);
        
		GPIOD->ODR &= ~(1 << 12);
        delay(100);
	}	
}
