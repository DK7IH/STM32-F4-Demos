///////////////////////////////////////////////////////////////////  
//                    Demo: Reading an input port STM32F4        //
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
//Purpose: This demo reads input pin PE4

#include "stm32f4xx.h"

#define BUTTON_PIN 4 //PE4

// Quick and dirty delay
static void delay (unsigned int time) 
{
    for (unsigned int i = 0; i < time; i++)
    {
        for (volatile unsigned int j = 0; j < 2000; j++);
    }    
}

int main (void) 
{

    uint32_t pin_input;
    //Turn on the GPIOA peripheral (LED out)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    //Turn on the GPIOE peripheral (Input pin)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    
    //LEDs
	//Put pin A6 and A7 in general purpose output mode
    GPIOA->MODER  |=  (1 << (6 << 1));	
    GPIOA->MODER  |=  (1 << (7 << 1));	
	
    //Pin PE4 should be set to 'input' mode with pull-up.
    GPIOE->MODER  &= ~(3 << (BUTTON_PIN << 1)); //Set to 00 -> Input mode
    GPIOE->PUPDR  &= ~(3 << (BUTTON_PIN << 1)); //Set to 00	-> No pullup nor pulldown
    GPIOE->PUPDR  |=  (1 << (BUTTON_PIN << 1)); //Set to 01 -> Pull up
    
    //Define output pins B6, B7
    GPIOA->BSRR = GPIO_BSRR_BS6;
    GPIOA->BSRR = GPIO_BSRR_BS7;

    while(1)
    {
		pin_input = ~GPIOE->IDR; //"0" means "pressed"!
		if(pin_input & (1 << BUTTON_PIN))
		{
            GPIOA->BSRR = GPIO_BSRR_BS7;
        }
        else
        { 
        	
            GPIOA->BSRR = GPIO_BSRR_BR7;
        }
        GPIOA->BSRR = GPIO_BSRR_BS6;
        delay(100);
        GPIOA->BSRR = GPIO_BSRR_BR6;
        delay(100);
    }

    return 0;
}
