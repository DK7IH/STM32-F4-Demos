///////////////////////////////////////////////////////////////////  
//                    Demo: Reading an input port STM32F4        //
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
//Purpose: This demo reads input pin PD15 (Board DIYMORE F407VGT6

#include "stm32f4xx.h"

#define BTNPIN 15 //PD15

int main (void) 
{
    uint32_t pin_input;
    
    //Turn on the GPIOE peripheral for signal LED
    RCC->AHB1ENR |= (1 << 4);
    //Put pin PE0 in general purpose output mode
    GPIOE->MODER |= (1 << (0 << 1));
    
    //Turn on the GPIOD peripheral (Input pin w. user button)
    RCC->AHB1ENR |= (1 << 3);
    
    //Pin PE4 must be set to 'input' mode with pull-up.
    GPIOE->MODER  &= ~(3 << (BTNPIN << 1)); //Set to 00b -> Input mode
    GPIOE->PUPDR  &= ~(3 << (BTNPIN << 1)); //Reset to 00b first 
    GPIOE->PUPDR  |=  (1 << (BTNPIN << 1)); //Set to 01b -> Pull up
    
    while(1)
    {
		pin_input = ~(GPIOD->IDR); //"0" means "pressed"!
		if(pin_input & (1 << BUTTON_PIN))
		{
            GPIOE->ODR &= ~(1 << 0);
        }
        else
        { 
        	GPIOE->ODR |= (1 << 0);
        }
    }

    return 0;
}
