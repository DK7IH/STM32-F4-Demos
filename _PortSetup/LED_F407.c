//////////////////////////////////////////////////////////////////////////
//              Port settings for STM32F4                               //
//////////////////////////////////////////////////////////////////////////
//  Mikrocontroller:  ARM Cortex M4 (STM32F407)                         //
//  Board:            F407                                              //
//  https://dk7ih.de/wp-content/uploads/2021/08/f407-stm32-board.png    //
//  Compiler:         GCC (GNU AVR C-Compiler)                          //
//  Author:           Peter (DK7IH)                                     //
//  Last change:      2021-08-08                                        //
//////////////////////////////////////////////////////////////////////////
//Purpose: Make e LED blink connected to port E pin 0                   //
//         Demonstrate port config                                      //
//////////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
 
// Quick and dirty delay
static void delay (unsigned int time) 
{
    for (unsigned int i = 0; i < time; i++)
    {
        for (volatile unsigned int j = 0; j < 2000; j++);
    }    
}
 
int main (void) {

    //Turn on the GPIOE peripheral
    RCC->AHB1ENR |= (1 << 4);
    
    //Put pin PE0 in general purpose output mode (2 bits count!)
    GPIOE->MODER |= (1 << (0 << 1));
    
    //Put pin PE0 in general purpose output mode pull UP (2 bits count!)
    GPIOE->PUPDR |= (1 << (0 << 1));
    
    //Set speed to high (2 bits count!)
    GPIOE->OSPEEDR |= (3 << (0 << 1));
    
    //Set output type to push pull (1 bit count!)
    GPIOE->OTYPER |= (0 << 1);
    
    while(1)
    {
		//Reset pin
	    GPIOE->ODR &= ~(1 << 0);
        delay(50);

        //Set pin
        GPIOE->ODR |= (1 << 0);
        delay(950);
    }

    return 0;
}
