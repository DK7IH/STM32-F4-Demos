//STM32 F4VE LED blink

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

    //////////////////////////////////////////////
    // LED setup
    //////////////////////////////////////////////
    //Turn on the GPIOA peripheral
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    //Put pins PA6 and PA7 in general purpose output mode
    GPIOA->MODER |= (1 << (6 << 1));
    GPIOA->MODER |= (1 << (7 << 1));
    
    while(1)
    {
	    GPIOA->ODR &= ~(1 << 6);
	    GPIOA->ODR |= (1 << 7);
        delay(500);

        GPIOA->ODR |= (1 << 6);
        GPIOA->ODR &= ~(1 << 7);
        delay(500);
    }

    return 0;
}
