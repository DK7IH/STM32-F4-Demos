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
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    
    //Put pin PE0 in general purpose output mode
    GPIOE->MODER |= (1 << 0);
    
    while(1)
    {
	    GPIOE->ODR &= ~(1 << 0);
        delay(500);

        GPIOE->ODR |= (1 << 0);
        delay(500);
    }

    return 0;
}
