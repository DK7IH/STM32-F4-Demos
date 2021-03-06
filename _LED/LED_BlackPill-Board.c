///////////////////////////////////////////////////////////////////
//                     Blinky test                               //
///////////////////////////////////////////////////////////////////
//  Mikrocontroller:  STM32F4                                    //
//                                                               //
//  Compiler:         ARM GCC TOOLCHAIN                          //
//  Author:           Peter Rachow (DK7IH)                       //
//                    JUL 2021                                   // 
///////////////////////////////////////////////////////////////////
//                                                               //
#include "stm32f4xx.h"
#include <stdlib.h>

//MISC
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

int main(void)
{
	//////////////////////////////////////////
    // Setup LED
    //////////////////////////////////////////
	//Turn on the GPIOC peripheral for LED
    RCC->AHB1ENR |= (1 << 2); 
	GPIOC->MODER |= (1 << (13 << 1));	//Set PC13 for output
	
    while(1)
    { 			    
        GPIOC->ODR |= (1 << 13);
        delay(100);
        GPIOC->ODR &= ~(1 << 13);
        delay(100);
    
	}
	return 0;
}	

