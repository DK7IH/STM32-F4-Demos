///////////////////////////////////////////////////////////////////  
//                    Rotary encoder demo for STM32F4            //
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         WeAct Blackpill board                      */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
//Purpose: This demo uses input pins PB0/PB1 to detect
//pin change and fire interrupt on EXTI0 when rotary encoder
//is turned cw/ccw. Rising edge of pin change will be detected.
//Value of variable "rotate" becomes either 1 or -1 to indicate 
//direction. This is shown by lighting one of leds (PB14, PB15 to VDD) 
//installed.
//
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <stdlib.h> 
#include <string.h> 


  ////////////
 //  MISC  //
////////////
int main(void);

int16_t rotate = 0;
int16_t laststate = 0; //Last state of rotary encoder

static void delay_ms(unsigned int); 

extern "C" void EXTI0_IRQHandler(void);

extern "C" void EXTI0_IRQHandler(void) 
{ 
	uint16_t state; 
		
	if(EXTI->PR & (1 << 0))  //Check first if the interrupt is triggered by EXTI0 
	{ 
		state = GPIOB->IDR & 0x03; //Read pins
        if(state & 1)
        {
            if(state & 2)
            {
                //Turn CW
                rotate = 1;
            }
            else
            {
                //Turn CCW
                rotate = -1;
            }
        }
        //Clear pending bit
        EXTI->PR = (1 << 0);
    } 
}

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
	int16_t rotate_old = 0;
	int t1;

    //////////////////////////////////////////
    // Setup LEDs
    //////////////////////////////////////////
	//Turn on the GPIOC peripheral for onboard LED
    RCC->AHB1ENR |= (1 << 2);           //Enable GPIOC
	GPIOC->MODER |= (1 << (13 << 1));	//Set PC13 for output
	
    //Turn on the GPIOB peripheral for LED PB14 and PB15
    RCC->AHB1ENR |= (1 << 1);            //RCC->AHB1ENR = 0b10
    GPIOB->MODER |= (1 << (14 << 1));    //Set PB14 for output
    GPIOB->MODER |= (1 << (15 << 1));    //Set PB15 for output
    
    //////////////////////////////////////////////
    //Rotary Encoder Setup
    //////////////////////////////////////////////
    //Set PB0, PB1 as input pins
    RCC->AHB1ENR |= (1 << 1);                           //GPIOB power up
    GPIOB->MODER &= ~((3 << (0 << 1))|(3 << (1 << 1))); //PB0 und PB1 for Input
    GPIOB->PUPDR |= (1 << (0 << 1))|(1 << (1 << 1));    //Pullup PB0 und PB1
    
    RCC->APB2ENR |= (1 << 14); //Enable SYSCFG clock (APB2ENR: bit 14)
    
    SYSCFG->EXTICR[0] |= 0x0001;  //Write 0b01 to map PB0 to EXTI0
    EXTI->RTSR |= 0x01;           //Enable rising edge trigger on EXTI0
    EXTI->IMR |= 0x01;            //Mask EXTI0
    
    NVIC_SetPriority(EXTI0_IRQn, 1); //Set Priority for each interrupt request Priority level 1
    NVIC_EnableIRQ(EXTI0_IRQn);      //Enable EXT0 IRQ from NVIC
    
    for(t1 = 0; t1 < 10; t1++)
    {
		GPIOB->ODR |= (1 << 14);
		GPIOB->ODR &= ~(1 << 15);
		delay_ms(10);
		GPIOB->ODR &= ~(1 << 14);
		GPIOB->ODR |= (1 << 15);
		delay_ms(10);
	}	
    
    //All LEDs off
    GPIOC->ODR |= (1 << 13);
    GPIOB->ODR |= (1 << 14);
    GPIOB->ODR |= (1 << 15);
    
    while(1)
    {		
		if(rotate != rotate_old)
		{
		    if(rotate < 0)
		    {
				GPIOB->ODR |= (1 << 14);
		        GPIOB->ODR &= ~(1 << 15);
		        rotate = 0;
		    }    
		    
		    if(rotate > 0)
		    {
		        GPIOB->ODR &= ~(1 << 14);
		        GPIOB->ODR |= (1 << 15);
		        rotate = 0;
		    }
		    rotate_old = rotate;
		}   
    }
	return 0;
}

