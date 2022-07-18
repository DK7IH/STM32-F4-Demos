///////////////////////////////////////////////////////////
// Demo for rotary encoder (connected to PB0, PB1)       //
// Uses 2 LEDs to GND with resistor on PB6 and PB7 to    //
// show rotating direction                               //  
//                                                       //
// 2021 by Peter Baier micromaker.de                     //
///////////////////////////////////////////////////////////
#include "stm32f4xx.h"

//Interrupt handler for rotary encoder
extern "C" void EXTI0_IRQHandler(void);
int main(void);

int rotate = 0;

extern "C" void EXTI0_IRQHandler(void)
{
   int16_t state; 
    
    GPIOC->ODR ^= (1 << 13); //Toggle LED
    
    // Check if the interrupt came from exti0
    if (EXTI->PR & (1 << 0))
    {
        state = GPIOB->IDR & 0x03; //Read pins
        if(state & 1)
        {
            if(state & 2)
            {
                rotate = 1;
            }
            else
            {
                rotate = -1;
            }
        }

        //Clear bit
        EXTI->PR = (1 << 0);
    }
}

int main(void)
{
	//GPIOC power up for LED
    RCC->AHB1ENR |= (1 << 2);  
    GPIOC->MODER |= (1 << (13 << 1));                         
	
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
    
    //Leds on PB3, PB4 to indicate rotate direction
  	GPIOB->MODER |= (1 << (6 << 1));  //PB6 as output                        
    GPIOB->MODER |= (1 << (7 << 1));  //PB7 as output     
    
    //GPIOB->ODR &= ~ (1 << 6);
    //GPIOB->ODR &= ~ (1 << 7);
        
    rotate = 0;
    
    while(1)
    {
		if(rotate == 1)                  
		{
			GPIOB->ODR &= ~(1 << 6);
			GPIOB->ODR |= (1 << 7);
			rotate = 0;
		}
		
		if(rotate == -1)                  
		{	
			GPIOB->ODR &= ~(1 << 7);
		    GPIOB->ODR |= (1 << 6);	
		    rotate = 0;
		}	
	}	
    
}
