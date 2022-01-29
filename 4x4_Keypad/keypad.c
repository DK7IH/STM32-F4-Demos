///////////////////////////////////////////////////////////////////
//           4x4 keypad decoder for STM32F411                    //
///////////////////////////////////////////////////////////////////
//  Mikrocontroller:  STM32F411 Black-Pill                       //
//                                                               //
//  Compiler:         GCC (GNU AVR C-Compiler)                   //
//  Author:           Peter Baier (DK7IH)                        //
//                    JAN-2022                                   // 
///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
static void delay(unsigned int) ;

// Quick and dirty delay
static void delay(unsigned int time) 
{
    for (unsigned int i = 0; i < time; i++)
    {
        for (volatile unsigned int j = 0; j < 2000; j++);
    }    
}


int main(void)
{	
	int t1, t2;
			
	//Enable ports
	RCC->AHB1ENR |= (1 << 0); //GPIOA enable (Keypad, LEDs)
	RCC->AHB1ENR |= (1 << 1); //GPIOB enable (LEDs)
	RCC->AHB1ENR |= (1 << 2); //GPIOC enable (LED onboard)
			
	//Lines as general purpose output
	for(t1 = 0; t1 < 4; t1++) //4 output lines A0..A3
	{
		GPIOA->MODER |= (1 << (t1 << 1));
	}	
	
	for(t1 = 4; t1 < 8; t1++) //4 input lines A4..A7
	{
		GPIOA->MODER &= ~(1 << (t1 << 1));
		GPIOA->PUPDR &= ~(1 << (t1 << 1));
	}	
	
	for(t1 = 8; t1 < 11; t1++) //3 output lines for red LEDs (A8..A10)
	{
		GPIOA->MODER |= (1 << (t1 << 1));
	}	
	
	for(t1 = 12; t1 < 15; t1++) //3 output lines for green LEDs (B8..B12)
	{
		GPIOB->MODER |= (1 << (t1 << 1));
	}	
	
	GPIOC->MODER |= (1 << (13 << 1)); //Onboard LED
	
	GPIOA->ODR &= ~(0b111 << 8); //All LEDs on
	GPIOB->ODR &= ~(0b111 << 12); //All LEDs on
	delay(100);
		
	//LEDs off
	for(t1 = 8; t1 < 11; t1++)
	{
		GPIOA->ODR |= (1 << t1);
		GPIOB->ODR |= (1 << (t1 + 4));
		delay(100);
	}	
		   
    while(1)
    {
		for(t1 = 0; t1 < 4; t1++)
	    {
			GPIOA->ODR &= ~(0b1111);         //Bits 4:0 as 0
		    GPIOA->ODR |= (1 << t1);         //Set 1 coloumn bit
		    for(t2 = 4; t2 < 8; t2++)        //Read row pins
		    {
				if(GPIOA->IDR & (1 << t2))
				{ 
					GPIOC->ODR &= ~(1 << 13);       //Onboard Led on
					GPIOA->ODR &= ~((t2 - 3) << 8); //Row LED
					GPIOB->ODR &= ~(((3 - t1) + 1) << 12); //Col LED
				}
				else
				{
				    GPIOC->ODR |= (1 << 13); //LED off	
				    GPIOA->ODR |= (0b111 << 8);
				    GPIOB->ODR |= (0b111 << 12);
				}		
			}	
	    }	
	    
	} 	
	return 0;	
}
