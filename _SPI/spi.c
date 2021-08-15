///////////////////////////////////////////////////////////////////  
/*                    SPI Demo for STM32F4                       */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         STM32F411 "Black Pill" Board               */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      AUG 2021                                   */
///////////////////////////////////////////////////////////////////
//
//This "bare metal" demo shwos how to set up and use
//SPI interface on ARM Corex-M4 MCU

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"


int main(void);

/*
static void delay_ms(unsigned int); 
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
*/

int main(void)
{
	uint16_t i;	
	//////////////////////////////////////////
    // Setup LED
    //////////////////////////////////////////
	//Turn on the GPIOC peripheral for LED
    RCC->AHB1ENR |= (1 << 2); 
	GPIOC->MODER |= (1 << (13 << 1));	

    //////////////////////////////////////////////
    // Set SystemClock to 100 MHz with 25 MHz HSE
    //////////////////////////////////////////////
    FLASH->ACR |= 0b010;                         //2 wait state for 100 MHz
    RCC->CR |= (1 << 16);                        //Activate external clock (HSE: 8 MHz)
    while ((RCC->CR & (1 << 17)) == 0);          //Wait until HSE is ready
    
    //Configuration of PLL
    RCC->PLLCFGR |= (1 << 22);                  //PLL source is HSE
    
                                                //Set PLLM
    RCC->PLLCFGR &= ~0x3F;                      //1st Reset bits
    RCC->PLLCFGR |= 20;                          //2nd define VCO input frequency = PLL input clock frequency (f.HSE) / PLLM with 2 ≤ PLLM ≤ 63 
                                                //-> f.VCO.in = 25MHz / 8 = 1.25MHz
                                                
                                                //Set PLLN: PPLLN defines VCO out frequency
    RCC->PLLCFGR &= ~0x7FC0;                    //1st Reset bits 14:6
    RCC->PLLCFGR |= (160 << 6);                 //2nd define f.VCO.out = f.VCO.in * 160 = 200MHz
     
                                                //Set PLLP: Main PLL (PLL) division factor for main system clock; Reset Bits 17:16
    RCC->PLLCFGR &= ~(0b11 << 16);              //Reset bits 17:16
                                                //f.PLL.output.clock = f.VCO.out / 2 = 100MHz
                                                
                                                //Set PLLQ. PLLQ = division factor for USB OTG FS, SDIO and random number generator clocks
    RCC->PLLCFGR &= ~(0b1111 << 24);            //Reset bits 27:24
    RCC->PLLCFGR |= (8 << 24);                  //PLL-Q: f.VCO.out / 8 = 25MHz
        
    RCC->CR |= (1 << 24);                       //Activate PLL, Bit 24
    while ((RCC->CR & (1 << 25)) == 0);         //Wait until PLL is ready Bit 25
    
                                                //Division of clock signal for bus system
    RCC->CFGR |= (0b1001 << 4)                  //AHB divider:  100MHz / 4 = 25 MHz
              | (0b100 << 10)                   //APB1 divider: /2
              | (0b100 << 13);                  //APB2 divider: /2
               
    RCC->CFGR |= 0b10;                          //Switching to PLL clock source
    
    //////////////////////////////////////////
    // Setup SPI
    //////////////////////////////////////////
	//PA4: NSS/CS PA5: SCK  PA6: MISO  PA7: MOSI
    //Enable GPIOA clock for PA7:PA4 
    RCC->AHB1ENR |= (1 << 0);                   //GPIOA power clock enable
    RCC->APB2ENR |= (1 << 12);                  //Enable SPI1 clock, bit 12 in APB2ENR

    GPIOA->MODER |=  (0b01 << (4 << 1));      //PA4 as output (CS)
    
    GPIOA->MODER &= ~(0b111111U << (5 << 1)); //Reset bits 7:5
    GPIOA->MODER |=  (0b101010U << (5 << 1));   //Set bits 7:5 to 0b101010 for alternate function (AF5)
    GPIOA->OSPEEDR |= (0b11111111U << (4 << 1)); //Speed vy hi PA7:PA4
    
    //Set AF5 (0x05) for SPI1 in AF registers (PA7:PA5)
    GPIOA->AFR[0] |= (0x05 << (5 << 2)); //PA5
    GPIOA->AFR[0] |= (0x05 << (6 << 2)); //PA6
    GPIOA->AFR[0] |= (0x05 << (7 << 2)); //PA7
    
    //Set Baud rate SPI_CR Bits 5:3
    SPI1->CR1 = (0b111 << 3); //Set Baud to fPCLK/256	

    //8 or 16-bit mode Bit 11
    SPI1->CR1 &= ~(0b11 << 11); //8-bit mode

    SPI1->CR1 |= (0 << 1);  //Clk is 1 when idle
    SPI1->CR1 |= (0 << 0);  //Clock phase = first clock transaction
    SPI1->CR1 &= ~(1 << 7); //0 - MSB transmitted first
    SPI1->CR2 &= ~(1 << 4); //0 - SPI Motorola mode

    //Software slave management - SSM bit 9
    SPI1->CR1 |= (1 << 9); // 1- ssm enabled
    //Internal slave select - SSI bit 8
    SPI1->CR1 |= (1 << 8); // set ssi to 1

    SPI1->CR1 |= (1 << 2); //Set master mode

    //Enable SPI - SPE bit 6
    SPI1->CR1 |= (1 << 6);

    while(1)
    {
		for(i = 0; i < 255; i++)
		{
		    GPIOC->ODR |= (1 << 13); //Blink LED
            GPIOA->ODR &= ~(1 << 4); //CS low
            SPI1->DR = i;
            while (!(SPI1->SR & (1 << 1)));  //Wait till TX buf is clear
            GPIOA->ODR |= (1 << 4); //CS high
            GPIOC->ODR &= ~(1 << 13);
         }   
    }

    return 0;
}
