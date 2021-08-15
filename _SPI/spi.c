///////////////////////////////////////////////////////////////////  
/*                    SPI Demo + IL9341 for STM32F4              */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         STM32F411 "Black Pill" Board               */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      AUG 2021                                   */
///////////////////////////////////////////////////////////////////
//
//This "bare metal" demo shwos how to set up SPI interface on 
//ARM Corex-M4 MCU.

//CS =        PB12
//SCK =       PB13
//MISO      = PB14 (not used in this example)
//MOSI(SDI) = PB15
//


#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

int main(void);
void spi_write(uint8_t, uint8_t);

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

void spi_write(uint8_t data)
{
	GPIOB->ODR &= ~(1 << 12);       //CS low
	SPI2->DR = data;                //Write data to SPI interface
	while (!(SPI2->SR & (1 << 1))); //Wait till TX buf is clear
    GPIOB->ODR |= (1 << 12);        //CS high
}	
int main(void)
{
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
	//PB12:CS; PB13:SCK; PB14:MISO; PB15:MOSI
    RCC->AHB1ENR |= (1 << 1);                   //GPIOB power clock enable
    RCC->APB1ENR |= (1 << 14);                  //Enable SPI1 clock, bit 12 in APB2ENR
    
    //Alternate function ports
    GPIOB->MODER &= ~(0b11111111U << (12 << 1));    //Reset bits 15:12
    GPIOB->MODER |=  (0b01 << (12 << 1));           //PB12 (CS) as output
    GPIOB->MODER |=  (0b101010U << (13 << 1));      //Set bits 15:13 to 0b101010 for alternate function (AF5)
    GPIOB->OSPEEDR |= (0b11111111 << (12 << 1));    //Speed vy hi PB15:PB12
    
    //Set AF5 (0x05) for SPI2 in AF registers (PB13:PB15)
    GPIOB->AFR[1] |= (0x05 << (20)); //PB13
    GPIOB->AFR[1] |= (0x05 << (24)); //PB14
    GPIOB->AFR[1] |= (0x05 << (28)); //PB15
    
    //Set SPI2 properties
    SPI2->CR1 |= (1 << 2);     //Master mode
    SPI2->CR1 |= SPI_CR1_SSM;  //Software slave management enabled
    SPI2->CR1 |=  SPI_CR1_SSI; //Internal slave select The value of this bit is 
                               //forced onto the NSS pin and the IO value of the NSS pin is ignored.
    SPI2->CR1 |= SPI_CR1_SPE;  //SPI2 enable
    

   //Ready to go with SPI    

    
    while(1)
    {
           
    }

    return 0;
}
