///////////////////////////////////////////////////////////////////  
/*                    AD9850 DDS driver software                 */
/*                    DDS in Parallel mode                       */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         Black Pill Board                           */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      OCT 2021                                   */
///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <math.h>

// PIN and PORT definitions for AD9850 lines connected to PORT A
#define DDS_DATA_GPIO GPIOA
#define DDS_CTRL_GPIO GPIOB
#define W_CLK 0   //PB0 blue
#define FQ_UD 1   //PB1 white
#define RES   2   //PB2 pink

//DDS
void set_frequency(unsigned long);

//MISC
static void delay_us (unsigned int);
 
// Quick and dirty delay
static void delay_us (unsigned int time) 
{
    for (unsigned int i = 0; i < time; i++)
    {
        for (volatile unsigned int j = 0; j < 2; j++);
    }    
}

//Set AD9850 to desired frequency
void set_frequency(unsigned long fx)
{
    unsigned long clk = 124999650;
    double fword0;
    unsigned long fword1;
    int t1;
    		
	//Define frequency word
    fword0 = (double) fx / clk * 4294967296;
    fword1 = (unsigned long) (fword0);
    
    DDS_CTRL_GPIO->ODR &= ~(1 << FQ_UD); //FQ_UD lo
    
    //Send W0
    DDS_DATA_GPIO->ODR = 0; //All bits are 0
    DDS_CTRL_GPIO->ODR |= (1 << W_CLK);  
    DDS_CTRL_GPIO->ODR &= ~(1 << W_CLK);   
    
    for(t1 = 3; t1 >= 0; t1--)
    {
		DDS_DATA_GPIO->ODR = (fword1 >> (8 * t1)) & 0xFF;
		
        DDS_CTRL_GPIO->ODR |= (1 << W_CLK);  
        DDS_CTRL_GPIO->ODR &= ~(1 << W_CLK);   
	}	
	
    DDS_CTRL_GPIO->ODR |= (1 << FQ_UD); //FQ_UD hi
	       
}

  //////////////////////
 //   Main program   //
//////////////////////
int main(void)
{
	int t1;
	unsigned long f;
	
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
    // Setup LED
    //////////////////////////////////////////
	//Turn on the GPIOC peripheral for LED
    RCC->AHB1ENR |= (1 << 2); //GPIOC enabale
	GPIOC->MODER |= (1 << (13 << 1));	//Set PC13 for output        
    
    ////////////////////////////////////////// 
    // Setup DDS
    //////////////////////////////////////////
	//Turn on the GPIOA peripheral for DDS
    RCC->AHB1ENR |= (1 << 0); //GPIOA enable
    RCC->AHB1ENR |= (1 << 1); //GPIOB enable 
    
    //Put pin A0..A7 in general purpose output mode (DATA-Port)
    for(t1 = 0; t1 < 8; t1 ++)
    {
		DDS_DATA_GPIO->MODER  |=  (1 << (t1 << 1));	
        DDS_DATA_GPIO->OSPEEDR =  (3 << (t1 << 1));	
    }
    
    //Put pin B0..B2 in general purpose output mode (CTRL-Port)
    DDS_CTRL_GPIO->MODER  |=  (1 << (W_CLK << 1));	
    DDS_CTRL_GPIO->OSPEEDR =  (3 << (W_CLK << 1));	
    DDS_CTRL_GPIO->MODER  |=  (1 << (FQ_UD << 1));	
    DDS_CTRL_GPIO->OSPEEDR =  (3 << (FQ_UD << 1));	
    DDS_CTRL_GPIO->MODER  |=  (1 << (RES << 1));	
    
    //Reset AD9850
    DDS_CTRL_GPIO->ODR |= (1 << RES);   //Bit set
    delay_us(1000);                //wait for > 20ns i. e. 1ms minimum time with _delay_s()
	DDS_CTRL_GPIO->ODR &= ~(1 << RES);  //Bit erase        
    set_frequency(27005000);
    set_frequency(27005000);
    
    while(1)
    {
		for(f = 100; f < 40000000; f++)
		{
			set_frequency(f);
			//Blinking LED shows proper operation
            GPIOC->ODR &= ~(1 << 13); //LED on 
            GPIOC->ODR |= (1 << 13); //LED off
        }    
        
    }
    return 0; 
}

