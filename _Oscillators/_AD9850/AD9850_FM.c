///////////////////////////////////////////////////////////////////  
/*                    AD9850 DDS driver software                 */
/*                    Extra: Frequency modulating the AD9850     */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         Black Pill Board                           */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      OCT 2021                                   */
///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include <string.h>
#include <stdlib.h>

// PIN and PORT definitions for AD9850 lines connected to PORT A
#define DDS_GPIO GPIOA
#define W_CLK 0   //PA0 blue
#define FQ_UD 1   //PA1 white
#define SDATA 2   //PA2 green
#define RES   3   //PA3 pink

//DDS
void set_frequency(unsigned long);
void spi_send_bit(int);
 
//MISC
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

  //////////////
 //    SPI   // 
//////////////
void spi_send_bit(int sbit)
{
    if(sbit) //Set/reset bit
	{
		DDS_GPIO->ODR |= (1 << SDATA);  
	}
	else
	{
		DDS_GPIO->ODR &= ~(1 << SDATA);
	}
	//Send clock signal
	DDS_GPIO->ODR |= (1 << W_CLK);  
    DDS_GPIO->ODR &= ~(1 << W_CLK);   
}

//Set AD9850 to desired frequency
void set_frequency(unsigned long fx)
{
    unsigned long clk = 125000000;
    double fword0;
    unsigned long  fword1;
    uint8_t t1;
    		
	//Define frequency word
    fword0 = (double) fx / clk * 4294967296;
    fword1 = (unsigned long) (fword0);
     
    //Send 32 frequency bits + 8 control & phase additional bits to DDS
	//Start sequence
	DDS_GPIO->ODR &= ~(1 << FQ_UD); //FQ_UD lo
       
	//W0...W31
	for(t1 = 0; t1 < 32; t1++)
    {
       spi_send_bit(fword1 & (1 << t1));
    }
	
	//W32...W39
	for(t1 = 0; t1 < 8; t1++)
	{
	    spi_send_bit(0);
	}	
	
	//Stop  sequence
    DDS_GPIO->ODR |= (1 << FQ_UD); //FQ_UD hi
	       
}
  //////////////
 //   MAIN   // 
//////////////
int main(void)
{
	unsigned long f = 5000000; //5 MHZ
	
	///////////////////////////////////
    // Set up LEDs - GPIOD 12,13,14,15
    ///////////////////////////////////
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~(0xFFU << 24);
    GPIOD->MODER |= (0x55 << 24);
    GPIOD->ODR    = 0x0000;

    //Turn on the GPIOA peripheral for DDS
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //RCC->AHB1ENR |= (1<<0);
    
    //DDS
    //Put pin A0..A3 in general purpose output mode
    DDS_GPIO->MODER  |=  (1 << (W_CLK << 1));	
    DDS_GPIO->MODER  |=  (1 << (FQ_UD << 1));	
    DDS_GPIO->MODER  |=  (1 << (SDATA << 1));	
    DDS_GPIO->MODER  |=  (1 << (RES << 1));	
    
    //Reset AD9850
    DDS_GPIO->ODR |= (1 << RES);   //Bit set
    delay(1);                      //wait for > 20ns i. e. 1ms minimum time with _delay_s()
	DDS_GPIO->ODR &= ~(1 << RES);  //Bit erase        
      		
	for(;;)
	{
		GPIOD->ODR |= (1 << 12); //Green led off, blink shows successful init
        delay(10);
            
        set_frequency(f);
        f += 100;
        if(f > 10000000)
        {
	        f = 5000000;
		}	
		DDS_GPIO->ODR &= ~(1 << SDATA);
        delay(10);
	}	
}
