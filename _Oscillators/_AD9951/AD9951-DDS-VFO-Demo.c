///////////////////////////////////////////////////////////////////  
/*                    AD9951 DDS driver software                 */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         F411E Discovery board by STM               */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include <string.h>
#include <stdlib.h>

// PIN and PORT definitions for AD9850 lines connected to PORT A
#define DDS_GPIO GPIOA
#define DDS_SDIO    0   //white
#define DDS_SCLK    1   //blue
#define DDS_IO_UD   2   //yellow
#define DDS_RESET   3   //gray
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
void spi_send_byte(unsigned int sbyte)
{
    int t1, x = (1 << 7);
	
	for(t1 = 0; t1 < 8; t1++)
	{
	    DDS_GPIO->ODR &= ~(1 << DDS_SCLK);  //DDS_SCLK lo
    	
        //Bit set or erase
	    if(sbyte & x)
	    {
		    DDS_GPIO->ODR |= (1 << DDS_SDIO);  
	    }
	    else
	    {
		    DDS_GPIO->ODR &= ~(1 << DDS_SDIO);  
	    }
	
        DDS_GPIO->ODR |= (1 << DDS_SCLK); //DDS_SCLK hi
		x >>= 1;
	}	
}

//Set frequency for AD9951 DDS
void set_frequency(unsigned long frequency)
{
    //unsigned long interfreq = 10E06; //Interfrequency of radio in Hz
    unsigned long f;
    unsigned long fword;
    int t1, shiftbyte = 24, resultbyte;
    unsigned long comparebyte = 0xFF000000;
	
	f = frequency; //Offset because of inaccuracy of crystal oscillator
	
    //Calculate frequency word
    //2³² / fClk = ----
    
    //Clock rate =  75MHz
    //fword = (unsigned long) f * 57.266230613;
    
    //Clock rate =  100MHz
    //fword = (unsigned long) f * 42.94967296;
        
    //Clock rate =  110MHz
    //fword = (unsigned long) f * 39.045157236;
    
    //Clock rate =  125MHz
    fword = (unsigned long) f * 34.358675; 
        	
	//Clock rate =  200MHz
    //fword = (unsigned long) f * 21.47478;  //..36448
    
    //Clock rate =  300MHz
    //fword = (unsigned long) f * 14.316557653;
    
    //Clock rate =  400MHz
    //fword = (unsigned long) f * 10.73741824;
		
	
    //Start transfer to DDS
    DDS_GPIO->ODR &= ~(1 << DDS_IO_UD); //DDS_IO_UD lo
    
	//Send instruction bit to set fequency by frequency tuning word
	spi_send_byte(0x04);	
	
    //Calculate and transfer the 4 bytes of the tuning word to DDS
    //Start with msb
    for(t1 = 0; t1 < 4; t1++)
    {
        resultbyte = (fword & comparebyte) >> shiftbyte;
        comparebyte >>= 8;
        shiftbyte -= 8;       
        spi_send_byte(resultbyte);	
    }    
	
	//End transfer sequence
    DDS_GPIO->ODR |= (1 << DDS_IO_UD); //DDS_IO_UD hi 
}

void set_clock_multiplier(void)
{
    //Start transfer to DDS
    DDS_GPIO->ODR &= ~(DDS_IO_UD); //DDS_IO_UD lo
    
	//Send CFR2
	spi_send_byte(0x01);
	
	//Multiply by 4
	spi_send_byte(0x00);
	spi_send_byte(0x00);
	spi_send_byte(0x24); //0x04 << 3
		
	//End transfer sequence
    DDS_GPIO->ODR |= (DDS_IO_UD); //DDS_IO_UD hi 
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
    DDS_GPIO->MODER  |=  (1 << (DDS_SCLK << 1));	
    DDS_GPIO->MODER  |=  (1 << (DDS_IO_UD << 1));	
    DDS_GPIO->MODER  |=  (1 << (DDS_SDIO << 1));	
    DDS_GPIO->MODER  |=  (1 << (DDS_RESET << 1));	
    
    //Reset AD9951
    delay(100);
	//Reset DDS (AD9951)
	DDS_GPIO->ODR |= (1 << DDS_RESET);  
	delay(100);
	DDS_GPIO->ODR &= ~(1 << DDS_RESET);  
    delay(100);
	DDS_GPIO->ODR |= (1 << DDS_RESET);  
	  		
	for(;;)
	{
		GPIOD->ODR |= (1 << 12); //Green led off, blink shows successful init
        delay(100);
            
        set_frequency(f);
        
		GPIOD->ODR &= ~(1 << 12);
        delay(100);
	}	
}
