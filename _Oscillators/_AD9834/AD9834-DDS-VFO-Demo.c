///////////////////////////////////////////////////////////////////  
/*                    AD9834 DDS driver software                 */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         STM32VET6 board                            */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"

//PIN and DDS_GPIO->ODR definitions for AD9834
//SPI lines connected to PORTA
#define DDS_GPIO GPIOA
#define FSYNC 0 //violet
#define SDATA 1 //green
#define SCLK  2 //white 

//DDS
void spi_send_bit(int);
void spi_stop(void);
void spi_start(void);
void set_frequency(unsigned long);

//MISC
static void delay(unsigned int);

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
void spi_start(void)
{
	DDS_GPIO->ODR |= (1 << SCLK);      //SCLK  hi
    DDS_GPIO->ODR &= ~(1 << FSYNC);    //FSYNC lo
}

void spi_stop(void)
{
	DDS_GPIO->ODR |= (1 << FSYNC);    //FSYNC hi
}

void spi_send_bit(int sbit)
{
	
    if(sbit)
	{
		DDS_GPIO->ODR |= (1 << SDATA);  //SDATA hi
	}
	else
	{
		DDS_GPIO->ODR &= ~(1 << SDATA);  //SDATA lo
	}
	DDS_GPIO->ODR |= (1 << SCLK);     //SCLK hi
	DDS_GPIO->ODR &= ~((1 << SCLK));  //SCLK lo
}

void set_frequency(unsigned long f)
{

    double fword0;
    long fword1, x;
    int l[] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int m[] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t t1;
    
    //Define frequency data
    //Equation: 2.440322327 = 268435456 / f.clock
    fword0 = (double) 3.579139413 * f; // 75MHz
    fword1 = (long) fword0;

    //Transfer frequency word to byte array
    x = (1 << 13);      //2^13
    for(t1 = 2; t1 < 16; t1++)
    {
		if(fword1 & x)
	    {
			l[t1] = 1;
	    }
	    x >>= 1;
    }
    
    x = (1L << 27);  //2^27
    for(t1 = 2; t1 < 16; t1++)
    {
	    if(fword1 & x)
	    {
	        m[t1] = 1;
	    }
	    x >>= 1;
    }
    ////////////////////////////////////////
    
    //Transfer to DDS
    //Send start command
    spi_start();
    for(t1 = 15; t1 >= 0; t1--)
    {
       spi_send_bit(0x2000 & (1 << t1));
    }
    spi_stop();
        
    //Transfer frequency word	
    //LS-WORD
    spi_start();
    for(t1 = 0; t1 < 16; t1++)
    {
       spi_send_bit(l[t1]);
    }
    spi_stop();
	
	//MS-WORD
	spi_start();
    for(t1 = 0; t1 < 16; t1++)
    {
       spi_send_bit(m[t1]);
    }
    spi_stop();
}

  //////////////
 //   MAIN   // 
//////////////
int main(void)
{
	unsigned long f = 1000000; //1 MHZ output frequency
	 
	delay(10);
	 
	//Turn on the GPIOA peripheral for DDS
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //RCC->AHB1ENR |= (1<<0);
    
    //DDS
    //Put pin 0..2 in general purpose output mode
    DDS_GPIO->MODER |= (1 << (FSYNC << 1));	
    DDS_GPIO->MODER |= (1 << (SDATA << 1));	
    DDS_GPIO->MODER |= (1 << (SCLK << 1));	
    DDS_GPIO->ODR = 0;
    
    //LED
    DDS_GPIO->MODER |= (1 << (6 << 1));	
    DDS_GPIO->MODER |= (1 << (7 << 1));	
    
    set_frequency(f);
    set_frequency(f);
    
	for(;;)
	{
		GPIOA->ODR |= (1 << 6); //Green led off, blink shows successful init
		GPIOA->ODR &= ~(1 << 7);
        
        set_frequency(f);
        delay(10);
                
		GPIOA->ODR &= ~(1 << 6);
		GPIOA->ODR |= (1 << 7);
	}	
}
