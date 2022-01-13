///////////////////////////////////////////////////////////////////  
//         Transmitter RFM12b ISM  for STM32F4                   //
///////////////////////////////////////////////////////////////////
//Purpose: This demonstration uses an off-the-shelf ISM 433MHz   //
//module to establish data transfer via short range              //
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         WeAct Blackpill board                      */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JAN 2022                                   */
///////////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <string.h>
#include <stdlib.h>

//Output lines
//TRX port & lines (PORTD)
#define SDI  2   //YELLOW
#define SCK  1   //BLUE
#define NSEL 0  //VIOLET

#define SDO  3   //BROWN
#define FSK  4   //GREY
#define SPI_GPIO GPIOA
#define TWAIT 100 //Wait Microceonds for SPI
  
  ////////////
 //  MISC  //
////////////
int main(void);
static void delay_ms(unsigned int); 
static void delay_us(unsigned int); 

  //////////////
 // RFM12b   //
//////////////
int spi_send_word(unsigned short);
void rfm12b_setbandwidth(unsigned char, unsigned char, unsigned char);
void rfm12b_setfreq(int);
void rfm12b_setbaud(int);
void rfm12b_setpower(unsigned char, unsigned char);

  //////////////
 //  USART1  //
//////////////
void usart1_send(char*);

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

static void delay_us(unsigned int time) 
{
    for (unsigned int i = 0; i < time; i++)
    {
        for (volatile unsigned int j = 0; j < 2; j++);
    }    
}

///////////////
//    SPI    //
///////////////
int spi_send_word(unsigned short txdata)
{
	int t1 = 0;
	int r = 0;
	    
	SPI_GPIO->ODR &= ~(1 << NSEL);
	delay_us(TWAIT);   	
	
	//Send ID byte to define register
	for(t1 = 15; t1 >= 0; t1--)
    {
        SPI_GPIO->ODR &= ~(1 << SCK);   
        delay_us(TWAIT);  

	    if((1 << t1) & txdata)
        {
            SPI_GPIO->ODR |= (1 << SDI);  
        }
        else
        {
            SPI_GPIO->ODR &= ~(1 << SDI);
        }
        delay_us(TWAIT);   
        
        SPI_GPIO->ODR |= (1 << SCK);  
		delay_us(TWAIT);  
        
        if(SPI_GPIO->IDR & (1 << SDO))
        { 
			r += (1 << t1);
		}	
		
        SPI_GPIO->ODR &= ~(1 << SCK);   
        delay_us(TWAIT);  
    }
    
    SPI_GPIO->ODR |= (1 << NSEL); 
    
    return r;
}

void rfm12b_setbandwidth(unsigned char bandwidth, unsigned char gain, unsigned char drssi)
{
	spi_send_word(0x9400 | ((bandwidth&7) << 5) | ((gain & 3) <<3 ) | (drssi & 7));
}

void rfm12b_setfreq(int freq)
{	
	if (freq < 96)				// 430,2400MHz
	{
		freq = 96;
	}	
	else if (freq > 3903)			// 439,7575MHz
	{
		freq = 3903;
	}	
	spi_send_word(0xA000 | freq);
}

void rfm12b_setbaud(int baud)
{
	if (baud < 663)
	{
		spi_send_word(0xC680|((43104/5400)-1));;
	}
		
	if (baud < 5400)					// Baudrate= 344827,58621/(R+1)/(1+CS*7)
	{
		spi_send_word(0xC680|((43104/baud)-1));
	}
	else
	{
		spi_send_word(0xC600|((344828UL/baud)-1));
	}	
}

void rfm12b_setpower(unsigned char power, unsigned char mod)
{	
	spi_send_word(0x9800|(power&7)|((mod&15)<<4));
}

  //////////////////////////////
 // USART1 transmit function //
//////////////////////////////
void usart1_send(char *xmsg)
{
	unsigned int t1;
	
	for(t1 = 0; t1 < strlen(xmsg); t1++)
    {
	    USART1->DR = xmsg[t1]; //Transmit character
        while(!(USART1->SR & (1 << 6))); //Wait for transmission to be completed
    }
    USART1->DR = 10; //Transmit character
    while(!(USART1->SR & (1 << 6))); //Wait for transmission to be completed
    USART1->DR = 13; //Transmit character
    while(!(USART1->SR & (1 << 6))); //Wait for transmission to be completed
}

  ///////////////////
 // Main programm //
///////////////////
int main(void)
{
    uint8_t ch;
    char *s;
    int t1;
     
    /////////////////////////////////////////////
    // Set SystemClock to 50 MHz with 25 MHz HSE
    //////////////////////////////////////////////
    FLASH->ACR |= (1 << 1);                     //2 wait states for 96+ MHz
    RCC->CR |= (1 << 16);                       //Activate external clock (HSE: 25 MHz)
    while ((RCC->CR & (1 << 17)) == 0);         //Wait until HSE is ready
    
    RCC->PLLCFGR |= (1 << 22);                  //PLL source is HSE
    
    RCC->PLLCFGR &= ~0b11111;                   //Reset bits 4..0, then set PLL-M: VCO input frequency = PLL input clock frequency / PLLM with 2 ≤ PLLM ≤ 63
    RCC->PLLCFGR |= 20;                         // -> f.VCO.in = 25MHz / 20 = 1.25MHz
                                                
    RCC->PLLCFGR &= ~(32704 << 6);
    RCC->PLLCFGR |= 160 << 6;                   //PLL-N: f.VCO.out = f.VCO.in * 160 = 250MHz
    
    RCC->PLLCFGR &= ~(0b11 << 16);              //Reset - PLL-P: Main PLL (PLL) division factor for main system clock
    RCC->PLLCFGR &= ~(0b01 << 16);              //f.PLL.output.clock = f.VCO.out / 4 = 50MHz
                                                
    RCC->PLLCFGR &= ~(0b111 << 24);             //Main PLL (PLL) division factor for USB OTG FS, SDIO and 
                                                //random number generator clocks (f<=48MHz for RNG!, 48MHz for USB)
    RCC->PLLCFGR |= (4 << 24);                  //PLL-Q: f.VCO.out / 4 = 25MHz
        
    RCC->CR |= (1 << 24);                       //Activate PLL (Output: 100 MHz)
    while ((RCC->CR & (1 << 25)) == 0);         //Wait until PLL is ready
    
    //Division by 2 of clk signal       
    RCC->CFGR |= (0b1000 << 4)                  //AHB divider:  /2
              | (0b100 << 10)                   //APB1 divider: /2
              | (0b100 << 13);                  //APB2 divider: /2
               
    RCC->CFGR |= (1 << 1);                      //Switching to PLL clock source
        
	//////////////////////////////////////////
    // Setup LED w. GPIOC
    //////////////////////////////////////////
    RCC->AHB1ENR |= (1 << 2);
    GPIOC->MODER |= (1 << (13 << 1));
    
    //////////////////////////////////////////
    // Setup SPI for RFM12B GPIOA
    //////////////////////////////////////////
	RCC->AHB1ENR |= (1 << 0);
		  
	SPI_GPIO->MODER  |=  (1 << (NSEL << 1));	
    SPI_GPIO->MODER  |=  (1 << (SCK << 1));	    
    SPI_GPIO->MODER  |=  (1 << (SDI << 1));	    
    SPI_GPIO->MODER  &=  ~(1 << (SDO << 1));	
    //SPI_GPIO->MODER  |=  (1 << (FSK << 1));	
    
    //////////////////////////////////////////
    //Init Si4421 on RFM12b module
    //////////////////////////////////////////
    spi_send_word(0x8017);      //NO FIFO => !EL,!EF,433band,12.0pF   (1. Configuration Setting Command)
    spi_send_word(0x8239);      // !er,!ebb,ET,ES,EX,!eb,!ew,DC, FIFO (2. Power Management Command)
    spi_send_word(0xA640);      //434MHz freq. definition (n=1600)    (3. Frequency Setting Command)
    spi_send_word(0xC647);      //4.8kbps                             (4. Data Rate Command)
    spi_send_word(0x94A0);      //VDI,FAST,134kHz,0dBm,-103dBm        (5. Receiver Control Command)
    spi_send_word(0xC2AC);      //AL,!ml,DIG,DQD4                     (6. Data Filter Command)
    spi_send_word(0xC483);      //@PWR,NO RSTRIC,!st,!fi,OE,EN        (10. AFC Command) 
    spi_send_word(0x9820);      // !mp,45kHz,MAX OUT*/                (11. TX Configuration Control Command)
    spi_send_word(0xCC77);      //OB1,OB0, LPX,!ddy,DDIT,BW0          (12. PLL Setting Command)
    spi_send_word(0xE000);      //NOT USED                            (14. Wake-Up Timer Command)
    spi_send_word(0xC800);      //NOT USED                            (15. Low Duty-Cycle Command)
    spi_send_word(0xC040);      //1.66MHz,2.2V                        (16. Low Battery Detector and Microcontroller Clock Divider Command)  
 
    delay_ms(500);
    spi_send_word(0x8208);			        // Turn on crystal
	rfm12b_setfreq((433.92-430.0)/0.0025);
	rfm12b_setbandwidth(4, 1, 4);			// 200kHz band width, -6dB gain, DRSSI threshold: -79dBm 
	rfm12b_setbaud(19200);					// 19200 baud
	rfm12b_setpower(4, 6);			  	    // 1mW output power, 120kHz frequency shift
    	
	spi_send_word(0x8238);                  //Power Management Command TX on
	 
	//////////////////////////////////////////////
    // Setup USART1 for TX data
    //////////////////////////////////////////////
    RCC->APB2ENR |= (1 << 4);                 //Enable USART1 clock, bit 4 on APB2ENR
    RCC->AHB1ENR |= (1 << 0);                 //Eenable GPIOA clock, bit 0 on AHB1ENR

    //Set pin modes as alternate mode 7 (pins A2 and A3)
    //USART1 TX and RX pins are PA9 (TX) and PA10 (RX) respectively
    GPIOA->MODER &= ~(0b1111 << 18); //Reset bits 19, 18 for PA9 and 21, 20 for PA10
    GPIOA->MODER |=  (0b1010 << 18); //Set   bits 19, 18 for PA9 and 21, 20 for PA10 to alternate mode (10)

    //Set pin modes as fast speed
    GPIOA->OSPEEDR |= 0b1010 << 18; //Set PA9 PA10 to fast speed mode (0b10)

    //Choose AF7 for USART1 in Alternate Function registers
    GPIOA->AFR[1] |= (0x07 << 4); // for pin A9
    GPIOA->AFR[1] |= (0x07 << 8); // for pin A10

    //USART1 word length M, bit 12
    USART1->CR1 |= (0 << 12); // 0 - 1,8,n

    //USART1 parity control, bit 9
    USART1->CR1 |= (0 << 9); // 0 - no parity

    //USART1 TX enable, TE bit 3 
    USART1->CR1 |= (1 << 3);

    // USART1 rx enable, RE bit 2
    USART1->CR1 |= (1 << 2);

    //Baud = fCK / (8 * (2 - OVER8) * USARTDIV)
    //-> USARTDIV = fCK / ((8 * (2 - OVER8) * Baud)
    //-> USARTDIV = 25000000 / ( 16 * 9600)= 156.25
    //Mantissa : 156
    //Fraction : 16*.25 = 4 (multiply fraction with 16)
    
    USART1->BRR |= (156 << 4); // 12-bit mantissa
    USART1->BRR |= 4;          // 4-bit fraction 
  
    //TX and RX enable
    USART1->CR1 |= (USART_CR1_RXNEIE);
    USART1->CR1 |= (1 << 13);  //Enable USART1 - UE, bit 13    
    
    ch = 33;
    s = (char*)malloc(8);
    for(t1 = 0; t1 < 8; t1++)
    {
		s[t1] = 0;
	}
		
    while(1)
    { 
		s[0] = ch;
		if(ch < 128)
		{
			ch++;
		}
		else	
		{
			ch = 33;
		}
		
		usart1_send((char*)s);
		delay_ms(800);
		GPIOC->ODR &= ~(1 << 13);            //LED on
		delay_ms(100);
		GPIOC->ODR |= (1 << 13);            //LED off
		delay_ms(100);
	}
	return 0;
}
