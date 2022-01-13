///////////////////////////////////////////////////////////////////  
//         Receiver RFM12b ISM  for STM32F4                      //
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

volatile uint8_t device_addr = 0x78; //I2C address of OLED device

//RX char buffer USART1
#define MAXBUFFER 16

#define OLEDCMD 0x00   //Command follows
#define OLEDDATA 0x40  //Data follows

#define FONTW 6
#define FONTH 8

#define S_SETLOWCOLUMN           0x00
#define S_SETHIGHCOLUMN          0x10
#define S_PAGEADDR               0xB0
#define S_SEGREMAP               0xA0

#define S_LCDWIDTH               128
#define S_LCDHEIGHT              64 
  
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
extern "C" void USART1_IRQHandler(void);

  //////////////
 // O L E D  //
//////////////
//OLED
void oled_command(uint8_t value);
void oled_data(unsigned int*, unsigned int);
void oled_gotoxy(uint8_t, uint8_t);
void oled_cls(int);
void oled_init(void);
void oled_byte(uint8_t);
void oled_putchar1(unsigned int x, unsigned int y, uint8_t ch, int);
void oled_putnumber(int, int, long, int, int);
void oled_putstring(int, int, char*, int);
void oled_write_section(int, int, int, int);

//String
int int2asc(long num, int dec, char *buf, int buflen);

//I2C
void i2c_start(void);
void i2c_stop(void); 
void i2c_write(uint8_t, uint8_t);
void i2c_write_byte(int, int, uint8_t);
void i2c_write_bytes(uint8_t*, uint8_t);
uint8_t i2c_read(uint8_t); 

//Variables
char *msg_usart1;
int bufcnt1 = 0;
int scomplete1 = 0;
uint8_t rxdata;

//////////////////////////////////////
//   L   C   D   
//////////////////////////////////////
// Font 6x8 for OLED
const char font[485] = {
0x00,0x00,0x00,0x00,0x00, // 20 space ASCII table for NOKIA LCD: 96 rows * 5 bytes= 480 bytes
0x00,0x00,0x5f,0x00,0x00, // 21 ! Note that this is the same set of codes for character you
0x00,0x07,0x00,0x07,0x00, // 22 " would find on a HD44780 based character LCD.
0x14,0x7f,0x14,0x7f,0x14, // 23 # Also, given the size of the LCD (84 pixels by 48 pixels),
0x24,0x2a,0x7f,0x2a,0x12, // 24 $ the maximum number of characters per row is only 14.
0x23,0x13,0x08,0x64,0x62, // 25 %
0x36,0x49,0x55,0x22,0x50, // 26 &
0x00,0x05,0x03,0x00,0x00, // 27 '
0x00,0x1c,0x22,0x41,0x00, // 28 (
0x00,0x41,0x22,0x1c,0x00, // 29 )
0x14,0x08,0x3e,0x08,0x14, // 2a *
0x08,0x08,0x3e,0x08,0x08, // 2b +
0x00,0x50,0x30,0x00,0x00, // 2c ,
0x08,0x08,0x08,0x08,0x08, // 2d -
0x00,0x60,0x60,0x00,0x00, // 2e .
0x20,0x10,0x08,0x04,0x02, // 2f /
0x3e,0x51,0x49,0x45,0x3e, // 30 0
0x00,0x42,0x7f,0x40,0x00, // 31 1
0x42,0x61,0x51,0x49,0x46, // 32 2
0x21,0x41,0x45,0x4b,0x31, // 33 3
0x18,0x14,0x12,0x7f,0x10, // 34 4
0x27,0x45,0x45,0x45,0x39, // 35 5
0x3c,0x4a,0x49,0x49,0x30, // 36 6
0x01,0x71,0x09,0x05,0x03, // 37 7
0x36,0x49,0x49,0x49,0x36, // 38 8
0x06,0x49,0x49,0x29,0x1e, // 39 9
0x00,0x36,0x36,0x00,0x00, // 3a :
0x00,0x56,0x36,0x00,0x00, // 3b ;
0x08,0x14,0x22,0x41,0x00, // 3c <
0x14,0x14,0x14,0x14,0x14, // 3d =
0x00,0x41,0x22,0x14,0x08, // 3e >
0x02,0x01,0x51,0x09,0x06, // 3f ?
0x32,0x49,0x79,0x41,0x3e, // 40 @
0x7e,0x11,0x11,0x11,0x7e, // 41 A
0x7f,0x49,0x49,0x49,0x36, // 42 B
0x3e,0x41,0x41,0x41,0x22, // 43 C
0x7f,0x41,0x41,0x22,0x1c, // 44 D
0x7f,0x49,0x49,0x49,0x41, // 45 E
0x7f,0x09,0x09,0x09,0x01, // 46 F
0x3e,0x41,0x49,0x49,0x7a, // 47 G
0x7f,0x08,0x08,0x08,0x7f, // 48 H
0x00,0x41,0x7f,0x41,0x00, // 49 I
0x20,0x40,0x41,0x3f,0x01, // 4a J
0x7f,0x08,0x14,0x22,0x41, // 4b K
0x7f,0x40,0x40,0x40,0x40, // 4c L
0x7f,0x02,0x0c,0x02,0x7f, // 4d M
0x7f,0x04,0x08,0x10,0x7f, // 4e N
0x3e,0x41,0x41,0x41,0x3e, // 4f O
0x7f,0x09,0x09,0x09,0x06, // 50 P
0x3e,0x41,0x51,0x21,0x5e, // 51 Q
0x7f,0x09,0x19,0x29,0x46, // 52 R
0x46,0x49,0x49,0x49,0x31, // 53 S
0x01,0x01,0x7f,0x01,0x01, // 54 T
0x3f,0x40,0x40,0x40,0x3f, // 55 U
0x1f,0x20,0x40,0x20,0x1f, // 56 V
0x3f,0x40,0x38,0x40,0x3f, // 57 W
0x63,0x14,0x08,0x14,0x63, // 58 X
0x07,0x08,0x70,0x08,0x07, // 59 Y
0x61,0x51,0x49,0x45,0x43, // 5a Z
0x00,0x7f,0x41,0x41,0x00, // 5b [
0x02,0x04,0x08,0x10,0x20, // 5c Yen Currency Sign
0x00,0x41,0x41,0x7f,0x00, // 5d ]
0x04,0x02,0x01,0x02,0x04, // 5e ^
0x40,0x40,0x40,0x40,0x40, // 5f _
0x00,0x01,0x02,0x04,0x00, // 60 `
0x20,0x54,0x54,0x54,0x78, // 61 a
0x7f,0x48,0x44,0x44,0x38, // 62 b
0x38,0x44,0x44,0x44,0x20, // 63 c
0x38,0x44,0x44,0x48,0x7f, // 64 d
0x38,0x54,0x54,0x54,0x18, // 65 e
0x08,0x7e,0x09,0x01,0x02, // 66 f
0x0c,0x52,0x52,0x52,0x3e, // 67 g
0x7f,0x08,0x04,0x04,0x78, // 68 h
0x00,0x44,0x7d,0x40,0x00, // 69 i
0x20,0x40,0x44,0x3d,0x00, // 6a j
0x7f,0x10,0x28,0x44,0x00, // 6b k
0x00,0x41,0x7f,0x40,0x00, // 6c l
0x7c,0x04,0x18,0x04,0x78, // 6d m
0x7c,0x08,0x04,0x04,0x78, // 6e n
0x38,0x44,0x44,0x44,0x38, // 6f o
0x7c,0x14,0x14,0x14,0x08, // 70 p
0x08,0x14,0x14,0x18,0x7c, // 71 q
0x7c,0x08,0x04,0x04,0x08, // 72 r
0x48,0x54,0x54,0x54,0x20, // 73 s
0x04,0x3f,0x44,0x40,0x20, // 74 t
0x3c,0x40,0x40,0x20,0x7c, // 75 u
0x1c,0x20,0x40,0x20,0x1c, // 76 v
0x3c,0x40,0x30,0x40,0x3c, // 77 w
0x44,0x28,0x10,0x28,0x44, // 78 x
0x0c,0x50,0x50,0x50,0x3c, // 79 y
0x44,0x64,0x54,0x4c,0x44, // 7a z
0x00,0x08,0x36,0x41,0x00, // 7b <
0x00,0x00,0x7f,0x00,0x00, // 7c |
0x00,0x41,0x36,0x08,0x00, // 7d >
0x10,0x08,0x08,0x10,0x08, // 7e Right Arrow ->
0x78,0x46,0x41,0x46,0x78, // 7f Left Arrow <-
0x00,0x06,0x09,0x09,0x06};  // 80 °

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

  //////////////////////////////
 // USART1 interrupt handler //
//////////////////////////////
//Declare as 'extern "C"' when compiler in 
//c++ mode to get correct name of intr function
extern "C" void USART1_IRQHandler(void)
{
	GPIOC->ODR ^= (1 << 13);            //LED on		
		
	rxdata = USART1->DR;
	
	if((rxdata >= 32) && (rxdata < 128))
	{
	    msg_usart1[bufcnt1++] = rxdata;
	    msg_usart1[bufcnt1] = 0;
	    if(bufcnt1 > MAXBUFFER)
	    {
			bufcnt1 = 0;
			scomplete1 = 1;
		}	
	}    
	
	if(rxdata == 13)
	{
		scomplete1 = 1;
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


////////////////////////////////
// I2C function defintions
///////////////////////////////
void i2c_start(void) 
{
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB));
}

void i2c_stop(void) 
{
    I2C1->CR1 |= I2C_CR1_STOP;
    while(!(I2C1->SR2 & I2C_SR2_BUSY));
}

//A general version of the I2C-TX function
//Fixed number of bytes (2)
void i2c_write(uint8_t data0, uint8_t data1) 
{
    //Send start signal
    i2c_start();

    //Send device address
    I2C1->DR = device_addr; 
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Perform one dummy read to clear flags
    (void)I2C1->SR2;

    //Send register address for device
    I2C1->DR = data0;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send data
    I2C1->DR = data1;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send stop signal
    i2c_stop();
}

//Variable number of bytes (2)
void i2c_write_bytes(uint8_t *data, uint8_t n) 
{
	int t1 = 0;
	    
    //Send start signal
    i2c_start();

    //Send device address
    I2C1->DR = device_addr; 
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Perform one dummy read to clear flags
    (void)I2C1->SR2;

    for(t1 = 0; t1 < n; t1++)
    {
		//Send data
        I2C1->DR = data[t1];
        while (!(I2C1->SR1 & I2C_SR1_BTF));
    }    
    
    //Send stop signal
    i2c_stop();
}

//Write 1 byte to screen
void i2c_write_byte(int x, int y, uint8_t data) 
{
	//Page mapping
	//Col and line adr for OLED 68x32
	oled_command(0x21); //COL (+32)
	oled_command(x + 32); 
	oled_command(x + 32); 
	
	oled_command(0x22); //PAGE (+2)
	oled_command(y + 2); 
	oled_command(y + 2); 
	    
    //Send start signal
    i2c_start();

    //Send device address
    I2C1->DR = device_addr; 
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Perform one dummy read to clear flags
    (void)I2C1->SR2;
    
    //Send DATA info
    I2C1->DR = OLEDDATA;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send data
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
    i2c_stop();
}

////////////////////////////////
// OLED routines
///////////////////////////////
//Send comand to OLED
void oled_command(uint8_t value)
{
   uint8_t d[2];
   
   d[0] = OLEDCMD;
   d[1] = value;
      
   i2c_write_bytes(d, 2);
} 

//Send a 'number' bytes of data to display - from RAM
void oled_data(unsigned int *data, unsigned int n)
{
   unsigned int t1;
   uint8_t d[n + 1];
   
   d[0] = OLEDDATA;
   for(t1 = 1; t1 < n + 1; t1++)
   {
	   d[t1] = data[t1 - 1];
   }	   
   i2c_write_bytes(d, n + 1);
}

//Set "cursor" to current position to screen
void oled_gotoxy(uint8_t x, uint8_t y)
{
   int x2 = x;	
   
   uint8_t d[4];
   d[0] = OLEDCMD;
   d[1] = S_PAGEADDR + y;
   d[2] = S_SETLOWCOLUMN + (x2 & 0x0F);
   d[3] = S_SETHIGHCOLUMN + ((x2 >> 4) & 0x0F);
   i2c_write_bytes(d, 4); 
}

void oled_cls(int invert)
{
    unsigned int row, col;
    uint8_t *d;
    d = (uint8_t*) malloc(S_LCDWIDTH + 1);
    int cnt;
    
    //Fill OLED memory with 0 or 255
    for(row = 0; row < S_LCDHEIGHT / 8; row++)
    {
		cnt = 0;
        d[cnt++] = OLEDDATA;
        for(col = 0; col < S_LCDWIDTH; col++)
        {
            if(!invert)
            {
                d[cnt++] = 0; //normal
            }   
            else
            {
                d[cnt++] = 255; //inverse
            } 
        }
        oled_gotoxy(0, row); //Set OLED address
        i2c_write_bytes(d, cnt);
    }
    oled_gotoxy(0, 0); //Return to 0, 0
    free(d);
}

//Write number of bitmaps to one row of screen
void oled_write_section(int x1, int x2, int row, int number)
{
    int t1;
    
    int n = x2 - x1;
    uint8_t *d;
    d = (uint8_t*) malloc(n);
    d[0] = OLEDDATA;
    int c = 1;

    for(t1 = x1; t1 < x2; t1++)
    {
       d[c++] = number; //Fill array
    }    
    
    oled_gotoxy(x1, row);
    i2c_write_bytes(d, n);
    free(d);
}


//Initialize OLED
void oled_init(void)
{
    oled_command(0xae);
	
    oled_command(0xa8); //Multiplex ratio
    oled_command(0x2F);
	
    oled_command(0xd3); //Set display offset
    oled_command(0x00);
    
    oled_command(0x40 + 0x00); //Set display start line
    
    oled_command(0xa1);
    	
	oled_command(0x20); //Adressing mode
    oled_command(0x00); //HOR
	
    oled_command(0xc0);
    oled_command(0xc8);
    
    oled_command(0xda);
    oled_command(0x12);
    
    oled_command(0x81);
    oled_command(0xfF);
    oled_command(0xa4); //Display ON with RAM content
    oled_command(0xa6); //Normal display (Invert display = A7)
    oled_command(0xd5);
    oled_command(0x80);
    oled_command(0x8d);
    oled_command(0x14);
	//oled_command(0x40); //Set display start line
    oled_command(0xAF); //Display ON
   
} 

//Write 1 byte pattern to screen using vertical orientation 
void oled_byte(uint8_t value)
{
   uint8_t d[1];
   d[0] = value;
   i2c_write_bytes(d, 1); //Device address  
}

//Write character to screen (normal size);
void oled_putchar1(unsigned int x, unsigned int y, uint8_t ch, int invert)
{
	int p, t1;
    char ch2;
	int c = x;
	    
    p = (5 * ch) - 160;
    for(t1 = 0; t1 < 5; t1++)
    { 
	    if(!invert)
		{
	        ch2 = font[p + t1];
		}
		else
		{
	        ch2 = ~(font[p + t1]);
		}
		
        i2c_write_byte(c++, y, ch2);
    }
	
	if(!invert)
	{
	    i2c_write_byte(c, y, 0x00);
	}
	else
	{
	    i2c_write_byte(c, y, 0xFF);
	}
}		

//Print string in given size
//lsize=0 => normal height, lsize=1 => double height
void oled_putstring(int col, int row, char *s, int inv)
{
    int c = col;
	
	while(*s)
	{
		oled_putchar1(c, row, *s++, inv);
		c += FONTW;
	}
}

//Print an integer/long to OLED
void oled_putnumber(int col, int row, long num, int dec, int inv)
{
    char *s = (char*)malloc(16);
	if(s != NULL)
	{
	    int2asc(num, dec, s, 16);
	    oled_putstring(col, row, s, inv);
	    free(s);
	}	
}

/////////////////////////////////
//
// STRING FUNCTIONS
//
////////////////////////////////
//INT 2 ASC
int int2asc(long num, int dec, char *buf, int buflen)
{
    int i, c, xp = 0, neg = 0;
    long n, dd = 1E09;

    if(!num)
	{
	    *buf++ = '0';
		*buf = 0;
		return 1;
	}	
		
    if(num < 0)
    {
     	neg = 1;
	    n = num * -1;
    }
    else
    {
	    n = num;
    }

    //Fill buffer with \0
    for(i = 0; i < buflen; i++)
    {
	    *(buf + i) = 0;
    }

    c = 9; //Max. number of displayable digits
    while(dd)
    {
	    i = n / dd;
	    n = n - i * dd;
	
	    *(buf + 9 - c + xp) = i + 48;
	    dd /= 10;
	    if(c == dec && dec)
	    {
	        *(buf + 9 - c + ++xp) = '.';
	    }
	    c--;
    }

    //Search for 1st char different from '0'
    i = 0;
    while(*(buf + i) == 48)
    {
	    *(buf + i++) = 32;
    }

    //Add minus-sign if neccessary
    if(neg)
    {
	    *(buf + --i) = '-';
    }

    //Eleminate leading spaces
    c = 0;
    while(*(buf + i))
    {
	    *(buf + c++) = *(buf + i++);
    }
    *(buf + c) = 0;
	
	return c;
}

  ///////////////////
 // Main programm //
///////////////////
int main(void)
{
    int t1, x = 0, y = 0;
    
	//////////////////////////////////////////
    // Setup LED w. GPIOC
    //////////////////////////////////////////
    RCC->AHB1ENR |= (1 << 2);
    GPIOC->MODER |= (1 << (13 << 1));
    
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
	rfm12b_setfreq((433.92 - 430.0)/0.0025);
	rfm12b_setbandwidth(4, 1, 4);			// 200kHz band width, -6dB gain, DRSSI threshold: -79dBm 
	rfm12b_setbaud(19200);					// 19200 baud
	rfm12b_setpower(4, 6);			  	    // 1mW output power, 120kHz frequency shift
    	 
	spi_send_word(0x82C8); //2. Power Management Command, RX on
	 
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
        
    ///////////////////////
    // RX Section USART1 //
    ///////////////////////
    //Setup the NVIC to enable interrupts.
    //Use 4 bits for 'priority' and 0 bits for 'subpriority'.
    NVIC_SetPriorityGrouping(0);
    //UART receive interrupts should be high priority.
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(0, 1, 0));
    NVIC_EnableIRQ(USART1_IRQn);
        
    //TX and RX enable
    USART1->CR1 |= (USART_CR1_RXNEIE);
    USART1->CR1 |= (1 << 13);  //Enable USART1 - UE, bit 13    

    //////////////////////////////////////////
    // Set up I2C - GPIOB 6, 9 for OLED
    //////////////////////////////////////////
    //Enable I2C clock
    RCC->APB1ENR |= (1 << 21); //I2C1 enable

    //Set up I2C PB6 and PB9 pins
    RCC->AHB1ENR |= (1 << 1);         //GPIOB enable
    GPIOB->MODER &= ~(3 << (6 << 1)); //PB6 as SCK
    GPIOB->MODER |=  (2 << (6 << 1)); //Alternate function
    GPIOB->OTYPER |= (1 << 6);        //Open-drain
    GPIOB->MODER &= ~(3 << (9 << 1)); //PB9 as SDA
    GPIOB->MODER |=  (2 << (9 << 1)); //Alternate function
    GPIOB->OTYPER |= (1 << 9);        //Open-drain

    //Choose AF4 for I2C1 in Alternate Function registers
    GPIOB->AFR[0] |= (4 << (6 << 2));     //for PB6
    GPIOB->AFR[1] |= (4 << ((9 - 8) << 2)); //for PB9

    //Reset and clear register
    I2C1->CR1 = (1 << 15); //I2C1 SWRST
    I2C1->CR1 = 0;

    I2C1->CR2 |= (10 << 0); //10Mhz peripheral clock
    I2C1->CCR |= (50 << 0); 
    
    //Maximum rise time defintion
    I2C1->TRISE |= (11 << 0); //Program for value=11 => 100khz
        
    //Enable I2C    
    I2C1->CR1 |= (1 << 0); //PE     
    
    //Start OLED
    delay_ms(500);
    oled_init();
    oled_cls(0);
    
    //Get buffer space in memory
    msg_usart1 = (char*)malloc(MAXBUFFER);
    
    //Init text buffer
    for(t1 = 0; t1 < MAXBUFFER; t1++)
    {
		msg_usart1[t1] = 0;
    }
    
    oled_putstring(0, 0, (char*)"RFM12b RX", 0);
    
    while(1)
    { 
		if(scomplete1) //Transmission from RFM12b completed -> process string
        {
			oled_putstring(x * FONTW, y, msg_usart1, 0);
		    if(x < 9)
		    {
				x++;
			}
			else
			{
				x = 0;
				if(y < 3)
		        {
				    y++;
			    }
			    else
			    {
				    x = 0;
				    y = 0;
				    oled_cls(0);
			    }
			}		
						
			scomplete1 = 0;
			bufcnt1 = 0;
			for(t1 = 0; t1 < MAXBUFFER; t1++)
            {
		        msg_usart1[t1] = 0;
            }
		}	
		
	}
	return 0;
}
