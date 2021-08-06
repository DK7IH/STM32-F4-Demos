///////////////////////////////////////////////////////////////////  
/*                    OLED SSD1306 driver software (Text only)   */
//                    Display 64x32                              // 
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         F411E Discovery board by STM               */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <stdlib.h>

volatile uint8_t device_addr = 0x78; //I2C address of device

#define OLEDCMD 0x00   //Command follows
#define OLEDDATA 0x40  //Data follows

#define FONTW 6
#define FONTH 8

#define S_SETLOWCOLUMN           0x00
#define S_SETHIGHCOLUMN          0x10
#define S_PAGEADDR               0xB0
#define S_SEGREMAP               0xA0

#define S_LCDWIDTH               64
#define S_LCDHEIGHT              32 

//////////////////////////////////////
//   L   C   D   
//////////////////////////////////////
// Font 6x8 for OLED
// Font 5x8 for OLED
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

///////////////////////////
//Function declarations
///////////////////////////
//OLED
void oled_command(uint8_t value);
void oled_data(unsigned int*, unsigned int);
void oled_gotoxy(uint8_t, uint8_t);
void oled_cls();
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

//MISC
int main(void);
static void delay (unsigned int);

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
    //Perform one read to clear flags
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

//Variable number of bytes
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
	int t1;
	
	//Page mapping
	oled_command(0x21); //COL
	oled_command(x); 
	oled_command(x); 
	
	oled_command(0x22); //PAGE
	oled_command(y); 
	oled_command(y); 
	    
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

//Clear screen
void oled_cls(void)
{

    int x, y;
	for(x = 0; x < 128; x++)
	{
	    for(y = 0; y < 4; y++)
		{
		    i2c_write_byte(x, y, 0);
		}
    }	

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
	
    oled_command(0xa8);//Multiplex ratio
    oled_command(0x3F);
	
    oled_command(0xd3);
    oled_command(0x00);
    oled_command(0x40);
    oled_command(0xa0);
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

//Print one character in normal size to OLED
void oled_putchar1(int col, int row, char ch1, int inv)
{ 
    int p, t1;
    char ch2;
	int c = col;
	    
    p = (5 * ch1) - 160;
    for(t1 = 0; t1 < 5; t1++)
    { 
	    if(!inv)
		{
	        ch2 = font[p + t1];
		}
		else
		{
	        ch2 = ~font[p + t1];
		}
		
        i2c_write_byte(c++, row, ch2);
    }
	
	if(!inv)
	{
	    i2c_write_byte(c, row, 0x00);
	}
	else
	{
	    i2c_write_byte(c, row, 0xFF);
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


// Quick and dirty delay
static void delay (unsigned int time) 
{
    for (unsigned int i = 0; i < time; i++)
    {
        for (volatile unsigned int j = 0; j < 2000; j++);
    }    
}

  ///////////////////
 //   Main code   //
/////////////////// 
int main(void)
{	
	///////////////////////////////////
    // Set up LEDs - GPIOD 12,13,14,15
    ///////////////////////////////////
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~(0xFFU << 24);
    GPIOD->MODER |= (0x55 << 24);
    GPIOD->ODR    = 0x0000;

    //////////////////////////////////////////
    // Set up I2C - GPIOB 6, 9
    //////////////////////////////////////////
    //Enable I2C clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    //Set up I2C PB6 and PB9 pins
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
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
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    I2C1->CR2 |= (10 << 0); //10Mhz peripheral clock
    I2C1->CCR |= (50 << 0); 
    
    //Maximum rise time defintion
    I2C1->TRISE |= (11 << 0); //Program for value=11 => 100khz
        
    //Enable I2C    
    I2C1->CR1 |= I2C_CR1_PE; 
    
    //Start OLED
    oled_init();
    oled_cls();
    
    //Do some testing
    //oled_putstring(0, 0, (char*) "DK7IH 2021", 0);
    //oled_putstring(0, 1, (char*) "DK7IH", 0);
    i2c_write_byte(0, 0, 255);

    while(1)
    {
        GPIOD->ODR |= (1 << 12); //Green led off, blink shows success
        delay(10);
     
        GPIOD->ODR &= ~(1 << 12); //Green led on
        delay(10);
    }
    return 0; 
}
