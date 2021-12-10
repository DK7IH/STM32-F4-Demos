///////////////////////////////////////////////////////////////////  
/*                    OLED SH1106 driver software (Text only)    */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         F411E Blackpillboard by WeAct              */
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

#define S_LCDWIDTH               128
#define S_LCDHEIGHT              64 

//////////////////////////////////////
//   L   C   D   
//////////////////////////////////////
// Font 6x8 for OLED
const char font[97][6] = {
{0x00,0x00,0x00,0x00,0x00,0x00},	// 0x20
{0x00,0x00,0x06,0x5F,0x06,0x00},	// 0x21
{0x00,0x07,0x03,0x00,0x07,0x03},	// 0x22
{0x00,0x24,0x7E,0x24,0x7E,0x24},	// 0x23
{0x00,0x24,0x2B,0x6A,0x12,0x00},	// 0x24
{0x00,0x63,0x13,0x08,0x64,0x63},	// 0x25
{0x00,0x36,0x49,0x56,0x20,0x50},	// 0x26
{0x00,0x00,0x07,0x03,0x00,0x00},	// 0x27
{0x00,0x00,0x3E,0x41,0x00,0x00},	// 0x28
{0x00,0x00,0x41,0x3E,0x00,0x00},	// 0x29
{0x00,0x08,0x3E,0x1C,0x3E,0x08},	// 0x2A
{0x00,0x08,0x08,0x3E,0x08,0x08},	// 0x2B
{0x00,0x00,0xE0,0x60,0x00,0x00},	// 0x2C
{0x00,0x08,0x08,0x08,0x08,0x08},	// 0x2D
{0x00,0x00,0x60,0x60,0x00,0x00},	// 0x2E
{0x00,0x20,0x10,0x08,0x04,0x02},	// 0x2F
{0x00,0x3E,0x51,0x49,0x45,0x3E},	// 0x30
{0x00,0x00,0x42,0x7F,0x40,0x00},	// 0x31
{0x00,0x62,0x51,0x49,0x49,0x46},	// 0x32
{0x00,0x22,0x49,0x49,0x49,0x36},	// 0x33
{0x00,0x18,0x14,0x12,0x7F,0x10},	// 0x34
{0x00,0x2F,0x49,0x49,0x49,0x31},	// 0x35
{0x00,0x3C,0x4A,0x49,0x49,0x30},	// 0x36
{0x00,0x01,0x71,0x09,0x05,0x03},	// 0x37
{0x00,0x36,0x49,0x49,0x49,0x36},	// 0x38
{0x00,0x06,0x49,0x49,0x29,0x1E},	// 0x39
{0x00,0x00,0x6C,0x6C,0x00,0x00},	// 0x3A
{0x00,0x00,0xEC,0x6C,0x00,0x00},	// 0x3B
{0x00,0x08,0x14,0x22,0x41,0x00},	// 0x3C
{0x00,0x24,0x24,0x24,0x24,0x24},	// 0x3D
{0x00,0x00,0x41,0x22,0x14,0x08},	// 0x3E
{0x00,0x02,0x01,0x59,0x09,0x06},	// 0x3F
{0x00,0x3E,0x41,0x5D,0x55,0x1E},	// 0x40
{0x00,0x7E,0x11,0x11,0x11,0x7E},	// 0x41
{0x00,0x7F,0x49,0x49,0x49,0x36},	// 0x42
{0x00,0x3E,0x41,0x41,0x41,0x22},	// 0x43
{0x00,0x7F,0x41,0x41,0x41,0x3E},	// 0x44
{0x00,0x7F,0x49,0x49,0x49,0x41},	// 0x45
{0x00,0x7F,0x09,0x09,0x09,0x01},	// 0x46
{0x00,0x3E,0x41,0x49,0x49,0x7A},	// 0x47
{0x00,0x7F,0x08,0x08,0x08,0x7F},	// 0x48
{0x00,0x00,0x41,0x7F,0x41,0x00},	// 0x49
{0x00,0x30,0x40,0x40,0x40,0x3F},	// 0x4A
{0x00,0x7F,0x08,0x14,0x22,0x41},	// 0x4B
{0x00,0x7F,0x40,0x40,0x40,0x40},	// 0x4C
{0x00,0x7F,0x02,0x04,0x02,0x7F},	// 0x4D
{0x00,0x7F,0x02,0x04,0x08,0x7F},	// 0x4E
{0x00,0x3E,0x41,0x41,0x41,0x3E},	// 0x4F
{0x00,0x7F,0x09,0x09,0x09,0x06},	// 0x50
{0x00,0x3E,0x41,0x51,0x21,0x5E},	// 0x51
{0x00,0x7F,0x09,0x09,0x19,0x66},	// 0x52
{0x00,0x26,0x49,0x49,0x49,0x32},	// 0x53
{0x00,0x01,0x01,0x7F,0x01,0x01},	// 0x54
{0x00,0x3F,0x40,0x40,0x40,0x3F},	// 0x55
{0x00,0x1F,0x20,0x40,0x20,0x1F},	// 0x56
{0x00,0x3F,0x40,0x3C,0x40,0x3F},	// 0x57
{0x00,0x63,0x14,0x08,0x14,0x63},	// 0x58
{0x00,0x07,0x08,0x70,0x08,0x07},	// 0x59
{0x00,0x71,0x49,0x45,0x43,0x00},	// 0x5A
{0x00,0x00,0x7F,0x41,0x41,0x00},	// 0x5B
{0x00,0x02,0x04,0x08,0x10,0x20},	// 0x5C
{0x00,0x00,0x41,0x41,0x7F,0x00},	// 0x5D
{0x00,0x04,0x02,0x01,0x02,0x04},	// 0x5E
{0x80,0x80,0x80,0x80,0x80,0x80},	// 0x5F
{0x00,0x00,0x03,0x07,0x00,0x00},	// 0x60
{0x00,0x20,0x54,0x54,0x54,0x78},	// 0x61
{0x00,0x7F,0x44,0x44,0x44,0x38},	// 0x62
{0x00,0x38,0x44,0x44,0x44,0x28},	// 0x63
{0x00,0x38,0x44,0x44,0x44,0x7F},	// 0x64
{0x00,0x38,0x54,0x54,0x54,0x08},	// 0x65
{0x00,0x08,0x7E,0x09,0x09,0x00},	// 0x66
{0x00,0x18,0xA4,0xA4,0xA4,0x7C},	// 0x67
{0x00,0x7F,0x04,0x04,0x78,0x00},	// 0x68
{0x00,0x00,0x00,0x7D,0x40,0x00},	// 0x69
{0x00,0x40,0x80,0x84,0x7D,0x00},	// 0x6A
{0x00,0x7F,0x10,0x28,0x44,0x00},	// 0x6B
{0x00,0x00,0x00,0x7F,0x40,0x00},	// 0x6C
{0x00,0x7C,0x04,0x18,0x04,0x78},	// 0x6D
{0x00,0x7C,0x04,0x04,0x78,0x00},	// 0x6E
{0x00,0x38,0x44,0x44,0x44,0x38},	// 0x6F
{0x00,0xFC,0x44,0x44,0x44,0x38},	// 0x70
{0x00,0x38,0x44,0x44,0x44,0xFC},	// 0x71
{0x00,0x44,0x78,0x44,0x04,0x08},	// 0x72
{0x00,0x08,0x54,0x54,0x54,0x20},	// 0x73
{0x00,0x04,0x3E,0x44,0x24,0x00},	// 0x74
{0x00,0x3C,0x40,0x20,0x7C,0x00},	// 0x75
{0x00,0x1C,0x20,0x40,0x20,0x1C},	// 0x76
{0x00,0x3C,0x60,0x30,0x60,0x3C},	// 0x77
{0x00,0x6C,0x10,0x10,0x6C,0x00},	// 0x78
{0x00,0x9C,0xA0,0x60,0x3C,0x00},	// 0x79
{0x00,0x64,0x54,0x54,0x4C,0x00},	// 0x7A
{0x00,0x08,0x3E,0x41,0x41,0x00},	// 0x7B
{0x00,0x00,0x00,0x77,0x00,0x00},	// 0x7C
{0x00,0x00,0x41,0x41,0x3E,0x08},	// 0x7D
{0x00,0x02,0x01,0x02,0x01,0x00},	// 0x7E
{0x00,0x3C,0x26,0x23,0x26,0x3C},	// 0x7F
{0x00,0x1E,0xA1,0xE1,0x21,0x12}};	// 0x80

///////////////////////////
//Function declarations
///////////////////////////
//OLED
void oled_command(uint8_t value);
void oled_data(unsigned int*, unsigned int);
void oled_gotoxy(uint8_t, uint8_t);
void oled_cls(int);
void oled_init(void);
void oled_byte(uint8_t);
void oled_putchar1(unsigned int x, unsigned int y, uint8_t ch, int);
void oled_putchar2(unsigned int x, unsigned int y, uint8_t ch, int);
void oled_putnumber(int, int, long, int, int, int);
void oled_putstring(int, int, char*, char, int);
void oled_write_section(int, int, int, int);

//String
int int2asc(long num, int dec, char *buf, int buflen);

//I2C
void i2c_start(void);
void i2c_stop(void); 
void i2c_write(uint8_t, uint8_t);
void i2c_write_bytes(uint8_t *data, uint8_t n);
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

//Variable number of bytes (>2)
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
    oled_command(0xAE);    // Display OFF
	oled_command(0x20);    // Set Memory Addressing Mode
    oled_command(0x00);    // HOR
    
    oled_command(0xB0);    //Set Page Start Address for Page Addressing Mode, 0-7
    oled_command(0xC8);    //Set COM Output Scan Direction
    oled_command(0x00);    //--set low column address
    oled_command(0x10);    //--set high column address
    oled_command(0x40);    //--set start line address
    oled_command(0x81);
    oled_command(0xFF);    //Set contrast control register
    oled_command(0xA1);    //Set Segment Re-map. A0=address mapped; A1=address 127 mapped.
    oled_command(0xA6);    //Set display mode. A6=Normal; A7=Inverse
    oled_command(0xA8);
    oled_command(0x3F);    //Set multiplex ratio(1 to 64)
    oled_command(0xA4);    //Output RAM to Display
					       //0xA4=Output follows RAM content; 0xA5,Output ignores RAM content
					       
    oled_command(0xD3);
    oled_command(0x00);    //Set display offset. 00 = no offset
    oled_command(0xD5);    //--set display clock divide ratio/oscillator frequency
    oled_command(0xF0);    //--set divide ratio
    oled_command(0xD9); 
    oled_command(0x22);    //Set pre-charge period
    oled_command(0xDA);
    oled_command(0x12);    //Set com pins hardware configuration
    oled_command(0xDB);    //--set vcomh
    oled_command(0x20);    //0x20,0.77xVcc
    oled_command(0x8D);
    oled_command(0x14);    //Set DC-DC enabl
    oled_command(0xAF);    //Display ON
   
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
	int t0;
	int n = FONTW;
    uint8_t *d = (uint8_t*) malloc(n);
    int cnt = 0;
	
	d[cnt++] = OLEDDATA;	
	oled_gotoxy(x, y);
	for(t0 = 0; t0 < FONTW; t0++)
	{
		if(!invert)
		{
            d[cnt++] = font[ch - 32][t0];
        }
        else    
        {
			d[cnt++] = ~font[ch - 32][t0];
        }
	}
	i2c_write_bytes(d, cnt);
	free(d);
}		

//Write character to screen (DOUBLE size);
void oled_putchar2(unsigned int x, unsigned int y, uint8_t ch, int invert)
{
	int t0, t1;
	uint8_t ch2;
	uint16_t i[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
    uint8_t *d;
    d = (uint8_t*) malloc(FONTW);
    
    int cnt;
		
	//Prepare bit pattern for chars in double size	
	for(t0 = 0; t0 < FONTW; t0++)
	{
		for(t1 = 0; t1 < 8; t1++)
		{
			if(!invert)
			{
				ch2 = font[ch - 32][t0];
			}
			else
			{
				ch2 = ~font[ch - 32][t0];
			}
				
		    if(ch2 & (1 << t1))
		    {
			    i[t0] += (1 << (t1 << 1));
			    i[t0] += (1 << ((t1 << 1) + 1));
		    }	
	    }
	}
	
    //Send bit patterns to OLED in 2 slices
    //Line 1
    cnt = 0;
	d[cnt++] = OLEDDATA;
	for(t0 = 0; t0 < FONTW; t0++)
	{		
	    d[cnt++] = i[t0] & 0xFF;
	    d[cnt++] = i[t0] & 0xFF;
	}
	oled_gotoxy(x, y);
	i2c_write_bytes(d, cnt);
	
	//Line 2
	cnt = 0;
	d[cnt++] = OLEDDATA;
	for(t0 = 0; t0 < FONTW; t0++)
	{		
	    d[cnt++] = (i[t0] & 0xFF00) >> 8;
	    d[cnt++] = (i[t0] & 0xFF00) >> 8;
	}
	oled_gotoxy(x, y + 1);
	i2c_write_bytes(d, cnt);
	free(d);
}		

//Print string in given size
//lsize=0 => normal height, lsize=1 => double height
void oled_putstring(int col, int row, char *s, char lsize, int inv)
{
    int c = col;
	
	while(*s)
	{
	    if(!lsize)
		{
	        oled_putchar1(c, row, *s++, inv);
		}
        else
        {
            oled_putchar2(c, row, *s++, inv);
		}	
		c += (lsize + 1) * FONTW;
	}
}

//Print an integer/long to OLED
void oled_putnumber(int col, int row, long num, int dec, int lsize, int inv)
{
    char *s = (char*)malloc(16);
	if(s != NULL)
	{
	    int2asc(num, dec, s, 16);
	    oled_putstring(col, row, s, lsize, inv);
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
    // Set up LED - GPIOC 13
    ///////////////////////////////////
    //Turn on the GPIOC peripheral for LED
    RCC->AHB1ENR |= (1 << 2); 
	GPIOC->MODER |= (1 << (13 << 1));	//Set PC13 for output
	
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
    oled_init();
    oled_cls(0);
    
    //Intro screen
    oled_putstring(0, 0, (char*) "DK7IH OLED Driver STM32", 0, 0);
    
    while(1)
    {
        GPIOC->ODR |= (1 << 13); //Led off
        delay(10);
     
        GPIOC->ODR &= ~(1 << 13); //Led on
        delay(10);
    }
    return 0; 
}
