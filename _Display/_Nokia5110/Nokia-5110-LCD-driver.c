///////////////////////////////////////////////////////////////////  
/*                    Nokia5110 LCD driver software (Text only)  */
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

// PIN definitions of Nokia 5110 lines on PORT A
#define LCDGPIO GPIOA
#define RST 0 //PA0 yellow
#define DC  1 //PA1 grey
#define DIN 2 //PA2 green
#define CLK 3 //PA3 blue

#define LEDGPIO GPIOA

// STRING FUNCTIONS
int int2asc(long, int, char*, int);
//LCD functions
void lcd_sendbyte(char, int);
void lcd_senddata(char);
void lcd_sendcmd(char);
void lcd_reset(void);
void lcd_gotoxy(char, char);
void lcd_cleanram(void);
int xp2(int);
void lcd_putchar2(int, int, char, int);
void lcd_putchar1(int, int, char, int);
void lcd_putstring(int, int, char*, char, int);
void lcd_putnumber(int, int, long, int, int, int);
void lcd_init(void);
void lcd_clearsection(int, int, int);
void lcd_cls(int, int, int, int);
//  DATA DISPLAY ROUTINES
void show_frequency(unsigned long);

//MISC
static void delay (unsigned int);

//DATA
// Font 5x8 for LCD Display Nokia 5110
const char xchar[485] = {
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
 //  Cheap and dirty delay  //
/////////////////////////////
static void delay (unsigned int time) 
{
    for (unsigned int i = 0; i < time; i++)
    {
        for (volatile unsigned int j = 0; j < 2000; j++);
    }    
}

  /////////////////////////////
 //    String functions     //
/////////////////////////////
//*******************************
// STRING FUNCTIONS
//*******************************
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

  /////////////////////////////
 //  SPI for LCD Nokia 5110 //
/////////////////////////////
// PIN definitions of Nokia 5110 lines on PORT A
// RST 0 //PA0 yellow
// DC  1 //PA1 grey
// DIN 2 //PA2 green
// CLK 3 //PA3 blue
//Send the information to LCD (command or data)
void lcd_sendbyte(char x, int command)
{ 
	int t1, bx = 128;
	
	if(command)
	{   
		LCDGPIO->ODR &= ~(1 << DC); //CMD
	}
	else 
    {  
		LCDGPIO->ODR |= (1 << DC); //DATA
	} 
		
    for(t1 = 0; t1 < 8; t1++)
    { 
		LCDGPIO->ODR &= ~(1 << CLK);
	    //GPIOA->BSRR = GPIO_BSRR_BR3;
			    
        if(x & bx)
	    {
            LCDGPIO->ODR |= (1 << DIN);
		}
	    else
        {
	        LCDGPIO->ODR &= ~(1 << DIN);
	    }	
        
        LCDGPIO->ODR |= (1 << CLK);
        //GPIOA->BSRR = GPIO_BSRR_BS3;
		bx >>= 1;
    }
}

//Reset LCD when program starts
void lcd_reset(void)
{
	LCDGPIO->ODR |= (1 << RST);
	delay(100);
    LCDGPIO->ODR &= ~(1 << RST);
	delay(100);
    LCDGPIO->ODR |= (1 << RST);
}

//Send a display data  to Nokia LCD 5110
void lcd_senddata(char x)
{
    lcd_sendbyte(x, 0);
}

//Send a command to Nokia LCD 5110
void lcd_sendcmd(char x)
{
    lcd_sendbyte(x, 1);
}

//Set pos of cursor to start next text display on LCD
void lcd_gotoxy(char x, char y)
{ 
    lcd_sendcmd(0x40|(y & 0x07));
    lcd_sendcmd(0x80|(x & 0x7f));
}

//Init RAM of LCD
void lcd_cleanram(void)
{
    int i;
    
	lcd_gotoxy(0,0);
	
    delay(10);
    
	for(i = 0; i < 768; i++)
    {
        lcd_senddata(0x00);
	}	
    
	delay(1);
}

//2^x
int xp2(int xp)
{
    int t1, r = 1;
    for(t1 = 0; t1 < xp; t1++)
    {
	    r <<= 1;
    }
    return r;

}

//Print character in double size
void lcd_putchar2(int col, int row, char ch1, int inv)
{ 
    int p, t1, t2, x;
	int b, b1, b2; 
    char colval;
	   
    p = (5 * ch1) - 160;
    	
	for(t2 = 0; t2 < 6; t2++)
    { 
	    //Get vertical byte data of char
		if(!inv)
		{
	        colval = xchar[p];
			if(t2 == 5)
			{
			    colval = 0;
			}
		}
		else
		{
	        colval = ~xchar[p];
			if(t2 == 5)
			{
			    colval = 255;
			}
		}
	  		
		b = 0;
		x = 1;
        
		for(t1 = 0; t1 < 7; t1++)
        {
            if(colval & x)
            {
	            b += xp2(t1 * 2);
	            b += xp2(t1 * 2 + 1);
            }
            x <<= 1;
	    }
    
        b1 = b & 0xFF; //Lower byte
        b2 = (b & 0xFF00) >> 8; //Upper byte
		
		//Print data to screen
		lcd_gotoxy(col + t2 * 2, row);
		lcd_senddata(b1);
		lcd_gotoxy(col + t2 * 2, row + 1);
		lcd_senddata(b2);
		lcd_gotoxy(col + t2 * 2 + 1, row);
		lcd_senddata(b1);
		lcd_gotoxy(col + t2 * 2 + 1, row + 1);
		lcd_senddata(b2);
		p++;
	}	
	
    lcd_senddata(0x00);
}

//Print character in normal size
void lcd_putchar1(int col, int row, char ch1, int inv)
{ 
    int p, t1;
    char ch2;
	
	lcd_gotoxy(col, row);
    
    p = (5 * ch1) - 160;
    for(t1 = 5; t1 > 0; t1--)
    { 
	    if(!inv)
		{
	        ch2 = xchar[p];
		}
		else
		{
	        ch2 = ~xchar[p];
		}
		
        lcd_senddata(ch2);
        p++;
    }
	
	if(!inv)
	{
	    lcd_senddata(0x00);
	}
	else
	{
	    lcd_senddata(0xFF);
	}
}

//Print string in given size
void lcd_putstring(int col, int row, char *s, char lsize, int inv)
{
    int c = col;
	
	while(*s)
	{
	    if(!lsize)
		{
	        lcd_putchar1(c, row, *s++, inv); //Normal size
		}
        else
        {
            lcd_putchar2(c, row, *s++, inv); //Double size
		}	
		c += (lsize + 1) * 6;
	}
}

//PUTNUMBER
void lcd_putnumber(int col, int row, long num, int dec, int lsize, int inv)
{
    char *s;
    s = (char*) malloc(16);
	if(s != NULL)
	{
	    int2asc(num, dec, s, 16);
	    lcd_putstring(col, row, s, lsize, inv);
	    free(s);
	}	
}

//Init NOKIA 5110 LCD
void lcd_init(void)
{

    delay(20);
	lcd_reset();
	delay(10);	
	
	//Configure the display
    lcd_sendcmd(0x21);    //Extended commands follow
    lcd_sendcmd(0xA5);    //Set Vop (Contrast)
    lcd_sendcmd(0x04);    //Set Temp. coefficient
    lcd_sendcmd(0x14);    //LCD bias mode
    lcd_sendcmd(0x20);    //Normal instruction set
    lcd_sendcmd(0x0C);    //Set display to normal mode.
	
	lcd_cleanram();
    delay(100);
}	

//Clear part of LCD
void lcd_clearsection(int x0, int x1, int y0)
{

    int t1;
	
	for(t1 = x0; t1 < x1; t1++)
	{
	    lcd_gotoxy(t1, y0);
		lcd_senddata(0x00);
	}	

}

//Clear whole LCD
void lcd_cls(int x0, int x1, int y0, int y1)
{
    int x, y;
	
	for(y = y0; y < y1; y++)
	{
	    for(x = x0; x < x1; x++)
		{
            lcd_gotoxy(x, y);
		    lcd_senddata(0x00);
		}
	}	
}

int main(void)
{
    //Turn on the GPIOA peripheral
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //RCC->AHB1ENR |= (1<<0);
    
    //LCD
    //Put pin A0..A3 in general purpose output mode
    LCDGPIO->MODER  |=  (1 << (RST << 1));	
    LCDGPIO->MODER  |=  (1 << (DC << 1));	
    LCDGPIO->MODER  |=  (1 << (DIN << 1));	
    LCDGPIO->MODER  |=  (1 << (CLK << 1));	
    
	//LEDs
	//Put pin A6 and A7 in general purpose output mode
    LEDGPIO->MODER  |=  (1 << (6 << 1));	
    LEDGPIO->MODER  |=  (1 << (7 << 1));	
	
	//INIT LCD
	delay(200);
	lcd_init();
	
	lcd_putstring(0, 0, (char*) " Nokia 5110", 0, 0);
	lcd_putstring(0, 1, (char*) "    for", 0, 0);
	lcd_putstring(0, 2, (char*) "  STM32F407", 0, 0);
	lcd_putstring(0, 3, (char*) "2021 by Peter B.", 0, 0);
	lcd_putstring(0, 4, (char*) "   (DK7IH)", 0, 0);
	lcd_putstring(0, 5, (char*) "   dk7ih.de", 0, 0);
		
	for(;;)
	{
		// Set the state of pin A6 to output high
        GPIOA->BSRR = GPIO_BSRR_BS6;
        // Set the state of pin A7 to output low
        GPIOA->BSRR = GPIO_BSRR_BR7;
        delay(500);
        		
		 // Set the state of pin A6 to output low
        GPIOA->BSRR = GPIO_BSRR_BR6;
        // Set the state of pin A7 to output high
        GPIOA->BSRR = GPIO_BSRR_BS7;
        delay(500);
	}	
} 
 
