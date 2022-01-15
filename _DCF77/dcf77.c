///////////////////////////////////////////////////////////////////  
/*                    DCF77 time decoder w. Nokia5110            */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         F411E Discovery board by STM               */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JAN 2022                                   */
///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include <string.h>
#include <stdlib.h>

// PIN definitions of Nokia 5110 lines on PORT A
#define LCDGPIO GPIOA
#define RST 3 //yellow
#define DC  2 //grey
#define DIN 1 //green
#define CLK 0 //blue

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

//DCF77 decoding
int get_bits(int[], int, int);
int check_parity(int[], int, int);
void decode_date(int[]);
void decode_time(int[]);

//MISC
static void delay (unsigned int);
extern "C" void TIM2_IRQHandler(void); 

//DATA
// Font 5x8 for LCD Display Nokia 5110
#define FONTW 6
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
0x00,0x36,0x00,0x00,0x00, // 3a :
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

unsigned int ms0 = 0;

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
 //     TIM2 INT Handler    //
/////////////////////////////
extern "C" void TIM2_IRQHandler(void)//IRQ-Handler for TIM2
{
    if (TIM2->SR & TIM_SR_UIF)       //Toggle LED on update event every 500ms
    {
		ms0++;
    }
    TIM2->SR = 0x0;                  //Reset status register
}

  /////////////////////////////
 //    String functions     //
/////////////////////////////
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
	    		    
        if(x & bx)
	    {
            LCDGPIO->ODR |= (1 << DIN);
		}
	    else
        {
	        LCDGPIO->ODR &= ~(1 << DIN);
	    }	
        
        LCDGPIO->ODR |= (1 << CLK);
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
	        lcd_putchar1(c * FONTW, row, *s++, inv); //Normal size
		}
        else
        {
            lcd_putchar2(c * FONTW, row, *s++, inv); //Double size
		}	
		c += (lsize + 1);
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
    lcd_sendcmd(0xA7);    //Set Vop (Contrast)
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

  /////////////////////////////
 //   DCF77 decode routines //
/////////////////////////////
int get_bits(int b[], int sta, int end)
{
    int t0, n = 0;
    
    for(t0 = sta; t0 < end + 1; t0++)
    {
        n += b[t0] << (t0 - sta); //Change bit order! 0100 bin. = 2 dec.!!!
    }
    return n;
    
}

void decode_date(int b[])
{
	char *weekday[7] = {(char*)"MON", (char*)"TUE", (char*)"WEN", (char*)"THU", (char*)"FRI", (char*)"SAT", (char*)"SUN"};
	int wday = 0;
	
	lcd_putstring(0, 2, (char*)"               ", 0, 0);
	lcd_putstring(0, 2, (char*)"--.--.--", 0, 0);
	
	if(check_parity(b, 36, 57) != get_bits(b, 58, 58))
	{
	    return;
	}
	
    //Day
	lcd_putnumber(0, 2, get_bits(b, 40, 41), -1, 0, 0);
	lcd_putnumber(1, 2, get_bits(b, 36, 39), -1, 0, 0);
				
	//Month
	lcd_putnumber(3, 2, get_bits(b, 49, 49), -1, 0, 0);
	lcd_putnumber(4, 2, get_bits(b, 45, 48), -1, 0, 0);
				
	//Year
	lcd_putnumber(6, 2, get_bits(b, 54, 57), -1, 0, 0);
	lcd_putnumber(7, 2, get_bits(b, 50, 53), -1, 0, 0);	
	
	//Day of week	
	wday = get_bits(b, 42, 44) - 1;
	
	if((wday >= 0) && (wday <= 6))
	{	
	    lcd_putstring(10, 2, weekday[wday], 0, 0); 
	}
	else
	{
		lcd_putstring(10, 2, (char*)"---", 0, 0);     
	}	
}	

void decode_time(int b[])
{
	lcd_putstring(0, 4, (char*)"       ", 1, 0);
	
 	//parity check hour
	if(check_parity(b, 29, 34) != get_bits(b, 35, 35))
	{
		lcd_putstring(0, 4, (char*)"--", 1, 0);
	}
	else
	{
	    //Hour
	    lcd_putnumber(0, 4, get_bits(b, 33, 34), -1, 1, 0);
	    lcd_putnumber(2, 4, get_bits(b, 29, 32), -1, 1, 0);
	}
	
	lcd_putstring(4, 4, (char*)":", 1, 0);
		    
	//parity check minute
	if(check_parity(b, 21, 27) != get_bits(b, 28, 28))
	{
		lcd_putstring(5, 4, (char*)"--", 1, 0);
	}
	else
	{
		//Minute
	    lcd_putnumber(5, 4, get_bits(b, 25, 27), -1, 1, 0);
	    lcd_putnumber(7, 4, get_bits(b, 21, 24), -1, 1, 0);
	}    
							
	//MEZ or MESZ?
	if(!b[17] && b[18])
	{
		lcd_putstring(10, 5, (char*)"MEZ ", 0, 0);
	}
			
	if(b[17] && !b[18])
	{
		lcd_putstring(10, 5, (char*)"MESZ", 0, 0);
	}		
}	

int check_parity(int b[], int sta, int end)
{
    int t0, n = 0;
    
    for(t0 = sta; t0 < end + 1; t0++)
    {
        if(b[t0])
        {
			n++;
		}	
    }
    
    if((n / 2) * 2 == n)
    {
		return 0; //even parity
	}
	else	
	{
		return 1; //odd parity
	}
}

int main(void)
{
	int cin; //Input pin signal
	int bitcnt = -1;
	int data[60];
	
	//Turn on the GPIOA peripheral
    RCC->AHB1ENR |= (1 << 0); 
    
    ////////////////////////////////////////////////
    // Set SystemClock to 100 MHz with 25 MHz HSE //
    ////////////////////////////////////////////////
    FLASH->ACR |= (1 << 1);                     //2 wait states for 96+ MHz
    RCC->CR |= (1 << 16);                       //Activate external clock (HSE: 25 MHz)
    while ((RCC->CR & (1 << 17)) == 0);         //Wait until HSE is ready
    
    RCC->PLLCFGR |= (1 << 22);                  //PLL source is HSE
    
    RCC->PLLCFGR &= ~0b11111;                   //Reset bits 4..0, then set PLL-M: VCO input frequency = PLL input clock frequency / PLLM with 2 ≤ PLLM ≤ 63
    RCC->PLLCFGR |= 20;                         // -> f.VCO.in = 25MHz / 20 = 1.25MHz
                                                
    RCC->PLLCFGR &= ~(32704 << 6);
    RCC->PLLCFGR |= 200 << 6;                   //PLL-N: f.VCO.out = f.VCO.in * 200 = 250MHz
    
    RCC->PLLCFGR &= ~(0b11 << 16);              //PLL-P: Main PLL (PLL) division factor for main system clock
                                                //f.PLL.output.clock = f.VCO.out / 2 = 100MHz
                                                
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
    
    /////////
    // LCD //
    /////////
    //Put pin A0..A3 in general purpose output mode
    LCDGPIO->MODER  |=  (1 << (RST << 1));	
    LCDGPIO->MODER  |=  (1 << (DC << 1));	
    LCDGPIO->MODER  |=  (1 << (DIN << 1));	
    LCDGPIO->MODER  |=  (1 << (CLK << 1));	
    
    /////////
    // LED //
    /////////
	//Put pin C13 to general purpose output mode
	RCC->AHB1ENR |= (1 << 2);
    GPIOC->MODER  |=  (1 << (13 << 1));	
    
    //////////////
    // B0 Input //
    //////////////
    RCC->AHB1ENR |= (1 << 1);
    GPIOB->MODER &= ~(1 << (1 << 0));
    
    ////////////////////////////////
    // TIMER2 Millisecond counter //
    ////////////////////////////////
    RCC->APB1ENR |= (1 << 0); //Enable TIM2 clock (bit0)
    
    //Timer calculation
    //Timer update frequency = TIM_CLK/(TIM_PSC+1)/(TIM_ARR + 1) 
    TIM2->PSC = 100000; //Divide system clock (f=100MHz) by 100000 -> update frequency = 1000/s
    TIM2->ARR = 1;    //Define overrun after 1ms

    //Update Interrupt Enable
    TIM2->DIER |= (1 << 0);

    NVIC_SetPriority(TIM2_IRQn, 2); //Priority level 2
    NVIC_EnableIRQ(TIM2_IRQn);      //Enable TIM2 IRQ from NVIC
	
	TIM2->CR1 |= (1 << 0);           //Enable Timer 2 module (CEN, bit0)
	 
	//INIT LCD
	delay(200);
	lcd_init();
	
	lcd_putstring(0, 0, (char*) "DCF77-Demo for", 0, 0);
	lcd_putstring(0, 1, (char*) "STM32F4", 0, 0);
			
	for(;;)
	{
		GPIOC->ODR &= ~(1 << 13);     //Blue LED on			    
	    //Read input pin for LO state
	    cin = GPIOB->IDR & (1 << 0);
	    ms0 = 0;
	    while(!cin)
	    {
			cin = GPIOB->IDR & (1 << 0);
		}
		GPIOC->ODR |= (1 << 13);     //Blue LED off
				
		if(ms0 > 1500)
		{
			//Print date & time
			decode_date(data);
			decode_time(data);
			
			bitcnt = 0; //Start of new minute
		}	
		
		
		//Read input pin for HI state
	    cin = GPIOB->IDR & (1 << 0);
	    ms0 = 0;
	    while(cin)
	    {
			cin = GPIOB->IDR & (1 << 0);
		}
				
		if(ms0 > 100)
		{
			data[bitcnt] = 1;
		}
		else
		{
			data[bitcnt] = 0;
		}
		
		lcd_putnumber(12, 1, (long)bitcnt, -1, 0, 0);
		
		if(bitcnt < 60)
		{
			bitcnt++;
		}	
		
	}	
} 
 
