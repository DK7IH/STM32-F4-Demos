///////////////////////////////////////////////////////////////////
//                    Driver for LCD 20x4 (green)                //
//                    Module No.:CFAH2004D-YYH-ET                //
//                    Crystalfontz America, Inc.                 //
///////////////////////////////////////////////////////////////////
/*  Mikrocontroller:  ARM Cortex M4 (STM32F407)                  */
/*                                                               */
/*  Compiler:         GCC (GNU AVR C-Compiler)                   */
/*  Autor:            Peter Rachow                               */
/*  Letzte Aenderung: 2021-07-05                                 */
/*****************************************************************/
#include "stm32f4xx.h"
#include <string.h>
#include <stdlib.h>

  ///////////////////
 //  LCD-Display  //
///////////////////
#define LCDGPIO GPIOA
#define LCD_D0 0 //PA0
#define LCD_D1 1 //PA1
#define LCD_D2 2 //PA2
#define LCD_D3 3 //PA3
#define LCD_RS 4 //PA4
#define LCD_RW 5 //PA5
#define LCD_E  6 //PA6

//LEDs
#define LEDGPIO GPIOA

void lcd_write(char, unsigned char);
void lcd_write(char, unsigned char);
void lcd_init(void);
void lcd_cls(void);
void lcd_line_cls(int);
void lcd_putchar(int, int, unsigned char);
void lcd_putstring(int, int, char*);
int lcd_putnumber(int, int, long, int, int, char, char);
void lcd_display_test(void);
int lcd_check_busy(void);

static void delay_ms(unsigned int); 
static void delay_us(unsigned int); 

int main(void);

// Write CMD or DATA to LCD
void lcd_write(char lcdmode, unsigned char value)
{
    int t1;
    
    while(lcd_check_busy()); //Check busy flag
    	
	LCDGPIO->ODR &= ~(1 << LCD_RW);   //Set RW to write operation, i. e. =0
	    
    if(!lcdmode)
	{
        LCDGPIO->ODR &= ~(1 << LCD_RS); //CMD
	}	
    else
	{
        LCDGPIO->ODR |= (1 << LCD_RS);   //DATA
	}	
	    
    LCDGPIO->ODR |= (1 << LCD_E); //E = 1
    //HI NIBBLE    
    for(t1 = 0; t1 < 4; t1++)
	{
	    if(((value & 0xF0) >> 4) & (1 << t1))
	    {
	       LCDGPIO->ODR |= (1 << t1);      
	    }
        else	
	    {
           LCDGPIO->ODR &= ~(1 << t1);     
	    }  
	}	
	
	LCDGPIO->ODR &= ~(1 << LCD_E);
	
	//LO NIBBLE
	LCDGPIO->ODR |= (1 << LCD_E); //E = 1
	for(t1 = 0; t1 < 4; t1++)
	{
	    if(value  & (1 << t1))
	    {
	       LCDGPIO->ODR |= (1 << t1);      
	    }
        else	
	    {
           LCDGPIO->ODR &= ~(1 << t1);     
	    }  
	}
    
	LCDGPIO->ODR &= ~(1 << LCD_E);
}

int lcd_check_busy(void)
{
    int t1;
	unsigned char value;
	
	//LCD_PORT data bits (0:3) on rx mode
	for(t1 = 0; t1 < 4; t1++)
	{
	    GPIOE->MODER  &= ~(3 << (t1 << 1)); //Set to 00 -> Input mode
        GPIOE->PUPDR  &= ~(3 << (t1 << 1)); //Set to 00	-> No pullup nor pulldown
    } 
	
    LCDGPIO->ODR &= ~(0x01);
    LCDGPIO->ODR |= (1 << LCD_RW);  //Read operation => RW=1
	
	LCDGPIO->ODR &= ~(1 << LCD_RS); //CMD => RS=0: for busy flag
	
	//Read data
	//Hi nibble
	LCDGPIO->ODR |= (1 << LCD_E);          //E=1
    delay_us(100);       
	value = (LCDGPIO->IDR & 0x0F) << 4;
    LCDGPIO->ODR &= ~(1 << LCD_E);       //E=0	
		
	//Lo nibble
	LCDGPIO->ODR |= (1 << LCD_E);          //E=1
    delay_us(100);       
	value += (LCDGPIO->IDR & 0x0F);
    LCDGPIO->ODR &= ~(1 << LCD_E);       //E=0	
		
	//Put pin D0..D3 in general purpose output mode
    LCDGPIO->MODER  |=  (1 << (LCD_D0 << 1));	
    LCDGPIO->MODER  |=  (1 << (LCD_D1 << 1));	
    LCDGPIO->MODER  |=  (1 << (LCD_D2 << 1));	
    LCDGPIO->MODER  |=  (1 << (LCD_D3 << 1));	
    	
	LCDGPIO->ODR |= 0x01;   
	
	return (value >> 8) & 1;
}  


//Send one char to LCD
void lcd_putchar(int row, int col, unsigned char ch)
{
   int offs[] = {0x00, 0x40, 0x14, 0x54}; //Individual memory offset for each LCD line
	
	lcd_write(0, 0x80 + offs[row] + col);
    lcd_write(1, ch);
}


//Print out \0-terminated string on LCD
void lcd_putstring(int row, int col, char *s)
{
    unsigned char t1 = col;

    while(*(s))
	{
        lcd_putchar(row, t1++, *(s++));
	}	
}

//Clear LCD
void lcd_cls(void)
{
    lcd_write(0, 1);
}


//Init LCD
void lcd_init(void)
{
	
	// Basic settings of LCD
    // 4-Bit mode, 5 pixels width matrix
    lcd_write(0, 0x28);
        
    //4-line mode
    lcd_write(0, 0x2C); //RE=1
    lcd_write(0, 0x09);
    lcd_write(0, 0x28); //RE=0
            
    // Display on, Cursor off, Blink off 
    lcd_write(0, 0x0C);
    
    // No display shift, no cursor move
    lcd_write(0, 0x04);
    
    return;	   
    // Basic settings of LCD
    // 4-Bit mode, 2 lines, 5x7 matrix
    lcd_write(0, 0x28);
    delay_ms(2);
    lcd_write(0, 0x28);
    delay_ms(2);
    
    // Display on, Cursor off, Blink off 
    lcd_write(0, 0x0C);
    delay_ms(2);

    // No display shift, no cursor move
    lcd_write(0, 0x04);
    delay_ms(2);
}

//Write an n-digit number (int or long) to LCD
int lcd_putnumber(int row, int col, long num, int digits, int dec, char orientation, char showplussign)
{
    char cl = col, minusflag = 0;
    unsigned char cdigit[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, digitcnt = 0;
    long t1, t2, n = num, r, x = 1;

    if(num < 0)
    {
        minusflag = 1;
        n *= -1;
    }

    /* Stellenzahl automatisch bestimmen */
    if(digits == -1)
    {
        for(t1 = 1; t1 < 10 && (n / x); t1++)
		{
            x *= 10;
		}	
        digits = t1 - 1;
    }

    if(!digits)
        digits = 1;

    for(t1 = digits - 1; t1 >= 0; t1--)
    {
        x = 1;
        for(t2 = 0; t2 < t1; t2++)
            x *= 10;
        r = n / x;
        cdigit[digitcnt++] = r + 48;

        if(t1 == dec) 
            cdigit[digitcnt++] = 46;
        n -= r * x;
    }

    digitcnt--;
    t1 = 0;

    /* Ausgabe */
    switch(orientation)
    {
        case 'l':   cl = col;
                    if(minusflag)
                    {
                        lcd_putchar(row, cl++, '-');
                        digitcnt++;
                    }	 
		            else
		            {
		                if(showplussign)
			            {
			                lcd_putchar(row, cl++, '+');
                            digitcnt++;
			            } 
                    }	
			
                    while(cl <= col + digitcnt)                       /* Linksbuendig */
		            {
                        lcd_putchar(row, cl++, cdigit[t1++]);
					}	
                    break;

        case 'r':   t1 = digitcnt;                              /* Rechtsbuendig */
                    for(cl = col; t1 >= 0; cl--)              
					{
                        lcd_putchar(row, cl, cdigit[t1--]);
                        if(minusflag)	
						{
                            lcd_putchar(row, --cl, '-');
                        }
					}	
    }
	
    if(dec == -1)
	{
        return digits;
	}	
    else
	{
        return digits + 1;	
	}	
}	

void lcd_line_cls(int ln)
{
    int t1; 
	
	for(t1 = 0; t1 < 15; t1++)
	{
	    lcd_putchar(ln, t1, 32);
	}
}	

//Define chars
void defcustomcharacters(void)
{
    int i1;
    unsigned char adr = 0x40;

    unsigned char customchar[]={0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, //S-Meter chars
	                            0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
	                            0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C,  
	                            0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 
	                            0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, //End S-Mater chars
	                            0x00, 0x00, 0x0E, 0x0E, 0x0E, 0x0E, 0x00, 0x00,
	                            0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x00, 0x00,
	                            0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E};
    lcd_write(0, 0);
    lcd_write(1, 0);

    //Send data to CGRAM in LCD
    for (i1 = 0; i1 < 64; i1++)
    {
        lcd_write(0, adr++);
        lcd_write(1, customchar[i1]);
    }
}

  /////////////////////////////
 //  Cheap and dirty delay  //
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

int main()
{
	delay_ms(100);

	//Turn on the GPIOA peripheral
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //RCC->AHB1ENR |= (1<<0);
    
    //LCD
    //Put pin D0..D6in general purpose output mode
    LCDGPIO->MODER  |=  (1 << (LCD_D0 << 1));	
    LCDGPIO->MODER  |=  (1 << (LCD_D1 << 1));	
    LCDGPIO->MODER  |=  (1 << (LCD_D2 << 1));	
    LCDGPIO->MODER  |=  (1 << (LCD_D3 << 1));	
    LCDGPIO->MODER  |=  (1 << (LCD_RS << 1));	
    LCDGPIO->MODER  |=  (1 << (LCD_RW << 1));	
    LCDGPIO->MODER  |=  (1 << (LCD_E << 1));	
    
	//Display init
	delay_ms(20); //Datasheet: "Wait for more than 15ms after VDD rises to 4.5V"
    lcd_init();
    lcd_cls();		
    
    defcustomcharacters();		
    	
    lcd_putstring(0, 0, (char*)"LCD DEMO");
    lcd_putstring(1, 0, (char*)"by DK7IH");
    lcd_putstring(2, 0, (char*)"(C) 2021");
    lcd_putstring(3, 0, (char*)"vy 73!");
       		    	
    for(;;) 
	{
		
	}
	return 0;
}

