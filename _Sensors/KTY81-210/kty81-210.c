///////////////////////////////////////////////////////////////////  
//                    ADC demo for STM32F4                       //
///////////////////////////////////////////////////////////////////
//Purpose: This demonstration uses analog sensor KTY81-210 to    //
//maesure ambient temp. A voltage divider with 2k fixed R to 3.3V//
//and KTY81-210 to GND is used. PB0 is analog input to the center//
//of the voltage divider.                                        //
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         WeAct Blackpill board                      */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JAN 2022                                   */
///////////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

  ///////////////////
 //  LCD-Display  //
///////////////////
#define LCD_GPIO GPIOA
#define LCD_D0 0 
#define LCD_D1 1 
#define LCD_D2 2 
#define LCD_D3 3 
#define LCD_E  4 
#define LCD_RW 5
#define LCD_RS 6 

void lcd_write(char, unsigned char);
void lcd_init(void);
void lcd_cls(void);
void lcd_line_cls(int);
void lcd_putchar(int, int, unsigned char);
void lcd_putstring(int, int, char*);
int lcd_putnumber(int, int, long, int, int, char, char);
int lcd_check_busy(void);

  ////////////
 //  MISC  //
////////////
int main(void);

static void delay_ms(unsigned int); 
static void delay_us(unsigned int); 
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

  /////////////////////////////
 //         L C D           //
/////////////////////////////

// Write CMD or DATA to LCD
void lcd_write(char lcdmode, unsigned char value)
{
    uint8_t nh = (value >> 4) & 0x0F;
    uint8_t nl = value & 0x0F;
    
    while(lcd_check_busy()); //Check busy flag
    	
	LCD_GPIO->ODR &= ~(1 << LCD_RW);   //Set RW to write operation, i. e. =0
	    
    if(!lcdmode)
	{
        LCD_GPIO->ODR &= ~(1 << LCD_RS); //CMD
	}	
    else
	{
        LCD_GPIO->ODR |= (1 << LCD_RS);   //DATA
	}	
	
	LCD_GPIO->ODR &= ~0x0F; //Reset data port
		
	//HI NIBBLE        
    LCD_GPIO->ODR |= nh;
    LCD_GPIO->ODR |= (1 << LCD_E);
    delay_ms(1);   
    LCD_GPIO->ODR &= ~(1 << LCD_E);
	
	LCD_GPIO->ODR &= ~0x0F; //Reset data port
	
	//LO NIBBLE        

    LCD_GPIO->ODR |= nl;
    LCD_GPIO->ODR |= (1 << LCD_E);
    delay_ms(1);   
    LCD_GPIO->ODR &= ~(1 << LCD_E);
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
	
    LCD_GPIO->ODR &= ~(0x01);
    LCD_GPIO->ODR |= (1 << LCD_RW);  //Read operation => RW=1
	
	LCD_GPIO->ODR &= ~(1 << LCD_RS); //CMD => RS=0: for busy flag
	
	//Read data
	//Hi nibble
	LCD_GPIO->ODR |= (1 << LCD_E);          //E=1
    delay_us(100);       
	value = (LCD_GPIO->IDR & 0x0F) << 4;
    LCD_GPIO->ODR &= ~(1 << LCD_E);       //E=0	
		
	//Lo nibble
	LCD_GPIO->ODR |= (1 << LCD_E);          //E=1
    delay_us(100);       
	value += (LCD_GPIO->IDR & 0x0F);
    LCD_GPIO->ODR &= ~(1 << LCD_E);       //E=0	
		
	//Put pin D0..D3 in general purpose output mode
    LCD_GPIO->MODER  |=  (1 << (LCD_D0 << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_D1 << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_D2 << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_D3 << 1));	
    	
	LCD_GPIO->ODR |= 0x01;   
	
	return (value >> 8) & 1;
}  


//Send one char to LCD
void lcd_putchar(int row, int col, unsigned char ch)
{
    lcd_write(0, col + 128 + row * 0x40);
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
//Delays must set in accordance to respective LCD model (driver)!
void lcd_init(void)
{
    //Basic settings of LCD
    //4-Bit mode, 2 lines, 5x7 matrix
    lcd_write(0, 0x28);
    delay_ms(8);
    lcd_write(0, 0x28);
    delay_ms(18);
    
    //Display on, Cursor off, Blink off 
    lcd_write(0, 0x0C);
    delay_ms(18);

    //No display shift, no cursor move
    lcd_write(0, 0x04);
    delay_ms(8);
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


  ///////////////////
 // Main programm //
///////////////////
int main(void)
{
    int adcret = 0;//ADC return value (0..1023)
	double volt;   //Voltage derived from ADC value
	double rtherm; //Resistance of KTY81-210
	int r1 = 2000; //Fixed resitor in voltage divider to VDD in Ohms
	double vdd = 3.3;   //Ref. voltage for MCU ADC
	double temp;   //Resulting temperature
	
	//////////////////////////////////////////
    // Setup LED w. GPIOC
    //////////////////////////////////////////
    RCC->AHB1ENR |= (1 << 2);
    GPIOC->MODER |= (1 << (13 << 1));
    
	//////////////////////////////////////////
    // Setup LCD
    //////////////////////////////////////////
	//Turn on the GPIOA peripheral
    RCC->AHB1ENR |= (1 << 0);
    
    //LCD
    //Put pin A0..A6 into general purpose output mode
    LCD_GPIO->MODER  |=  (1 << (LCD_D0 << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_D1 << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_D2 << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_D3 << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_RS << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_RW << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_E << 1));	
    
	//Display init procedure
	delay_ms(20); //Wait for more than 15ms after VDD rises to 4.5V
    lcd_init();
    lcd_cls();		
    
    //////////////////////////////////////////////
    //ADC1 Init
    //////////////////////////////////////////////
    //Port config
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;            //Power up PORTB
    GPIOB->MODER |= (3 << 0);                       //Set PB0 to analog mode
    
    //ADC config sequence
    RCC->APB2ENR |= (1 << 8);	                    //Enable ADC1 clock (Bit8) 
    ADC1->CR1 &= ~(1 << 8);			                //SCAN mode disabled (Bit8)
	ADC1->CR1 &= ~(3 << 24);				        //12bit resolution (Bit24,25 0b00)
	ADC1->SQR1 |= (1 << 20);                        //Set number of conversions projected (L[3:0] 0b0001)
	ADC1->SQR3 &= ~(0x3FFFFFFF);	                //Clears whole 1st 30bits in register
	ADC1->SQR3 |= (8 << 0);			                //First conversion in regular sequence: PB0 as ADC1_In8
    ADC1->CR2 &= ~(1 << 1);			                //Single conversion
	ADC1->CR2 &= ~(1 << 11);			            //Right alignment of data bits  bit12....bit0
	ADC1->SMPR2 |= (7 << 0);	 		            //Sampling rate 480 cycles. 16MHz bus clock for ADC. 1/16MHz = 62.5ns. 480*62.5ns=30us
    ADC1->CR2 |= (1 << 0);                          //Switch on ADC1
    //ADC ready for use
    
    //Display init
	delay_ms(20); //Datasheet: "Wait for more than 15ms after VDD rises to 4.5V"
    lcd_init();
    delay_ms(20);
    defcustomcharacters();		
    delay_ms(20);
    lcd_cls();		
    	
    lcd_putstring(0, 0, (char*)"micromaker.de");
    
    GPIOC->ODR &= ~(1 << 13); //LED on
    
    while(1)
    { 
		ADC1->CR2 |=  (1 << 30); //Start 1 conversion SWSTART
		if(ADC1->SR & (1 << 1)) //Is EOC (End of Conversion) bit set?
		{
			ADC1->CR2 |=  (1 << 30);       //Start 1st conversion SWSTART
            while(!(ADC1->SR & (1 << 1))); //Wait until conversion is complete
            adcret = ADC1->DR;             //Read value from register
            
   		    volt = (double) adcret * vdd / 4096;     //Convert ADCval to voltage
	        rtherm = r1 / (vdd / volt - 1);          //Calculate current resistance of KTY81-210 thermistor
	        temp = (rtherm - 1690) / 13.88 * 10.0;   //Calculate temp based on function R.therm = m*T+y0 
	                                                 //derived from data sheet
            
			lcd_putstring(1, 0, (char*)"    ");			
			lcd_putnumber(1, 0, temp, -1, 1, 'l', 0);
			GPIOC->ODR &= ~(1 << 13);            //LED on
			delay_ms(100);
			GPIOC->ODR |= (1 << 13);            //LED off
			delay_ms(900);
		}
    }
	return 0;
}
