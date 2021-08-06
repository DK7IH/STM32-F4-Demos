///////////////////////////////////////////////////////////////////
//               BMP180 Sensor Demo for STM32F4                  //
///////////////////////////////////////////////////////////////////
// A "bare metal" implementation for basic I2C usage of this     //
// sensor on an STM32F4 MCU - Sets STM32 as master and sets up   // 
// an I2C conection to BMP180                                    // 
// Output: Temp in Deg. centigrade, air pressure in Pascal       //
// LCD:    DOT Matrix 20x4                                       //
///////////////////////////////////////////////////////////////////     
/*  Mikrocontroller:  ARM Cortex M4 (STM32F407)                  */
/*                    LEDs are for STM32 Blackpill board!        */
/*  Compiler:         GCC (GNU ARM C-Compiler)                   */
/*  Author:           Peter (DK7IH)  https://dk7ih.de            */
/*  Last change:      2021-07                                    */
///////////////////////////////////////////////////////////////////     
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

volatile uint8_t device_addr = 0xEE; //Device address BMP190
int oss = 0;

  ///////////////////////////////
 // I2C Function declarations //
///////////////////////////////
void i2c_start(void);
void i2c_stop(void); 
void i2c_write_1byte(uint8_t, uint8_t);
void i2c_write_nbytes(uint8_t*, uint8_t); 
int16_t  i2c_read(uint8_t); 

  ///////////////////
 //  LCD-Display  //
///////////////////
#define LCDGPIO GPIOA
#define LCD_D0 0 
#define LCD_D1 1 
#define LCD_D2 2 
#define LCD_D3 3 
#define LCD_RS 4 
#define LCD_RW 5 
#define LCD_E  6 

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

  ////////////////////////
 //  BMP180 functions  //
////////////////////////
int32_t BMP180_get_temp(void);
void BMP180_get_cvalues(void);
int32_t BMP180_get_pressure(void);

//BMP180 calibration data
int16_t ac1, ac2, ac3; 
uint16_t ac4, ac5, ac6;
int16_t b1, b2;
int32_t b5;
int16_t mb, mc, md;

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
// Write CMD or DATA item to LCD
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
		
	//Put pin D0..D3 in general purpose output mode again
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
	                                      //Must be remapped for other displays!
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
	//Basic settings of LCD
    //4-Bit mode, 5 pixels width matrix
    lcd_write(0, 0x28);
        
    //4-line mode
    lcd_write(0, 0x2C); //RE=1
    lcd_write(0, 0x09);
    lcd_write(0, 0x28); //RE=0
            
    //Display on, Cursor off, Blink off 
    lcd_write(0, 0x0C);
    
    //No display shift, no cursor move
    lcd_write(0, 0x04);
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
    {
        digits = 1;
    }   

    for(t1 = digits - 1; t1 >= 0; t1--)
    {
        x = 1;
        for(t2 = 0; t2 < t1; t2++)
        {
            x *= 10;
        }    
        r = n / x;
        cdigit[digitcnt++] = r + 48;

        if(t1 == dec) 
        {
            cdigit[digitcnt++] = 46;
        }   
        n -= r * x;
    }

    digitcnt--;
    t1 = 0;

    //Display on screen
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
			
                    while(cl <= col + digitcnt)                       //left
		            {
                        lcd_putchar(row, cl++, cdigit[t1++]);
					}	
                    break;

        case 'r':   t1 = digitcnt;                              //right
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

//CLS one line
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

    unsigned char customchar[]={0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, //Bar chars
	                            0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
	                            0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C,  
	                            0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 
	                            0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, //End
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
 //         I 2 C           //
/////////////////////////////
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

void i2c_write_1byte(uint8_t regaddr, uint8_t data) 
{
    //Start signal
    i2c_start();

    //Send chipaddr to be targeted
    I2C1->DR = device_addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)); //Wait until transfer done
    //Perform one dummy read to clear register flags etc.
    (void)I2C1->SR2; //Clear addr reg

    //Send operation type/register 
    I2C1->DR = regaddr;
    while (!(I2C1->SR1 & I2C_SR1_BTF)); //Wait until transfer done

    //Send data
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));  //Wait

    //Send stop signal
    i2c_stop();
}

//Write multiple number of bytes (>1)
void i2c_write_nbytes(uint8_t *data, uint8_t n) 
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

int16_t i2c_read(uint8_t regaddr) 
{
    int16_t reg;

    //Start communication
    i2c_start();

    //Send device address
    I2C1->DR = device_addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Dummy read to clear flags
    (void)I2C1->SR2; //Clear addr register

    //Send operation type/register 
    I2C1->DR = regaddr;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Restart by sending stop & start sig
    i2c_stop();
    i2c_start();

    //Repeat
    I2C1->DR = device_addr | 0x01; // read
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    //Wait until data arrived in receive buffer
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    //Read value
    reg = (uint8_t)I2C1->DR;

    //Send stop signal
    i2c_stop();

    return reg;
}

   /////////////////////////////
  //         BMP180          //
 //    Temp. and ambient    //  
//        air pressure     //
////////////////////////////
//Gets uncorrected pressure data
//from ADC
int32_t BMP180_get_pressure(void)
{
	int32_t up;
	
	i2c_write_1byte(0xF4, 0x34 + (oss << 6));
	delay_ms(10);
	
	up = i2c_read(0xF6) << 8;
	delay_ms(10);
	up |= i2c_read(0xF7);
	
	return up;
}

//Calc real pressure by algorithm according to
//data sheet
int32_t BMP180_calc_real_pressure(int32_t up)
{
    int32_t x1, x2, x3;
	int32_t b3, b6, p;		
	uint32_t b4, b7;
		
	b6 = b5 - 4000L;
    x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((ac1 * 4 + x3) << oss) + 2) >> 2;
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * ((unsigned long)(x3 + 32768))) >> 15;
    b7 = ((unsigned long)up - b3) * (50000L >> oss);
    
	if(b7 < 0x80000000L) 
	{
		p = ((b7 << 1) / b4);
	}
	else
	{
		p = ((b7 / b4) << 1);
	}	
	
    x1 = (p >> 8) * (p >> 8); //Weird!
    x1 = (x1 * 3038L) >> 16;
    x2 = (-7357L * p) >> 16;
    p += ((x1 + x2 + 3791L) >> 4);
	
	return p;
}

//Read "raw" temperature from BMP180
int32_t BMP180_get_temp(void)
{
	uint32_t ut;
		
	i2c_write_1byte(0xF4, 0x2E);
	delay_ms(10);
	
	ut = i2c_read(0xF6) << 8;		
	ut |= i2c_read(0xF7);
		
	return ut;
}

//Calc real temp
uint32_t BMP180_calc_real_temp(long ut)
{
	int32_t t;
	int32_t x1, x2;
	
    //Calculate temperature
	x1 = ((ut - ac6) * ac5) >> 15;
	x2 = (mc << 11) / (x1 + md);
	b5 = x1 + x2;
	t = (b5 + 8) >> 4;
	
	return t;
}

//Get the calibration values from BMP180 sensor
//from EEPROM
void BMP180_get_cvalues(void)
{
	int msb, lsb;
	
    msb = i2c_read(0xAA);
    lsb = i2c_read(0xAB);
    ac1 = (msb << 8) + lsb;
    delay_ms(10);
    
	msb = i2c_read(0xAC);
    lsb = i2c_read(0xAD);
    ac2 = ((msb << 8) + lsb);
    delay_ms(10);
            
    msb = i2c_read(0xAE);
    lsb = i2c_read(0xAF);
    ac3 = ((msb << 8) + lsb);
    delay_ms(10);
                
    msb = i2c_read(0xB0);
    lsb = i2c_read(0xB1);
    ac4 = ((msb << 8) + lsb);
    delay_ms(10);
            
    msb = i2c_read(0xB2);
    lsb = i2c_read(0xB3);
    ac5 = ((msb << 8) + lsb);
    delay_ms(10);
        
    msb = i2c_read(0xB4);
    lsb = i2c_read(0xB5);
    ac6 = ((msb << 8) + lsb);
    delay_ms(10);
            
    msb = i2c_read(0xB6);
    lsb = i2c_read(0xB7);
    b1 = ((msb << 8) + lsb);
    delay_ms(10);
        
    msb = i2c_read(0xB8);
    lsb = i2c_read(0xB9);
    b2 = ((msb << 8) + lsb);
    delay_ms(10);
        
    msb = i2c_read(0xBA);
    lsb = i2c_read(0xBB);
    mb = ((msb << 8) + lsb);
    delay_ms(10);
    
    msb = i2c_read(0xBC);
    lsb = i2c_read(0xBD);
    mc = ((msb << 8) + lsb);
    delay_ms(10);
    
    msb = i2c_read(0xBE);
    lsb = i2c_read(0xBF);
    md = ((msb << 8) + lsb);
    delay_ms(10);     
}

  ///////////////////
 // Main programm //
///////////////////
int main(void)
{
    long t0, p0, tr, pr;
    
    //////////////////////////////////////////
    // Setup LCD
    //////////////////////////////////////////
	//Turn on the GPIOA peripheral
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //RCC->AHB1ENR |= (1<<0);
    
    //Put pin D0..D6in general purpose output mode
    LCDGPIO->MODER  |=  (1 << (LCD_D0 << 1));	
    LCDGPIO->MODER  |=  (1 << (LCD_D1 << 1));	
    LCDGPIO->MODER  |=  (1 << (LCD_D2 << 1));	
    LCDGPIO->MODER  |=  (1 << (LCD_D3 << 1));	
    LCDGPIO->MODER  |=  (1 << (LCD_RS << 1));	
    LCDGPIO->MODER  |=  (1 << (LCD_RW << 1));	
    LCDGPIO->MODER  |=  (1 << (LCD_E << 1));	
    
	//Display init procedure
	delay_ms(20); //Wait for more than 15ms after VDD rises to 4.5V
    lcd_init();
    lcd_cls();		
    
    defcustomcharacters();		
   
    //////////////////////////////////////////
    // Setup I2C - GPIOB 6 SCK, 9 SDA
    //////////////////////////////////////////
    //Enable I2C clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    //Setup I2C PB6 and PB9 pins
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~(3 << (6 << 1)); //PB6 as SCK
    GPIOB->MODER |=  (2 << (6 << 1)); //Alternate function
    GPIOB->OTYPER |= (1 << 6);        //open-drain
    GPIOB->MODER &= ~(3 << (9 << 1)); //PB9 as SDA
    GPIOB->MODER |=  (2 << (9 << 1)); //Alternate function
    GPIOB->OTYPER |= (1 << 9);        //open-drain

    //Choose AF4 for I2C1 in Alternate Function registers
    GPIOB->AFR[0] |= (4 << (6 << 2));     // for PB6 SCK
    GPIOB->AFR[1] |= (4 << ((9-8) << 2)); // for PB9 SDA

    //Turn on the GPIOA peripheral for LED
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //RCC->AHB1ENR |= (1<<0);
    GPIOC->MODER |= (1 << (13 << 1));	
    GPIOC->ODR |= (1 << 13);
	 
    //Reset and clear register
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    //Set I2C clock
    I2C1->CR2 |= (10 << 0); // 10Mhz periph clock
    I2C1->CCR |= (50 << 0);

    //Maximum rise time.
    I2C1->TRISE |= (11 << 0); //Set TRISE to 11 eq. 100khz
    
    I2C1->CR1 |= I2C_CR1_PE; //Enable i2c
    // I2C init procedure accomplished. //////////////////////////
    
    lcd_putstring(0, 0, (char*)"BMP180 DEMO");
    lcd_putstring(1, 0, (char*)"by DK7IH (C) 2021");
    
    //Check if sensor is available
    //Try to read chip ID
    if(i2c_read(0xD0) == 0x55) //Chip ID correct?
    {
		lcd_putstring(2, 0, (char*)"BMP180 OK.");
	}
	else	
	{
		lcd_putstring(2, 0, (char*)"BMP180 FAIL!");
		while(1); //Terminate program
	}

    //Read parameter values from EEPROM
    BMP180_get_cvalues();	     
    
    while(1)
    {
		GPIOC->ODR |= (1 << 13); //Blink LED
        delay_ms(500); 
        
        //Start measurement
        t0 = BMP180_get_temp();     //Temp first to get temp value for press. measurement
        p0 = BMP180_get_pressure(); //Get pressure ADC 
        
        tr = BMP180_calc_real_temp(t0);      //Calc real values
        pr = BMP180_calc_real_pressure(p0);

        lcd_putnumber(3, 1, tr, -1, 1, 'l', 0);
        lcd_putnumber(3, 9, pr, -1, -1, 'l', 0);

        GPIOC->ODR &= ~(1 << 13);
        delay_ms(500);
    }
    return 0; 
}
