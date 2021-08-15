///////////////////////////////////////////////////////////////////  
/*                    SPI Demo + IL9341 for STM32F4              */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         STM32F411 "Black Pill" Board               */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      AUG 2021                                   */
///////////////////////////////////////////////////////////////////
//
//This "bare metal" demo shwos how to set up and use
//SPI interface on ARM Corex-M4 MCU and drive an IL9341 serial 
//LCD. 

//Connections are:

//DC =        PA2
//RES =       PA3 
//CS =        PB12
//SCK =       PB13
//MISO      = PB14 (not used in this example)
//MOSI(SDI) = PB15
//
//Hint: This is a simple example for educational purposes
//showing how to drive IL9341 with onboard SPI from an STM32F4 MCU. 

//This code inits IL9341 display and performs clearscren. That's all. 
//For a full code example including charcter set and display functions, 
//defined colors etc. please refer to 
//https://github.com/DK7IH/STM32-F4-Demos/blob/main/_Display/_ILI9341_SPI/ILI9341_SPI_LCD_driver.c


#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

//define ports for RST and DC line of IL9341 LCD connected to GPIOA
#define LCD_DC  2
#define LCD_RES 3
#define LCDPORT GPIOA->ODR

#define LCD_CMD   0
#define LCD_DATA  1

#define LCD_WIDTH   320
#define LCD_HEIGHT  240	

int main(void);
void lcd_write(uint8_t, uint8_t);
void lcd_init(void);
void lcd_cls(int);
void lcd_set_xy(int, int);
void lcd_draw_pixel(int);

static void delay_ms(unsigned int); 

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

void lcd_write(uint8_t cmd, uint8_t data)
{
	if(!cmd) //Cmd (0) or Data(1)?
	{
	    LCDPORT &= ~(1 << LCD_DC);  //Cmd=0
	}
	else
	{
	    LCDPORT |= (1 << LCD_DC);     //Data=1
	}
	
	GPIOB->ODR &= ~(1 << 12);       //CS low
	SPI2->DR = data;                 //Write data to SPI interface
	while (!(SPI2->SR & (1 << 1)));  //Wait till TX buf is clear
    GPIOB->ODR |= (1 << 12);        //CS high
}	

//Init LCD to vertical alignement and 16-bit color mode
void lcd_init(void)
{
    lcd_write(LCD_CMD,  0xCB);
    lcd_write(LCD_DATA, 0x39);
    lcd_write(LCD_DATA, 0x2C);
    lcd_write(LCD_DATA, 0x00);
    lcd_write(LCD_DATA, 0x34);
    lcd_write(LCD_DATA, 0x02);

    lcd_write(LCD_CMD, 0xCF);
    lcd_write(LCD_DATA, 0x00);
    lcd_write(LCD_DATA, 0XC1);
    lcd_write(LCD_DATA, 0X30);

    lcd_write(LCD_CMD,  0xE8);
    lcd_write(LCD_DATA, 0x85);
    lcd_write(LCD_DATA, 0x00);
    lcd_write(LCD_DATA, 0x78);

    lcd_write(LCD_CMD,  0xEA);
    lcd_write(LCD_DATA, 0x00);
    lcd_write(LCD_DATA, 0x00);

    lcd_write(LCD_CMD,  0xED);
    lcd_write(LCD_DATA, 0x64);
    lcd_write(LCD_DATA, 0x03);
    lcd_write(LCD_DATA, 0X12);
    lcd_write(LCD_DATA, 0X81);

    lcd_write(LCD_CMD,  0xF7);
    lcd_write(LCD_DATA, 0x20);

    lcd_write(LCD_CMD,  0xC0); // Power control
    lcd_write(LCD_DATA, 0x23); // VRH[5:0]

    lcd_write(LCD_CMD,  0xC1); // Power control
    lcd_write(LCD_DATA, 0x10); // SAP[2:0];BT[3:0]

    lcd_write(LCD_CMD,  0xC5); // VCM control
    lcd_write(LCD_DATA, 0x3e);
    lcd_write(LCD_DATA, 0x28);

    lcd_write(LCD_CMD,  0xC7); // VCM control2
    lcd_write(LCD_DATA, 0x86);

    lcd_write(LCD_CMD,  0x36); // Memory Access Control
    lcd_write(LCD_DATA, 0x88); // C8

    lcd_write(LCD_CMD,  0x3A);
    lcd_write(LCD_DATA, 0x55);

    lcd_write(LCD_CMD,  0xB1);
    lcd_write(LCD_DATA, 0x00);
    lcd_write(LCD_DATA, 0x18);

    lcd_write(LCD_CMD,  0xB6); // Display Function Control
    lcd_write(LCD_DATA, 0x08);
    lcd_write(LCD_DATA, 0x82);
    lcd_write(LCD_DATA, 0x27);

    lcd_write(LCD_CMD,  0xF2); // 3Gamma Function Disable
    lcd_write(LCD_DATA, 0x00);

    lcd_write(LCD_CMD,  0x26); // Gamma curve selected
    lcd_write(LCD_DATA, 0x01);

    lcd_write(LCD_CMD,  0xE0); // Set Gamma
    lcd_write(LCD_DATA, 0x0F);
    lcd_write(LCD_DATA, 0x31);
    lcd_write(LCD_DATA, 0x2B);
    lcd_write(LCD_DATA, 0x0C);
    lcd_write(LCD_DATA, 0x0E);
    lcd_write(LCD_DATA, 0x08);
    lcd_write(LCD_DATA, 0x4E);
    lcd_write(LCD_DATA, 0xF1);
    lcd_write(LCD_DATA, 0x37);
    lcd_write(LCD_DATA, 0x07);
    lcd_write(LCD_DATA, 0x10);
    lcd_write(LCD_DATA, 0x03);
    lcd_write(LCD_DATA, 0x0E);
    lcd_write(LCD_DATA, 0x09);
    lcd_write(LCD_DATA, 0x00);

    lcd_write(LCD_CMD,  0xE1); // Set Gamma
    lcd_write(LCD_DATA, 0x00);
    lcd_write(LCD_DATA, 0x0E);
    lcd_write(LCD_DATA, 0x14);
    lcd_write(LCD_DATA, 0x03);
    lcd_write(LCD_DATA, 0x11);
    lcd_write(LCD_DATA, 0x07);
    lcd_write(LCD_DATA, 0x31);
    lcd_write(LCD_DATA, 0xC1);
    lcd_write(LCD_DATA, 0x48);
    lcd_write(LCD_DATA, 0x08);
    lcd_write(LCD_DATA, 0x0F);
    lcd_write(LCD_DATA, 0x0C);
    lcd_write(LCD_DATA, 0x31);
    lcd_write(LCD_DATA, 0x36);
    lcd_write(LCD_DATA, 0x0F);

    lcd_write(LCD_CMD,  0x11); // Sleep out
    delay_ms(120);
    lcd_write(LCD_CMD, 0x2c);  
        
    lcd_write(LCD_CMD, 0x29); // Display on 
    lcd_write(LCD_CMD, 0x2c);	
	
}	

void lcd_cls(int bcolor)
{
	int x;
	unsigned char y;
	
	lcd_set_xy(0, 0);
	for(x = 0; x < LCD_WIDTH; x++)
	{
        for(y = 0; y < LCD_HEIGHT; y++)
        {
			lcd_draw_pixel(bcolor);
		}	
	}
}		

void lcd_set_xy(int x, int y)
{
	//X
	lcd_write(LCD_CMD, 0x2B);
    lcd_write(LCD_DATA, x >> 8);
    lcd_write(LCD_DATA, x & 0xFF);
    lcd_write(LCD_CMD, 0x2c);

    //Y 
    lcd_write(LCD_CMD, 0x2A);
    lcd_write(LCD_DATA, y >> 8);
    lcd_write(LCD_DATA, y & 0xFF);
    lcd_write(LCD_CMD, 0x2c);
}

void lcd_draw_pixel(int color)
{
    lcd_write(LCD_DATA, color >> 8);
    lcd_write(LCD_DATA, color & 0xFF);
}

int main(void)
{
	//////////////////////////////////////////
    // Setup LED
    //////////////////////////////////////////
	//Turn on the GPIOC peripheral for LED
    RCC->AHB1ENR |= (1 << 2); 
	GPIOC->MODER |= (1 << (13 << 1));	

    //////////////////////////////////////////////
    // Set SystemClock to 100 MHz with 25 MHz HSE
    //////////////////////////////////////////////
    FLASH->ACR |= 0b010;                         //2 wait state for 100 MHz
    RCC->CR |= (1 << 16);                        //Activate external clock (HSE: 8 MHz)
    while ((RCC->CR & (1 << 17)) == 0);          //Wait until HSE is ready
    
    //Configuration of PLL
    RCC->PLLCFGR |= (1 << 22);                  //PLL source is HSE
    
                                                //Set PLLM
    RCC->PLLCFGR &= ~0x3F;                      //1st Reset bits
    RCC->PLLCFGR |= 20;                          //2nd define VCO input frequency = PLL input clock frequency (f.HSE) / PLLM with 2 ≤ PLLM ≤ 63 
                                                //-> f.VCO.in = 25MHz / 8 = 1.25MHz
                                                
                                                //Set PLLN: PPLLN defines VCO out frequency
    RCC->PLLCFGR &= ~0x7FC0;                    //1st Reset bits 14:6
    RCC->PLLCFGR |= (160 << 6);                 //2nd define f.VCO.out = f.VCO.in * 160 = 200MHz
     
                                                //Set PLLP: Main PLL (PLL) division factor for main system clock; Reset Bits 17:16
    RCC->PLLCFGR &= ~(0b11 << 16);              //Reset bits 17:16
                                                //f.PLL.output.clock = f.VCO.out / 2 = 100MHz
                                                
                                                //Set PLLQ. PLLQ = division factor for USB OTG FS, SDIO and random number generator clocks
    RCC->PLLCFGR &= ~(0b1111 << 24);            //Reset bits 27:24
    RCC->PLLCFGR |= (8 << 24);                  //PLL-Q: f.VCO.out / 8 = 25MHz
        
    RCC->CR |= (1 << 24);                       //Activate PLL, Bit 24
    while ((RCC->CR & (1 << 25)) == 0);         //Wait until PLL is ready Bit 25
    
                                                //Division of clock signal for bus system
    RCC->CFGR |= (0b1001 << 4)                  //AHB divider:  100MHz / 4 = 25 MHz
              | (0b100 << 10)                   //APB1 divider: /2
              | (0b100 << 13);                  //APB2 divider: /2
               
    RCC->CFGR |= 0b10;                          //Switching to PLL clock source
    
    //////////////////////////////////////////
    // Setup SPI
    //////////////////////////////////////////
	//PB12:CS; PB13:SCK; PB14:MISO; PB15:MOSI
    
    RCC->AHB1ENR |= (1 << 0);                   //GPIOA power clock enable
    RCC->AHB1ENR |= (1 << 1);                   //GPIOB power clock enable
    RCC->APB1ENR |= (1 << 14);                  //Enable SPI1 clock, bit 12 in APB2ENR

    //Non-AF ports for LCD
    GPIOA->MODER |=  (1 << (LCD_RES << 1));     //PA3 as output (Reset)
    GPIOA->MODER |=  (1 << (LCD_DC << 1));      //PA2 as output (DC)
    
    //Alternate function ports
    GPIOB->MODER &= ~(0b11111111U << (12 << 1));    //Reset bits 15:12
    GPIOB->MODER |=  (0b01 << (12 << 1));           //PB12 (CS) as output
    GPIOB->MODER |=  (0b101010U << (13 << 1));      //Set bits 15:13 to 0b101010 for alternate function (AF5)
    GPIOB->OSPEEDR |= (0b11111111 << (12 << 1));    //Speed vy hi PB15:PB12
    
    //Set AF5 (0x05) for SPI2 in AF registers (PB13:PB15)
    GPIOB->AFR[1] |= (0x05 << (20)); //PB13
    GPIOB->AFR[1] |= (0x05 << (24)); //PB14
    GPIOB->AFR[1] |= (0x05 << (28)); //PB15
    
    //Set SPI2 properties
    SPI2->CR1 |= (1 << 2);     //Master mode
    SPI2->CR1 |= SPI_CR1_SSM;  //Software slave management enabled
    SPI2->CR1 |=  SPI_CR1_SSI; //Internal slave select The value of this bit is 
                               //forced onto the NSS pin and the IO value of the NSS pin is ignored.
    SPI2->CR1 |= SPI_CR1_SPE;  //SPI2 enable
    
    //Reset LCD
    delay_ms(500);
    LCDPORT &= ~(1 << LCD_RES);  //0
	delay_ms(50);
    LCDPORT |= 1 << LCD_RES;     //1
    delay_ms(50);
    
    lcd_init();
    
    while(1)
    {
		//Perform CLS just for fun :-)
		GPIOC->ODR &= ~(1 << 13);
        lcd_cls(0xAAAA);
        GPIOC->ODR |= (1 << 13); //Blink LED
        delay_ms(1000);
           
    }

    return 0;
}
