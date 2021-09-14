///////////////////////////////////////////////////////////////////
/*                    Si5351 driver software                     */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         F411E BlackPill Board                      */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <math.h>

//I2C Ports for Si5351
//SCK: PB6
//SDA: PB9

  ///////////////////////////////
 // Function declarations I2C //
///////////////////////////////
int main(void);
void i2c_start(void);
void i2c_stop(void); 
void i2c_write(uint8_t, uint8_t);

  ////////////////////////
 // Defines for Si5351 //
////////////////////////
#define SI5351_ADDRESS 0xC0     //Check individual module for correct address setting. IDs may vary!
#define FXTAL          27000000 //Hz
#define PLLRATIO       32       //FXTAL * PLLRATIO = f.VCO

//Set of Si5351A relevant register addresses
#define CLK_ENABLE_CONTROL          3
#define PLLX_SRC				   15
#define CLK0_CONTROL               16 
#define CLK1_CONTROL               17
#define CLK2_CONTROL               18
#define SYNTH_PLL_A                26
#define SYNTH_PLL_B                34
#define SYNTH_MS_0                 42
#define SYNTH_MS_1                 50
#define SYNTH_MS_2                 58
#define SPREAD_SPECTRUM_PARAMETERS 149
#define PLL_RESET                  177
#define XTAL_LOAD_CAP              183

//SI5351 Declarations & frequency
void si5351_start(void);
void si5351_set_freq(int, unsigned long);

  //////////////////////
 // Si5351A commands //
//////////////////////
//Set PLLA (VCO) to internal clock rate of 900 MHz
//In this example PLLB is not used
//Equation fVCO = fXTAL * (a+b/c) => see AN619 p.3
void si5351_start(void)
{
  unsigned long a, b, c;
  unsigned long p1, p2;
    
  //Init
  i2c_write(PLLX_SRC, 0);              //Select XTAL as clock source for si5351C
  i2c_write(SPREAD_SPECTRUM_PARAMETERS, 0); //Spread spectrum diasble (Si5351 A or B only!
  i2c_write(XTAL_LOAD_CAP, 0xD2);      // Set crystal load capacitor to 10pF (default), 
                                       // for bits 5:0 see also AN619 p. 60
  i2c_write(CLK_ENABLE_CONTROL, 0x00); // Enable all outputs
  i2c_write(CLK0_CONTROL, 0x0E);       // Set PLLA to CLK0, 8 mA output
  i2c_write(CLK1_CONTROL, 0x0E);       // Set PLLA to CLK1, 8 mA output
  i2c_write(CLK2_CONTROL, 0x0E);       // Set PLLA to CLK2, 8 mA output
  i2c_write(PLL_RESET, (1 << 5));          // Reset PLLA and PLLB

  //Set VCO of PLLA to 864MHz
  a = PLLRATIO;     // Division factor 864/27 MHz
  b = 0;            // Numerator, sets b/c=0, See AN169 p.3!
  c = 0xFFFFF;      // Max. resolution, but irrelevant in this case as b=0. See AN169 p.3!

  //Formula for splitting up the numbers to register data, see AN619
  p1 = 128 * a + (unsigned long) floor(128 * b / c) - 512;
  p2 = 128 * b - c * (unsigned long) floor(128 * b / c);
    
  //Write data to registers of PLLA so that VCO is set to 864MHz internal freq
  i2c_write(SYNTH_PLL_A, 0xFF);
  i2c_write(SYNTH_PLL_A + 1, 0xFF);
  i2c_write(SYNTH_PLL_A + 2, (p1 & 0x00030000) >> 16);
  i2c_write(SYNTH_PLL_A + 3, (p1 & 0x0000FF00) >> 8);
  i2c_write(SYNTH_PLL_A + 4, (p1 & 0x000000FF));
  i2c_write(SYNTH_PLL_A + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  i2c_write(SYNTH_PLL_A + 6, (p2 & 0x0000FF00) >> 8);
  i2c_write(SYNTH_PLL_A + 7, (p2 & 0x000000FF));

}

void si5351_set_freq(int synth, unsigned long freq)
{
  unsigned long  a, b, c = 0xFFFFF; 
  unsigned long f_xtal = FXTAL;
  double fdiv = (double) (f_xtal * PLLRATIO) / freq; //division factor fvco/freq (will be integer part of a+b/c)
  double rm; //remaining
  unsigned long p1, p2, p3;
  
  a = (unsigned long) fdiv;
  rm = fdiv - a;  //(equiv. to fractional part b/c)
  b = (unsigned long) (rm * c);
  p1  = 128 * a + (unsigned long) floor(128 * b / c) - 512;
  p2 = 128 * b - c * (unsigned long) floor(128 * b / c);
  p3 = c;
      
  //Write data to multisynth registers of synth
  i2c_write(synth + 0, (p3 & 0xFF00) >> 8);  
  i2c_write(synth + 1, p3 &  0xFF);      
  i2c_write(synth + 2, (p1 >> 16) & 0x03);
  i2c_write(synth + 3, (p1 & 0xFF00) >> 8);
  i2c_write(synth + 4, (p1 & 0xFF));
  i2c_write(synth + 5, (p3 & 0xF0) | ((p2 >> 16) & 0x0F));
  i2c_write(synth + 6, (p2 & 0x0000FF00) >> 8);
  i2c_write(synth + 7, (p2 & 0x000000FF));
}

// Quick and dirty delay
static void delay (unsigned int time) 
{
    for (unsigned int i = 0; i < time; i++)
    {
        for (volatile unsigned int j = 0; j < 2000; j++);
    }    
}

  //////////////////////
 //   I2C commands   //
//////////////////////
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

void i2c_write(uint8_t regaddr, uint8_t data) 
{
    //Send start signal
    i2c_start();

    //Send device identifier
    I2C1->DR = SI5351_ADDRESS;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Dummy read to clear flags
    (void)I2C1->SR2;

    //Send register addr
    I2C1->DR = regaddr;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    I2C1->DR = data; //Send data
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send stop signal
    i2c_stop();
}

  //////////////////////
 //   Main program   //
//////////////////////
int main(void)
{
    ////////////////////////////////////////// 
    // Setup LED
    //////////////////////////////////////////
	//Turn on the GPIOC peripheral for LED
    RCC->AHB1ENR |= (1 << 2); 
	GPIOC->MODER |= (1 << (13 << 1));	//Set PC13 for output        
    //Enable I2C clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Setup I2C as GPIOB 6 SCL, 9 SDA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~(3 << (6 << 1)); //PB6 as SCK
    GPIOB->MODER |=  (2 << (6 << 1)); //Alternate function
    GPIOB->OTYPER |= (1 << 6);        //open-drain
    GPIOB->MODER &= ~(3 << (9 << 1)); //PB9 as SDA
    GPIOB->MODER |=  (2 << (9 << 1)); //Alternate function
    GPIOB->OTYPER |= (1 << 9);        //open-drain

    //Choose AF4 option for I2C1 in Alternate Function registers
    GPIOB->AFR[0] |= (4 << (6 << 2));     // for PB6
    GPIOB->AFR[1] |= (4 << ((9 - 8) << 2)); // for PB9

    //Reset and clear control register
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    //Enable error interrupt
    I2C1->CR2 |= (I2C_CR2_ITERREN); 

    //Set I2C clock
    I2C1->CR2 |= (10 << 0); //10Mhz peripheral clock
    I2C1->CCR |= (50 << 0);
    //Maximum rise time set
    I2C1->TRISE |= (11 << 0); //TRISE=11ns for 100khz
    
    //Enable I2C
    I2C1->CR1 |= I2C_CR1_PE; 
    
    //Init Si5351
    si5351_start();
    
    //Set a frequency for check
	si5351_set_freq(SYNTH_MS_0, 1000000);
	
    while(1)
    {
		//Blinking LED shows proper operation
        GPIOC->ODR &= ~(1 << 13); //LED on 
        delay(100);
        si5351_set_freq(SYNTH_MS_0, 30000000);
        GPIOC->ODR |= (1 << 13); //LED off
        delay(100);
    }
    return 0; 
}
