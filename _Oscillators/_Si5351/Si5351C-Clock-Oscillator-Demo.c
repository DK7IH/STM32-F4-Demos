///////////////////////////////////////////////////////////////////
/*                    Si5351 driver software                     */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         F411E Discovery board by STM               */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

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
#define SI5351_ADDRESS 0xC0 //Check your module for correct address setting. IDs may vary!
#define PLLRATIO 36
#define CFACTOR 1048575

//Set of Si5351A register addresses
#define CLK_ENABLE_CONTROL       3
#define PLLX_SRC				15
#define CLK0_CONTROL            16 
#define CLK1_CONTROL            17
#define CLK2_CONTROL            18
#define SYNTH_PLL_A             26
#define SYNTH_PLL_B             34
#define SYNTH_MS_0              42
#define SYNTH_MS_1              50
#define SYNTH_MS_2              58
#define PLL_RESET              177
#define XTAL_LOAD_CAP          183

//SI5351 Declarations & frequency
void si5351_start(void);
void si5351_set_freq(int, unsigned long);

  //////////////////////
 // Si5351A commands //
//////////////////////

// Set PLLs (VCOs) to internal clock rate of 900 MHz
// Equation fVCO = fXTAL * (a+b/c) (=> AN619 p. 3
void si5351_start(void)
{
  unsigned long a, b, c;
  unsigned long p1, p2;//, p3;
    
  // Init clock chip
  i2c_write(XTAL_LOAD_CAP, 0xD2);      // Set crystal load capacitor to 10pF (default), 
                                          // for bits 5:0 see also AN619 p. 60
  i2c_write(CLK_ENABLE_CONTROL, 0x00); // Enable all outputs
  i2c_write(CLK0_CONTROL, 0x0F);       // Set PLLA to CLK0, 8 mA output
  i2c_write(CLK1_CONTROL, 0x2F);       // Set PLLB to CLK1, 8 mA output
  i2c_write(CLK2_CONTROL, 0x2F);       // Set PLLB to CLK2, 8 mA output
  i2c_write(PLL_RESET, 0xA0);          // Reset PLLA and PLLB

  // Set VCOs of PLLA and PLLB to 650 MHz
  a = PLLRATIO;     // Division factor 650/25 MHz !!!!
  b = 0;            // Numerator, sets b/c=0
  c = CFACTOR;      //Max. resolution, but irrelevant in this case (b=0)

  //Formula for splitting up the numbers to register data, see AN619
  p1 = 128 * a + (unsigned long) (128 * b / c) - 512;
  p2 = 128 * b - c * (unsigned long) (128 * b / c);
  //p3  = c;
  
  //Write data to registers PLLA and PLLB so that both VCOs are set to 900MHz intermal freq
  i2c_write(SYNTH_PLL_A, 0xFF);
  i2c_write(SYNTH_PLL_A + 1, 0xFF);
  i2c_write(SYNTH_PLL_A + 2, (p1 & 0x00030000) >> 16);
  i2c_write(SYNTH_PLL_A + 3, (p1 & 0x0000FF00) >> 8);
  i2c_write(SYNTH_PLL_A + 4, (p1 & 0x000000FF));
  i2c_write(SYNTH_PLL_A + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  i2c_write(SYNTH_PLL_A + 6, (p2 & 0x0000FF00) >> 8);
  i2c_write(SYNTH_PLL_A + 7, (p2 & 0x000000FF));

  i2c_write(SYNTH_PLL_B, 0xFF);
  i2c_write(SYNTH_PLL_B + 1, 0xFF);
  i2c_write(SYNTH_PLL_B + 2, (p1 & 0x00030000) >> 16);
  i2c_write(SYNTH_PLL_B + 3, (p1 & 0x0000FF00) >> 8);
  i2c_write(SYNTH_PLL_B + 4, (p1 & 0x000000FF));
  i2c_write(SYNTH_PLL_B + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  i2c_write(SYNTH_PLL_B + 6, (p2 & 0x0000FF00) >> 8);
  i2c_write(SYNTH_PLL_B + 7, (p2 & 0x000000FF));
}

void si5351_set_freq(int synth, unsigned long freq)
{
  unsigned long  a, b, c = CFACTOR; 
  unsigned long f_xtal = 25000000;
  double fdiv = (double) (f_xtal * PLLRATIO) / freq; //division factor fvco/freq (will be integer part of a+b/c)
  double rm; //remainder
  unsigned long p1, p2;
  
  a = (unsigned long) fdiv;
  rm = fdiv - a;  //(equiv. to fractional part b/c)
  b = rm * c;
  p1  = 128 * a + (unsigned long) (128 * b / c) - 512;
  p2 = 128 * b - c * (unsigned long) (128 * b / c);
      
  //Write data to multisynth registers of synth n
  i2c_write(synth, 0xFF);      //1048575 MSB
  i2c_write(synth + 1, 0xFF);  //1048575 LSB
  i2c_write(synth + 2, (p1 & 0x00030000) >> 16);
  i2c_write(synth + 3, (p1 & 0x0000FF00) >> 8);
  i2c_write(synth + 4, (p1 & 0x000000FF));
  i2c_write(synth + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
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
    //Setup LEDs - GPIOD 12,13,14,15 F411-E Discovery board
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~(0xFFU << 24);
    GPIOD->MODER |= (0x55 << 24);
    GPIOD->ODR    = 0x0000;
        
    //Enable I2C clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Setup I2C as GPIOB 6, 9  SCK, SDA
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
	si5351_set_freq(SYNTH_MS_0, 5000000);
	
    while(1)
    {
		//Blinking LED shows proper operation
        GPIOD->ODR |= (1 << 12); //LED on 
        delay(100);
        si5351_set_freq(SYNTH_MS_0, 5000000);
        GPIOD->ODR &= ~(1 << 12); //LED off
        delay(100);
    }
    return 0; 
}
