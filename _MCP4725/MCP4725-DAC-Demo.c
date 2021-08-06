///////////////////////////////////////////////////////////////////
/*                    MCP4725 DAC driver software                */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         F411E Discovery board by STM               */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

volatile uint8_t DeviceAddr = 0xC0U; //C0 with "ADDR" on chip open or GND, C1 with tied to VDD;

///////////////////////////
//Function declarations
///////////////////////////
int main(void);
void i2c_start(void);
void i2c_stop(void); 
void i2c_write(uint8_t, uint8_t);
void i2c_write_mcp4725(uint8_t, uint8_t, uint8_t);

uint8_t i2c_read(uint8_t); 

// Quick and dirty delay
static void delay (unsigned int time) 
{
    for (unsigned int i = 0; i < time; i++)
    {
        for (volatile unsigned int j = 0; j < 2000; j++);
    }    
}

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
void i2c_write(uint8_t regaddr, uint8_t data) 
{
    //Send start signal
    i2c_start();

    //Send device address
    I2C1->DR = DeviceAddr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Perform one dummy read to clear flags
    (void)I2C1->SR2;

    //Send register address for device
    I2C1->DR = regaddr;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send data
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send stop signal
    i2c_stop();
}

//For MCP4724 DAC with 3 parameters according to datasheet
void i2c_write_mcp4725(uint8_t data0, uint8_t data1, uint8_t data2 ) 
{
    //Send start signal
    i2c_start();

    //Send device address
    I2C1->DR = DeviceAddr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //One dummy read to clear flags
    (void)I2C1->SR2;
    
    //Send data byte 0
    I2C1->DR = data0;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
    
    //Send data byte 1
    I2C1->DR = data1;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send data byte 2
    I2C1->DR = data2;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send stop signal
    i2c_stop();
}

  ///////////////////
 //   Main code   //
/////////////////// 
int main(void)
{
	int value;
	
    //*******************************
    // Set up LEDs - GPIOD 12,13,14,15
    //*******************************
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
    GPIOB->OTYPER |= (1 << 6);        //open-drain
    GPIOB->MODER &= ~(3 << (9 << 1)); //PB9 as SDA
    GPIOB->MODER |=  (2 << (9 << 1)); //Alternate function
    GPIOB->OTYPER |= (1 << 9);        //open-drain

    //Choose AF4 for I2C1 in Alternate Function registers
    GPIOB->AFR[0] |= (4 << (6 << 2));     // for PB6
    GPIOB->AFR[1] |= (4 << ((9-8) << 2)); // for PB9

    //Reset and clear register
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    I2C1->CR2 |= (I2C_CR2_ITERREN); // enable error interrupt

    I2C1->CR2 |= (10 << 0); //10Mhz periph clock
    I2C1->CCR |= (50 << 0); 
    
    //Maximum rise time defintion
    I2C1->TRISE |= (11 << 0); //Program for value=11 => 100khz
        
    //Enable I2C    
    I2C1->CR1 |= I2C_CR1_PE; 
    
    value = 0;
    
    while(1)
    {
        GPIOD->ODR |= (1 << 12); //Green led off, blink shows success
        delay(10);
        //i2c_write(1, 2);
        
        i2c_write_mcp4725(64, (value >> 4) & 0xFF, value & 0xF0);
        if(value < 4095)
        {
			value += 10;
		}
		else
		{
			value = 0;
		}		
        GPIOD->ODR &= ~(1 << 12); //Green led on
        delay(10);
    }
    return 0; 
}
