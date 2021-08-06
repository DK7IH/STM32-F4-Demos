///////////////////////////////////////////////////////////////////
//              I2C Basic functions for STM32F4                  //
///////////////////////////////////////////////////////////////////
// A "bare metal" implementation for basic I2C usage on          //   
// STM32F4 MCU - Sets STM32 as master and establishes an I2C     // 
// communication (wrtite or read) with another I2C module        //
///////////////////////////////////////////////////////////////////     
/*  Mikrocontroller:  ARM Cortex M4 (STM32F407)                  */
/*                    LEDs are for STM32 Discovery board!        */
/*  Compiler:         GCC (GNU AVR C-Compiler)                   */
/*  Autor:            Peter (DK7IH)                              */
/*  Letzte Aenderung: 2021-07-05                                 */
/*****************************************************************/
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

volatile uint8_t DeviceAddr = 0x78; //I2C_DEVICE_ADDRESS; here for OLED SH1106

///////////////////////////
//Function declarations
///////////////////////////
int main(void);
void i2c_start(void);
void i2c_stop(void); 
void i2c_write_1byte(uint8_t, uint8_t);
void i2c_write_nbytes(uint8_t*, uint8_t); 
uint8_t i2c_read1(uint8_t); 
uint8_t i2c_read2(uint16_t) 

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

void i2c_write_1byte(uint8_t regaddr, uint8_t data) 
{
    //Start signal
    i2c_start();

    //Send chipaddr to be targeted
    I2C1->DR = DeviceAddr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)); //Wait until transfer done
    //Perform one read to clear flags etc.
    (void)I2C1->SR2; //Clear address register

    //Send operation type/register 
    I2C1->DR = regaddr;
    while (!(I2C1->SR1 & I2C_SR1_BTF)); //Wait until transfer done

    //Send data
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));  //Wait until transfer done

    //Send stop signal
    i2c_stop();
}

//Multiple number of bytes (2)
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

//Address a byte register, return one byte
uint8_t i2c_read1(uint8_t regaddr) 
{
    uint8_t reg;

    //Start communication
    i2c_start();

    //Send device address
    I2C1->DR = DeviceAddr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Dummy read to clear flags
    (void)I2C1->SR2; //Clear addr register

    //Send operation type/register 
    I2C1->DR = regaddr;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Restart by sending stop & start sig
    i2c_stop();
    i2c_start();

    //Repeat like before
    I2C1->DR = DeviceAddr | 0x01; // read
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

//Address a word register, return one byte
uint8_t i2c_read2(uint16_t regaddr) 
{
    int8_t reg;
    int16_t r_msb, r_lsb;
    
    r_msb = (regaddr & 0xFF00) >> 8;
    r_lsb = regaddr & 0x00FF;

    //Start communication
    i2c_start();

    //Send device address
    I2C1->DR = device_addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Dummy read to clear flags
    (void)I2C1->SR2; //Clear addr register

    //Send operation type/register MSB
    I2C1->DR = r_msb;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send operation type/register LSB
    I2C1->DR = r_lsb;
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
    reg = I2C1->DR;

    //Send stop signal
    i2c_stop();

    return reg;
}


int main(void)
{
    ////////////////////////////////////////////////////
    // setup LEDs - GPIOD 12,13,14,15 (STM Disco-Board
    ////////////////////////////////////////////////////
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~(0xFFU << 24);
    GPIOD->MODER |= (0x55 << 24);
    GPIOD->ODR    = 0x0000;
    
    //////////////////////////////////////////
    // setup I2C - GPIOB 6 SCK, 9 SDA
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

    //Reset and clear register
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    //Set I2C clock
    I2C1->CR2 |= (10 << 0); // 10Mhz periph clock
    I2C1->CCR |= (50 << 0);
    //Maximum rise time.
    I2C1->TRISE |= (11 << 0); //Set TRISE to 11 eq. 100khz
    
    I2C1->CR1 |= I2C_CR1_PE; //Enable i2c
    // I2C init procedure accomplished. 
    
    while(1)
    {
        GPIOD->ODR |= (1 << 12); //Green led on ("Discoboard" only)
        delay(10);
        GPIOD->ODR &= ~(1 << 12); //Green led off ("Discoboard" only)
        delay(10);
    }
    return 0; 
}
