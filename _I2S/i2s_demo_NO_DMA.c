///////////////////////////////////////////////////////////////////  
/*                    I2S Sound generator for STM32F4            */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         STM32F411 Discovery board                  */
/*                    with CS43L22 DAC Module                    */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      AUG 2021                                   */
///////////////////////////////////////////////////////////////////
//
//This "bare metal" demo puts out a sine wave on both audio 
//channels of the CS43L22 sound chip integrated on the 
//Discovry board. No DMA used so far.
//Demonstration setting up MCU ports, clock speed (168MHz)
//CS43L22, SPI3/I2S interface plus simple sine wave generation

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <math.h>

#define I2C_DEVICE_CHIP_ID 0x1C //Identifier of chip (read only)
#define I2C_DEVICE_REG_ID  0x01 //I2C regsiter whre ID is found

//List of all the defines
#define POWER_CONTROL1				0x02
#define POWER_CONTROL2				0x04
#define CLOCKING_CONTROL 	  		0x05
#define INTERFACE_CONTROL1			0x06
#define INTERFACE_CONTROL2			0x07
#define PASSTHROUGH_A				0x08
#define PASSTHROUGH_B				0x09
#define MISCELLANEOUS_CONTRLS		0x0E
#define PLAYBACK_CONTROL			0x0F
#define PASSTHROUGH_VOLUME_A		0x14
#define PASSTHROUGH_VOLUME_B		0x15
#define PCM_VOLUME_A				0x1A
#define PCM_VOLUME_B				0x1B
#define CONFIG_00					0x00
#define CONFIG_47					0x47
#define CONFIG_32					0x32

#define CS43L22_REG_MASTER_A_VOL    0x20
#define CS43L22_REG_MASTER_B_VOL    0x21
#define CS43_MUTE				 	0x00
#define CS43_RIGHT				    0x01
#define CS43_LEFT				 	0x02
#define CS43_RIGHT_LEFT	 	        0x03

#define VOLUME_CONVERT_A(Volume)    (((Volume) > 100)? 255:((uint8_t)(((Volume) * 255) / 100))) 
#define VOLUME_CONVERT_D(Volume)    (((Volume) > 100)? 24:((uint8_t)((((Volume) * 48) / 100) - 24))) 

volatile uint8_t device_addr = 0x94U; //I2C_DEVICE_ADDRESS;

///////////////////////////
//Function declarations
///////////////////////////
int main(void);
void i2c_start(void);
void i2c_stop(void); 
void i2c_write(uint8_t, uint8_t);
uint8_t i2c_read(uint8_t); 

void cs43_set_volume(uint8_t);
void cs43_enable_right_left(uint8_t);
void cs43_set_volume(uint8_t);
void cs43_start(void);

//Sine wave table 0 <= Phi <= 360°
uint16_t wave[] = {
16384, 16670, 16956, 17241, 17527, 17812, 18097, 18381, 18664, 18947, 19229, 19510,
19790, 20070, 20348, 20624, 20900, 21174, 21447, 21718, 21988, 22256, 22522, 22786,
23048, 23308, 23566, 23822, 24076, 24327, 24576, 24822, 25066, 25307, 25546, 25781, 
26014, 26244, 26471, 26695, 26915, 27133, 27347, 27558, 27765, 27969, 28170, 28366,
28560, 28749, 28935, 29117, 29295, 29469, 29639, 29805, 29967, 30125, 30278, 30428,
30573, 30714, 30850, 30982, 31110, 31233, 31352, 31466, 31575, 31680, 31780, 31875,
31966, 32052, 32133, 32210, 32281, 32348, 32410, 32467, 32519, 32566, 32609, 32646,
32678, 32706, 32728, 32746, 32758, 32766, 32768, 32766, 32758, 32746, 32728, 32706,
32678, 32646, 32609, 32566, 32519, 32467, 32410, 32348, 32281, 32210, 32133, 32052,
31966, 31875, 31780, 31680, 31575, 31466, 31352, 31233, 31110, 30982, 30850, 30714,
30573, 30428, 30278, 30125, 29967, 29805, 29639, 29469, 29295, 29117, 28935, 28749,
28560, 28366, 28170, 27969, 27765, 27558, 27347, 27133, 26915, 26695, 26471, 26244,
26014, 25781, 25546, 25307, 25066, 24822, 24576, 24327, 24076, 23822, 23566, 23308,
23048, 22786, 22522, 22256, 21988, 21718, 21447, 21174, 20900, 20624, 20348, 20070,
19790, 19510, 19229, 18947, 18664, 18381, 18097, 17812, 17527, 17241, 16956, 16670,
16384, 16098, 15812, 15527, 15241, 14956, 14671, 14387, 14104, 13821, 13539, 13258,
12978, 12698, 12420, 12144, 11868, 11594, 11321, 11050, 10780, 10512, 10246, 9982,
9720,  9460,  9202,  8946,  8692,  8441,  8192,  7946,  7702,  7461,  7222,  6987, 
6754,  6524,  6297,  6073,  5853,  5635,  5421,  5210,  5003,  4799,  4598,  4402, 
4208,  4019,  3833,  3651,  3473,  3299,  3129,  2963,  2801,  2643,  2490,  2340, 
2195,  2054,  1918,  1786,  1658,  1535,  1416,  1302,  1193,  1088,  988,   893, 
802,   716,   635,   558,   487,   420,   358,   301,   249,   202,   159,   122,
90,    62,    40,    22,    10,    2,     0,     2,     10,    22,    40,    62, 
90,    122,   159,   202,   249,   301,   358,   420,   487,   558,   635,   716,
802,   893,   988,   1088,  1193,  1302,  1416,  1535,  1658,  1786,  1918,  2054, 
2195,  2340,  2490,  2643,  2801,  2963,  3129,  3299,  3473,  3651,  3833,  4019, 
4208,  4402,  4598,  4799,  5003,  5210,  5421,  5635,  5853,  6073,  6297,  6524,
6754,  6987,  7222,  7461,  7702,  7946,  8192,  8441,  8692,  8946,  9202,  9460, 
9720,  9982,  10246, 10512, 10780, 11050, 11321, 11594, 11868, 12144, 12420, 12698, 
12978, 13258, 13539, 13821, 14104, 14387, 14671, 14956, 15241, 15527, 15812, 16098};

///////////////////////////
//I2S Function definitions
///////////////////////////
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
    //Send start condition
    i2c_start();

    //Send chipaddr in write mode
    //Wait until address is sent
    I2C1->DR = device_addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Dummy read to clear flags
    (void)I2C1->SR2; // clear addr condition

    //Send MAP byte with auto increment off
    //Wait until byte transfer complete (BTF)
    I2C1->DR = regaddr;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send data
    //Wait until byte transfer complete
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send stop condition
    i2c_stop();
}

uint8_t i2c_read(uint8_t regaddr) 
{
    uint8_t reg;

    //Send start condition
    i2c_start();

    //Send chipaddr in write mode
    //Wait until address is sent
    I2C1->DR = device_addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Dummy read to clear flags
    (void)I2C1->SR2; //Clear addr

    //Send MAP byte with auto increment off
    //Wait until byte transfer complete (BTF)
    I2C1->DR = regaddr;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Restart transmission by sending stop & start
    i2c_stop();
    i2c_start();

    //Send chipaddr in read mode. LSB is 1
    //Wait until address is sent
    I2C1->DR = device_addr | 0x01; // read
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    // dummy read to clear flags
    (void)I2C1->SR2; // clear addr condition

    // wait until receive buffer is not empty
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    // read content
    reg = (uint8_t)I2C1->DR;

    // send stop condition
    i2c_stop();

    return reg;
}

////////////////////////////////
//CS43L22 Function definitions
///////////////////////////////
//Enable Right and Left headphones
void cs43_enable_right_left(uint8_t side)
{
    uint8_t xdata;	
    
	switch (side)
	{
		case 0:	xdata =  (3 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			    xdata |= (3 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			    break;
		case 1: xdata =  (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			    xdata |= (3 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			    break;
		case 2: xdata =  (3 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			    xdata |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			    break;
		case 3:	xdata =  (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			    xdata |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			    break;
		default:break;
	}
	xdata |= (3 << 2);  // PDN_SPKB[0:1] = 11 (Speaker B always off)
	xdata |= (3 << 0);  // PDN_SPKA[0:1] = 11 (Speaker A always off)
	i2c_write(POWER_CONTROL2, xdata);
}


//Set Volume Level
void cs43_set_volume(uint8_t volume)
{
	int8_t temp_vol = volume - 50;
	
	temp_vol = temp_vol*(127/50);
	
	uint8_t my_volume = (uint8_t )temp_vol;
	
	i2c_write(PASSTHROUGH_VOLUME_A, my_volume);
	i2c_write(PASSTHROUGH_VOLUME_B, my_volume);
	
	//Set the Master volume
	i2c_write(CS43L22_REG_MASTER_A_VOL, VOLUME_CONVERT_D(volume));
	i2c_write(CS43L22_REG_MASTER_B_VOL, VOLUME_CONVERT_D(volume));
}

//Start the Audio DAC
void cs43_start(void)
{
	uint8_t xdata[16];	
	
	//Write 0x99 to register 0x00.
	i2c_write(CONFIG_00, 0x99);
	
	//Write 0x80 to register 0x47.
	i2c_write(CONFIG_47, 0x80);
	
	//Write '1'b to bit 7 in register 0x32.
	xdata[0] = i2c_read(CONFIG_32);
	xdata[0] |= 0x80;
	i2c_write(CONFIG_32, xdata[0]);
	
	// Write '0'b to bit 7 in register 0x32.
	xdata[0] = i2c_read(CONFIG_32);
	xdata[0] &= ~(0x80);
	i2c_write(CONFIG_32, xdata[0]);
	
	// Write 0x00 to register 0x00.
	i2c_write(CONFIG_00, 0x00);
	//Set the "Power Ctl 1" register (0x02) to 0x9E
	i2c_write(POWER_CONTROL1, 0x9E);
}

void CS43_Stop(void)
{
	i2c_write(POWER_CONTROL1, 0x01);
}

//////////////////////////////////////////////////
//Main code starts from here
//////////////////////////////////////////////////
int main(void)
{
	uint8_t xdata;
    uint8_t ret	;
	uint32_t t0, t1, t2;
	
	/*
    float sample_dt = F_OUT/F_SAMPLE;
	float sample_n = F_SAMPLE/F_OUT;
	float sine_val;
	*/	
    ///////////////////////////////////
    // setup LEDs - GPIOD 12,13,14,15
    ///////////////////////////////////
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~(0xFFU << 24);
    GPIOD->MODER |= (0x55 << 24);
    GPIOD->ODR    = 0x0000;

    //////////////////////////////////////////////
    // Set SystemClock to 168 MHz with 8 MHz HSE
    //////////////////////////////////////////////
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;        //2 wait state for 168 MHz
    RCC->CR |= RCC_CR_HSEON;                    //Activate external clock (HSE: 8 MHz)
    while ((RCC->CR & RCC_CR_HSERDY) == 0);     //Wait until HSE is ready
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;     //PLL source is HSE
    
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;          //PLL-M: VCO input frequency = PLL input clock frequency / PLLM with 2 ≤ PLLM ≤ 63
    RCC->PLLCFGR |= 4 << RCC_PLLCFGR_PLLM_Pos;  // -> f.VCO.in = 8MHz / 4 = 2MHz
                                                
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
    RCC->PLLCFGR |= 168 << RCC_PLLCFGR_PLLN_Pos; //PLL-N: f.VCO.out = f.VCO.in * 100 = 336MHz
    
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;          //PLL-P: Main PLL (PLL) division factor for main system clock
                                                //f.PLL.output.clock = f.VCO.out / 2 = 168MHz
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;          //Main PLL (PLL) division factor for USB OTG FS, SDIO and random number generator clocks
    RCC->PLLCFGR |= 7 << RCC_PLLCFGR_PLLQ_Pos;  //PLL-Q: f.VCO.out / 8 = 24MHz
        
    RCC->CR |= RCC_CR_PLLON;                    //Activate PLL (Output: 168 MHz)
    while((RCC->CR & RCC_CR_PLLRDY) == 0);      //Wait until PLL is ready
    
    //Division by 2 of clk signal       
    RCC->CFGR |= RCC_CFGR_HPRE_DIV2             //AHB divider:  /2
              | RCC_CFGR_PPRE1_DIV2             //APB1 divider: /2
              | RCC_CFGR_PPRE2_DIV2;            //APB2 divider: /2
               
    RCC->CFGR |= RCC_CFGR_SW_PLL;               //Switching to PLL clock source
    
    //////////////////////////////////////////
    //Setup I2C - GPIOB 6, 9
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
    GPIOB->AFR[0] |= (4 << (6 << 2));     //For PB6 26dec.
    GPIOB->AFR[1] |= (4 << ((9-8) << 2)); //For PB9 64dec.

    //Reset and clear register
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    I2C1->CR2 |= (10 << 0); //10Mhz periph clock
    I2C1->CCR |= (50 << 0);
    I2C1->TRISE |= (11 << 0); //Program TRISE to 11 for 100khz
        
    I2C1->CR1 |= I2C_CR1_PE; //Enable I2C

    //////////////////////////////////////////
    //Setup reset pin for CS43L22 - GPIOD 4
    //////////////////////////////////////////
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~(3U << 4*2);
    GPIOD->MODER |=  (1 << 4*2);

    //Activate CS43L22
    GPIOD->ODR   |=  (1 << 4);

    //Read Chip ID - first 5 bits of CHIP_ID_ADDR
    ret = i2c_read(I2C_DEVICE_REG_ID);

    if ((ret >> 3) != I2C_DEVICE_CHIP_ID)
    {
        GPIOD->ODR |= (1 << 13); //Orange LED ERROR
        while(1);                //Terminate program here
    }
    else
    {
        GPIOD->ODR |= (1 << 15); //Blue LED OK
    }
    
    //////////////////////////////////////////////
    //INIT CS43L22 DAC Module
    //////////////////////////////////////////////
    //PWR down
    i2c_write(POWER_CONTROL1, 0x01);
    
    //Enable Right and Left headphones
    xdata =  (2 << 6);  //PDN_HPB[0:1]  = 10 (HP-B always onCon)
	xdata |= (2 << 4);  //PDN_HPA[0:1]  = 10 (HP-A always on)
	xdata |= (3 << 2);  //PDN_SPKB[0:1] = 11 (Speaker B always off)
	xdata |= (3 << 0);  //PDN_SPKA[0:1] = 11 (Speaker A always off)
    i2c_write(POWER_CONTROL2, xdata);
    
    //Automatic clock detection
    i2c_write(CLOCKING_CONTROL, (1 << 7));
    
    //Interface control 1
    xdata = i2c_read(INTERFACE_CONTROL1);
	xdata &= (1 << 5); // Clear all bits except bit 5 which is reserved
	xdata &= ~(1 << 7);  // Slave
	xdata &= ~(1 << 6);  // Clock polarity: Not inverted
	xdata &= ~(1 << 4);  // No DSP mode
	xdata &= ~(1 << 2);  // Left justified, up to 24 bit (default)
	xdata |= (1 << 2);
    i2c_write(INTERFACE_CONTROL1, xdata);
    
    //PASSTHROUGH_A settings
    xdata = i2c_read(PASSTHROUGH_A);
    xdata &= 0xF0;      // Bits [7:4] are reserved
	xdata |=  (1 << 0); //Use AIN1A as source for passthrough
	i2c_write(PASSTHROUGH_A, xdata);
    
    //Passthrough B settings
    xdata = i2c_read(PASSTHROUGH_B);
	xdata &= 0xF0;      //Bits [4-7] are reserved
	xdata |=  (1 << 0); // Use AIN1B as source for passthrough
	i2c_write(PASSTHROUGH_B, xdata);
	
	i2c_write(INTERFACE_CONTROL1, (3 << 0)); //16-bit audio word length for I2S interface
	
    //Interface control 2
    i2c_write(INTERFACE_CONTROL2, (1 << 6)); //SCLK=MCLK
		
	//MISC settings
	i2c_write(MISCELLANEOUS_CONTRLS, 0x02);
    
    i2c_write(PLAYBACK_CONTROL, 0x00);
    i2c_write(PASSTHROUGH_VOLUME_A,0x00);
	i2c_write(PASSTHROUGH_VOLUME_B,0x00);
	i2c_write(PCM_VOLUME_A,0x00);
	i2c_write(PCM_VOLUME_B,0x00);
	
    cs43_set_volume(50); //0 - 100
	cs43_enable_right_left(CS43_RIGHT_LEFT);
    cs43_start();    
    ////////////////////End of INIT-Procedure ////////////////////////////
    
    /////////////////////////////////////////////////////////////////
    //Configure I2S-Ports for I2S_MCK, I2S_SCK, I2S_SD, I2S__WS
    /////////////////////////////////////////////////////////////////
    RCC->AHB1ENR |= (1 << 2);            //Enable PORTC
    GPIOC->MODER |= (2 << (7 << 1));     //I2S_MCK PC7  (AF)
    GPIOC->MODER |= (2 << (10 << 1));    //I2S_SCK PC10 (AF) 
    GPIOC->MODER |= (2 << (12 << 1));    //I2S_SD PC12  (AF)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //(1 << 0); //(Re)enable PORTA
    GPIOA->MODER |= (2 << (4 << 1));     //I2S_WS PA4 (AF)
      
    //Set AF-registers  
    GPIOC->AFR[0] |= (0b0110 << 28); //PC7  AF6
    GPIOC->AFR[1] |= (0b0110 << 8);  //PC10 AF6
    GPIOC->AFR[1] |= (0b0110 << 16); //PC12 AF6
    GPIOA->AFR[0] |= (0b0110 << 16); //PA4  AF6
    
    ///////////////////
    //Configure SPI3
    ///////////////////
    RCC->APB1ENR |= (1 << 15);  //Enable SPI3 clock
    SPI3->I2SCFGR |= (1 << 11); //I2S mode selected
    SPI3->I2SCFGR |= (2 << 8);  //Master TX
    SPI3->I2SCFGR &= ~(2 << 4); //Standard Phillips 
    SPI3->I2SCFGR &= ~(2 << 2); //Data format 16 bits
    SPI3->I2SCFGR &= ~(1 << 0); //CHLEN=16 bits
    
    SPI3->I2SPR |= (1 << 9);             //Master clock output enable        
    SPI3->I2SPR |= (2 << 3) | (1 << 8);  //Prescaler /8 and ODD-factor=1
    
    //Start PLL for I2S
    RCC->CR |= (1 << 26);           //PLLI2S on
    while(!(RCC->CR & (1 << 27)));  //Wait until PLL is ready
    
    SPI3->I2SCFGR |= (1 << 10);    //I2S enabled
    
    while(1)
    {
        //Build Sine wave, cheap AND dirty, just for demo!
	    for(t0 = 0; t0 < 360; t0++)
	    {
			for(t1 = 0; t1 < 2; t1++)
			{
			    SPI3->DR = wave[t0] << 1;
			    for(t2 = 0; t2 < 10; t2++){};
			}    
	    } 
    }
    return 0; 
}
