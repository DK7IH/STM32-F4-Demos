///////////////////////////////////////////////////////////////////  
//                    PWM demo for STM32F4                       //
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         STMf407 discovery board                    */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier (DK7IH)                        */
/*  Last change:      JUL 2021                                   */
///////////////////////////////////////////////////////////////////
//Purpose: 256 bytes of data are transferred between 2 memory 
//locations by DMA access. LEDS are showing the bit patterns
//of transferred data

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <stdlib.h>

#define BUF_SIZE 256

// Create a pointer at source and destination addresses
uint8_t *src_addr = (uint8_t*) 0x2000A000;
uint8_t *dst_addr = (uint8_t*) 0x2000A400;

int main(void);

int main(void)
{
	uint32_t t0, t1;
	
    //////////////////////////////////////////////
    // Set SystemClock to 50 MHz with 16 MHz HSI
    //////////////////////////////////////////////
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;        //2 wait states
    RCC->CR |= RCC_CR_HSION;                    //Activate internal clock (HSI: 16 MHz)
    while ((RCC->CR & RCC_CR_HSIRDY) == 0);     //Wait until HSI is ready
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;
    RCC->PLLCFGR |= 4 << RCC_PLLCFGR_PLLQ_Pos;  //PLL-Q: /4
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;     //PLL source is HSI
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;          //PLL-P: /2: f.PLL = f.VCO.out / 2 = 50MHz
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
    RCC->PLLCFGR |= 50 << RCC_PLLCFGR_PLLN_Pos; //PLL-N: x50: f.VCO.out = 50 x f.VCO.in = 100MHz
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
    RCC->PLLCFGR |= 8 << RCC_PLLCFGR_PLLM_Pos;  //PLL-M: /8 -> f.VCO.in = f.HSI / 8 = 2 MHZ
    RCC->CR |= RCC_CR_PLLON;                    //Activate PLL (Output: 50 MHz)
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);     //Wait until PLL is ready
    
    //Division by 2 of clk signal       
    RCC->CFGR |= RCC_CFGR_HPRE_DIV2             //AHB divider:  /2 =25MHz
              | RCC_CFGR_PPRE1_DIV2             //APB1 divider: /2 =25MHz
              | RCC_CFGR_PPRE2_DIV2;            //APB2 divider: /2 =25MHz
               
    RCC->CFGR |= RCC_CFGR_SW_PLL;               //Switching to PLL clock source
    
    //Allocate memory for buffer data
	src_addr = (uint8_t*)malloc(BUF_SIZE);
	dst_addr = (uint8_t*)malloc(BUF_SIZE);
	
    //////////////////////////////////
    //Setup LEDs - GPIOD 12,13,14,15
    //////////////////////////////////
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~(0xFFU << 24);
    GPIOD->MODER |= (0x55 << 24);
    GPIOD->ODR    = (0xF << 12);
    
    for(t1 = 0; t1 < 100000; t1++); //Wait

    //Fill src_addr with numbers
    //Zero out dst_addr
    for (int t0 = 0; t0 < BUF_SIZE; t0++)
    {
        src_addr[t0] = (uint8_t)t0;
        dst_addr[t0] = 0;
    }
    
    //////////////////////////////////
    //Setup DMA
    //////////////////////////////////
    //Enable DMA2 clock, bit 22 on AHB1ENR
    RCC->AHB1ENR |= (1 << 22);

    //Clear DMA Stream configuration register
    //Single transfer, M0 is target, single buffer mode, circular buffer disabled
    DMA2_Stream0->CR = 0;
    //Wait until dma is disabled
    while(DMA2_Stream0->CR & (1 << 0));

    //Set channel CHSEL: Bits27:25 to Channel0
    DMA2_Stream0->CR |= (0 << 25);

    //Set data transfer direction DIR: Bits7:6 memory-to-memory
    DMA2_Stream0->CR |= (2 << 6);

    //Set channel priority PL Bits17:16 to medium
    DMA2_Stream0->CR |= (0x1 << 16);

    //Increment memory MINC : Bit10
    DMA2_Stream0->CR |= (1 << 10);
    //Memory data size MSIZE : bits14:13 to byte
    DMA2_Stream0->CR |= (0 << 13);

    //Increment peripheral PINC : Bit9
    DMA2_Stream0->CR |= (1 << 9);
    // peripheral data size PSIZE : Bits12:11 to byte
    DMA2_Stream0->CR |= (0 << 11);
    
    while(1)
    {
		//Transfer data
        DMA2_Stream0->PAR = (uint32_t)src_addr;   //Source memory address
        DMA2_Stream0->M0AR = (uint32_t)dst_addr;  //Destination memory address
        DMA2_Stream0->NDTR = BUF_SIZE;            //Number of bytes to be transferred
        DMA2_Stream0->CR |= (1 << 0);             //Enable DMA Bit0
    
	    //Read destination after DMA transfer and display as binary pattern
        for (int t0 = 0; t0 < BUF_SIZE; t0++)
        {
            GPIOD->ODR = (uint16_t)(((dst_addr[t0] >> 4) & 0xFF) << 12); //MS-Nibble
            for(t1 = 0; t1 < 100000; t1++);
            GPIOD->ODR = (uint16_t)((dst_addr[t0] & 0xFF) << 12); //LS-Nibble
            for(t1 = 0; t1 < 100000; t1++);
        }
        
        for(t0 = 12; t0 <= 15; t0++)
        {
			GPIOD->ODR |= (1 << t0);
		}	
		for(t1 = 0; t1 < 1000000; t1++);
    }

    return 0;
}
