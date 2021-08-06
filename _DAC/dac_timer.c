///////////////////////////////////////////////////////////////////  
/*                    DAC Demo for STM32F4                       */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F4 (ARM Cortex M4)                    */
/*  Hardware:         STM32 F4VE  Eval-Board  (F407 NCU)         */
/*                    with CS43L22 DAC Module                    */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      AUG 2021                                   */
///////////////////////////////////////////////////////////////////
//This demo generates sine wave with frequency about 1kHz by using
//DAC2 of STM32F07 MCU and TIMER2 for accurate sine wave timing

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

///////////////////////////
//Function declarations
///////////////////////////
int main(void);

//Sine wave table 0 <= Phi < 360°
uint16_t wave[360] = {2048, 2084, 2119, 2155, 2191, 2226, 2262, 2298, 2333, 2368, 2404, 2439, 
	               2474, 2509, 2543, 2578, 2613, 2647, 2681, 2715, 2748, 2782, 2815, 2848, 
	               2881, 2914, 2946, 2978, 3009, 3041, 3072, 3103, 3133, 3163, 3193, 3223, 
	               3252, 3281, 3309, 3337, 3364, 3392, 3418, 3445, 3471, 3496, 3521, 3546, 
	               3570, 3594, 3617, 3640, 3662, 3684, 3705, 3726, 3746, 3766, 3785, 3803, 
	               3822, 3839, 3856, 3873, 3889, 3904, 3919, 3933, 3947, 3960, 3972, 3984, 
	               3996, 4007, 4017, 4026, 4035, 4044, 4051, 4058, 4065, 4071, 4076, 4081, 
	               4085, 4088, 4091, 4093, 4094, 4094, 4095, 4095, 4094, 4093, 4091, 4088, 
	               4085, 4081, 4076, 4071, 4065, 4058, 4051, 4044, 4035, 4026, 4017, 4007, 
	               3996, 3984, 3972, 3960, 3947, 3933, 3919, 3904, 3889, 3873, 3856, 3839, 
	               3822, 3803, 3785, 3766, 3746, 3726, 3705, 3684, 3662, 3640, 3617, 3594, 
	               3570, 3546, 3521, 3496, 3471, 3445, 3418, 3392, 3364, 3337, 3309, 3281, 
	               3252, 3223, 3193, 3163, 3133, 3103, 3072, 3041, 3009, 2978, 2946, 2914, 
	               2881, 2848, 2815, 2782, 2748, 2715, 2681, 2647, 2613, 2578, 2543, 2509, 
	               2474, 2439, 2404, 2368, 2333, 2298, 2262, 2226, 2191, 2155, 2119, 2084, 
	               2048, 2012, 1977, 1941, 1905, 1870, 1834, 1798, 1763, 1728, 1692, 1657, 
	               1622, 1587, 1553, 1518, 1483, 1449, 1415, 1381, 1348, 1314, 1281, 1248, 
	               1215, 1182, 1150, 1118, 1087, 1055, 1024,  993,  963,  933,  903,  873, 
	                844,  815,  787,  759,  732,  704,  678,  651,  625,  600,  575,  550,
	                526,  502,  479,  456,  434,  412,  391,  370,  350,  330,  311,  293,  
	                274,  257,  240,  223,  207,  192,  177,  163,  149,  136,  124,  112,  
	                100,   89,   79,   70,   61,   52,   45,   38,   31,   25,   20,   15, 
	                 11,    8,    5,    3,    1,    0,    0,    0,    1,    3,    5,    8, 
	                 11,   15,   20,   25,   31,   38,   45,   52,   61,   70,   79,   89, 
	                100,  112,  124,  136,  149,  163,  177,  192,  207,  223,  240,  257, 
	                274,  293,  311,  330,  350,  370,  391,  412,  434,  456,  479,  502,  
	                526,  550,  575,  600,  625,  651,  678,  704,  732,  759,  787,  815,  
	                844,  873,  903,  933,  963,  993, 1024, 1055, 1087, 1118, 1150, 1182, 
	               1215, 1248, 1281, 1314, 1348, 1381, 1415, 1449, 1483, 1518, 1553, 1587, 
	               1622, 1657, 1692, 1728, 1763, 1798, 1834, 1870, 1905, 1941, 1977, 2012};
	               
//////////////////////////////////////////////////
//Main code starts from here
//////////////////////////////////////////////////
int main(void)
{
	int32_t t = 0;
    uint32_t ti = 150; //Delay time in Microseconds
    
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
    RCC->PLLCFGR |= 168 << RCC_PLLCFGR_PLLN_Pos; //PLL-N: f.VCO.out = f.VCO.in * 168 = 336MHz
    
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
    ////////////////////////////////////////////
    
    
    RCC->AHB1ENR |= (1 << 0);         //Enable GPIOA clock, bit 0 on AHB1ENR
    RCC->APB1ENR |= (1 << 29);        //Enable DAC clock, Bit29 on APB1ENR
    
    GPIOA->MODER &= ~(3 << (5 << 1)); //Reset bits
    
    GPIOA->MODER |= (3 << (5 << 1));  //Set PA5 to analog mode
    GPIOA->PUPDR &= ~(3 << (5 << 1)); //No pullup/no pulldown

    DAC->CR |= (1 << 16);              //Enable DAC channel 2
    DAC->CR &= ~(1 << 17);             //Enable DAC Ch2 output buffer
    DAC->CR |= (1 << 18);              //Enable trigger
    DAC->CR |= (7 << 19);              //Choose sw trigger as source (0b111)
    
	//////////////////////////////////////////
    // Setup TIMER2
    //////////////////////////////////////////
    RCC->APB1ENR |= (1 << 0);         //Enable TIM2 clock (Bit0)
 	TIM2->CR1 = 0;                    //Reset register
	TIM2->SR = 0;                     //Clear the update event flag
	TIM2->PSC = 168;                  //Set prescaler to 1MHz
    TIM2->CR1 |= (1 << 0);            //Start the timer counter 
    
    while(1)
    {   
		for(t = 0; t < 360; t++)
		{
            DAC->DHR12R2 = wave[t];
            TIM2->EGR &= ~(1 << 0);           //Timer reset
            TIM2->CNT = 0x00;                 //Counter register reset
	        while(TIM2->CNT < ti);
	        DAC->SWTRIGR |= (1 << 1);         //Trigger DAC Ch2
        }
    } 
}
