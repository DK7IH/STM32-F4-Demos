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
//Hint: This is an extended example for educational purposes
//showing how to drive IL9341 with onboard SPI from an STM32F4 MCU
//generate full text display usage of this coloered LCD. 


#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <string.h>
#include <stdlib.h>


//define ports for RST and DC line of IL9341 LCD connected to GPIOA
#define LCD_DC  2
#define LCD_RES 3
#define LCDPORT GPIOA->ODR

#define LCD_CMD   0
#define LCD_DATA  1

#define LCD_WIDTH   320
#define LCD_HEIGHT  240	

#define WHITE       0xFFFF
#define SILVER1     0xC618
#define SILVER2     0xA510
#define BLACK       0x0000
#define GRAY        0x8410
#define LIGHT_GRAY  0xC618
#define LIGHT_GREEN 0x07E0
#define LIGHT_RED   0xF800
#define LIGHT_BLUE  0x03FF
#define RED         0xF800
#define MAROON1     0x8000
#define MAROON2     0x7800
#define FUCHSIA     0xF81F		
#define PURPLE1     0x8010
#define PURPLE2     0x780F
#define LIME        0x07E0
#define GREEN       0x0400
#define YELLOW      0xFFE0
#define OLIVE1      0x8400
#define OLIVE2      0x7BE0
#define BLUE        0x001F
#define NAVY1       0x0010
#define NAVY2       0x000F
#define AQUA        0x07FF
#define TEAL        0x0410
#define DARK_BLUE   0x0002
#define MAGENTA     0xF81F
#define CYAN        0x07FF
#define DARK_CYAN   0x03EF
#define ORANGE      0xFCA0
#define BROWN       0x8200
#define VIOLET      0x9199
#define PINK        0xF97F
#define GOLD        0xA508

#define ROW1 220
#define ROW2 200
#define ROW3 180
#define ROW4 160
#define ROW5 140
#define ROW6 120
#define ROW7 100
#define ROW8  80
#define ROW9  60
#define ROW10 40
#define ROW11 20

//Font
#define FONTWIDTH 12 
#define FONTHEIGHT 16
//Font 12x16 vert. MSB 
const char xchar[][24]={
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x00
{0x00,0x00,0x03,0xF0,0x0C,0x0C,0x10,0x02,0x11,0x32,0x22,0x31,0x22,0x01,0x22,0x31,0x11,0x32,0x10,0x02,0x0C,0x0C,0x03,0xF0},	// 0x01
{0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1F,0xFE,0x1E,0xCE,0x3D,0xCF,0x3D,0xFF,0x3D,0xCF,0x1E,0xCE,0x1F,0xFE,0x0F,0xFC,0x03,0xF0},	// 0x02
{0x00,0x00,0x00,0x00,0x00,0xF0,0x01,0xF8,0x03,0xF8,0x07,0xF0,0x0F,0xE0,0x07,0xF0,0x03,0xF8,0x01,0xF8,0x00,0xF0,0x00,0x00},	// 0x03
{0x00,0x00,0x00,0x00,0x00,0x80,0x01,0xC0,0x03,0xE0,0x07,0xF0,0x0F,0xF8,0x07,0xF0,0x03,0xE0,0x01,0xC0,0x00,0x80,0x00,0x00},	// 0x04
{0x00,0x00,0x03,0x80,0x07,0xC0,0x07,0xC0,0x13,0xB8,0x1B,0xFC,0x1F,0xFC,0x1B,0xFC,0x13,0xB8,0x07,0xC0,0x07,0xC0,0x03,0x80},	// 0x05
{0x00,0x00,0x00,0x00,0x03,0x80,0x07,0xC0,0x17,0xE0,0x1B,0xF0,0x1F,0xFC,0x1B,0xF0,0x17,0xE0,0x07,0xC0,0x03,0x80,0x00,0x00},	// 0x06
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x07
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x08
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x09
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x0A
{0x00,0x00,0x03,0x80,0x07,0xC0,0x0C,0x60,0x08,0x20,0x08,0x20,0x0C,0x60,0x07,0xC8,0x03,0xA8,0x00,0x18,0x00,0x78,0x00,0x00},	// 0x0B
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x70,0x08,0xF8,0x09,0x8C,0x3F,0x04,0x3F,0x04,0x09,0x8C,0x08,0xF8,0x00,0x70,0x00,0x00},	// 0x0C
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x0D
{0x00,0x00,0x06,0x00,0x0F,0x00,0x0F,0x00,0x07,0xFF,0x00,0x33,0x30,0x66,0x78,0xCC,0x79,0x98,0x3F,0xF0,0x00,0x00,0x00,0x00},	// 0x0E
{0x00,0x00,0x00,0x80,0x09,0xC8,0x07,0xF0,0x06,0x30,0x0C,0x18,0x3C,0x1E,0x0C,0x18,0x06,0x30,0x07,0xF0,0x09,0xC8,0x00,0x80},	// 0x0F
{0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xFC,0x0F,0xF8,0x07,0xF0,0x03,0xE0,0x01,0xC0,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x10
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x01,0xC0,0x03,0xE0,0x07,0xF0,0x0F,0xF8,0x1F,0xFC,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x11
{0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x10,0x0C,0x18,0x1C,0x1C,0x3F,0xFE,0x1C,0x1C,0x0C,0x18,0x04,0x10,0x00,0x00,0x00,0x00},	// 0x12
{0x00,0x00,0x00,0x00,0x00,0x00,0x37,0xFE,0x37,0xFE,0x00,0x00,0x00,0x00,0x37,0xFE,0x37,0xFE,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x13
{0x00,0x00,0x00,0x38,0x00,0x7C,0x00,0xC6,0x00,0x82,0x3F,0xFE,0x3F,0xFE,0x00,0x02,0x3F,0xFE,0x3F,0xFE,0x00,0x02,0x00,0x00},	// 0x14
{0x00,0x00,0x00,0x00,0x08,0xDC,0x19,0xFE,0x11,0x22,0x11,0x22,0x11,0x22,0x11,0x22,0x1F,0xE6,0x0E,0xC4,0x00,0x00,0x00,0x00},	// 0x15
{0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x00,0x38,0x00,0x38,0x00,0x38,0x00,0x38,0x00,0x38,0x00,0x38,0x00,0x38,0x00,0x00,0x00},	// 0x16
{0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x08,0x4C,0x0C,0x5C,0x0E,0x7F,0xFF,0x5C,0x0E,0x4C,0x0C,0x44,0x08,0x00,0x00,0x00,0x00},	// 0x17
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x18,0x00,0x1C,0x3F,0xFE,0x00,0x1C,0x00,0x18,0x00,0x10,0x00,0x00,0x00,0x00},	// 0x18
{0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x0C,0x00,0x1C,0x00,0x3F,0xFE,0x1C,0x00,0x0C,0x00,0x04,0x00,0x00,0x00,0x00,0x00},	// 0x19
{0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x07,0xF0,0x03,0xE0,0x01,0xC0,0x00,0x80,0x00,0x00},	// 0x1A
{0x00,0x00,0x00,0x00,0x00,0x80,0x01,0xC0,0x03,0xE0,0x07,0xF0,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x00},	// 0x1B
{0x00,0x00,0x3F,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x00,0x00},	// 0x1C
{0x00,0x00,0x00,0x80,0x01,0xC0,0x03,0xE0,0x07,0xF0,0x00,0x80,0x00,0x80,0x00,0x80,0x07,0xF0,0x03,0xE0,0x01,0xC0,0x00,0x80},	// 0x1D
{0x00,0x00,0x04,0x00,0x06,0x00,0x07,0x00,0x07,0x80,0x07,0xC0,0x07,0xE0,0x07,0xC0,0x07,0x80,0x07,0x00,0x06,0x00,0x04,0x00},	// 0x1E
{0x00,0x00,0x00,0x20,0x00,0x60,0x00,0xE0,0x01,0xE0,0x03,0xE0,0x07,0xE0,0x03,0xE0,0x01,0xE0,0x00,0xE0,0x00,0x60,0x00,0x20},	// 0x1F
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x20
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7C,0x33,0xFF,0x33,0xFF,0x00,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x21
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,0x3C,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x22
{0x00,0x00,0x02,0x00,0x1E,0x10,0x1F,0x90,0x03,0xF0,0x02,0x7E,0x1E,0x1E,0x1F,0x90,0x03,0xF0,0x02,0x7E,0x00,0x1E,0x00,0x10},	// 0x23
{0x00,0x00,0x00,0x00,0x04,0x78,0x0C,0xFC,0x0C,0xCC,0x3F,0xFF,0x3F,0xFF,0x0C,0xCC,0x0F,0xCC,0x07,0x88,0x00,0x00,0x00,0x00},	// 0x24
{0x00,0x00,0x30,0x00,0x38,0x38,0x1C,0x38,0x0E,0x38,0x07,0x00,0x03,0x80,0x01,0xC0,0x38,0xE0,0x38,0x70,0x38,0x38,0x00,0x1C},	// 0x25
{0x00,0x00,0x00,0x00,0x1F,0x00,0x3F,0xB8,0x31,0xFC,0x21,0xC6,0x37,0xE2,0x1E,0x3E,0x1C,0x1C,0x36,0x00,0x22,0x00,0x00,0x00},	// 0x26
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x27,0x00,0x3F,0x00,0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x27
{0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1F,0xFE,0x38,0x07,0x20,0x01,0x20,0x01,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x28
{0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x01,0x20,0x01,0x38,0x07,0x1F,0xFE,0x0F,0xFC,0x03,0xF0,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x29
{0x00,0x00,0x00,0x00,0x0C,0x98,0x0E,0xB8,0x03,0xE0,0x0F,0xF8,0x0F,0xF8,0x03,0xE0,0x0E,0xB8,0x0C,0x98,0x00,0x00,0x00,0x00},	// 0x2A
{0x00,0x00,0x00,0x00,0x01,0x80,0x01,0x80,0x01,0x80,0x0F,0xF0,0x0F,0xF0,0x01,0x80,0x01,0x80,0x01,0x80,0x00,0x00,0x00,0x00},	// 0x2B
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB8,0x00,0xF8,0x00,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x2C
{0x00,0x00,0x00,0x00,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x00,0x00,0x00,0x00},	// 0x2D
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x00,0x38,0x00,0x38,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x2E
{0x00,0x00,0x18,0x00,0x1C,0x00,0x0E,0x00,0x07,0x00,0x03,0x80,0x01,0xC0,0x00,0xE0,0x00,0x70,0x00,0x38,0x00,0x1C,0x00,0x0E},	// 0x2F
{0x00,0x00,0x07,0xF8,0x1F,0xFE,0x1E,0x06,0x33,0x03,0x31,0x83,0x30,0xC3,0x30,0x63,0x30,0x33,0x18,0x1E,0x1F,0xFE,0x07,0xF8},	// 0x30
{0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x0C,0x30,0x0C,0x30,0x0E,0x3F,0xFF,0x3F,0xFF,0x30,0x00,0x30,0x00,0x30,0x00,0x00,0x00},	// 0x31
{0x00,0x00,0x30,0x1C,0x38,0x1E,0x3C,0x07,0x3E,0x03,0x37,0x03,0x33,0x83,0x31,0xC3,0x30,0xE3,0x30,0x77,0x30,0x3E,0x30,0x1C},	// 0x32
{0x00,0x00,0x0C,0x0C,0x1C,0x0E,0x38,0x07,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x39,0xE7,0x1F,0x7E,0x0E,0x3C},	// 0x33
{0x00,0x00,0x03,0xC0,0x03,0xE0,0x03,0x70,0x03,0x38,0x03,0x1C,0x03,0x0E,0x03,0x07,0x3F,0xFF,0x3F,0xFF,0x03,0x00,0x03,0x00},	// 0x34
{0x00,0x00,0x0C,0x3F,0x1C,0x7F,0x38,0x63,0x30,0x63,0x30,0x63,0x30,0x63,0x30,0x63,0x30,0x63,0x38,0xE3,0x1F,0xC3,0x0F,0x83},	// 0x35
{0x00,0x00,0x0F,0xC0,0x1F,0xF0,0x39,0xF8,0x30,0xDC,0x30,0xCE,0x30,0xC7,0x30,0xC3,0x30,0xC3,0x39,0xC3,0x1F,0x80,0x0F,0x00},	// 0x36
{0x00,0x00,0x00,0x03,0x00,0x03,0x00,0x03,0x30,0x03,0x3C,0x03,0x0F,0x03,0x03,0xC3,0x00,0xF3,0x00,0x3F,0x00,0x0F,0x00,0x03},	// 0x37
{0x00,0x00,0x0F,0x00,0x1F,0xBC,0x39,0xFE,0x30,0xE7,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xE7,0x39,0xFE,0x1F,0xBC,0x0F,0x00},	// 0x38
{0x00,0x00,0x00,0x3C,0x00,0x7E,0x30,0xE7,0x30,0xC3,0x30,0xC3,0x38,0xC3,0x1C,0xC3,0x0E,0xC3,0x07,0xE7,0x03,0xFE,0x00,0xFC},	// 0x39
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x3A
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9C,0x70,0xFC,0x70,0x7C,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x3B
{0x00,0x00,0x00,0x00,0x00,0xC0,0x01,0xE0,0x03,0xF0,0x07,0x38,0x0E,0x1C,0x1C,0x0E,0x38,0x07,0x30,0x03,0x00,0x00,0x00,0x00},	// 0x3C
{0x00,0x00,0x00,0x00,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x00,0x00},	// 0x3D
{0x00,0x00,0x00,0x00,0x30,0x03,0x38,0x07,0x1C,0x0E,0x0E,0x1C,0x07,0x38,0x03,0xF0,0x01,0xE0,0x00,0xC0,0x00,0x00,0x00,0x00},	// 0x3E
{0x00,0x00,0x00,0x1C,0x00,0x1E,0x00,0x07,0x00,0x03,0x37,0x83,0x37,0xC3,0x00,0xE3,0x00,0x77,0x00,0x3E,0x00,0x1C,0x00,0x00},	// 0x3F
{0x00,0x00,0x0F,0xF8,0x1F,0xFE,0x18,0x07,0x33,0xF3,0x37,0xFB,0x36,0x1B,0x37,0xFB,0x37,0xFB,0x36,0x07,0x03,0xFE,0x01,0xF8},	// 0x40
{0x00,0x00,0x38,0x00,0x3F,0x00,0x07,0xE0,0x06,0xFC,0x06,0x1F,0x06,0x1F,0x06,0xFC,0x07,0xE0,0x3F,0x00,0x38,0x00,0x00,0x00},	// 0x41
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xE7,0x39,0xFE,0x1F,0xBC,0x0F,0x00,0x00,0x00},	// 0x42
{0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1C,0x0E,0x38,0x07,0x30,0x03,0x30,0x03,0x30,0x03,0x38,0x07,0x1C,0x0E,0x0C,0x0C,0x00,0x00},	// 0x43
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0x03,0x30,0x03,0x30,0x03,0x30,0x03,0x38,0x07,0x1C,0x0E,0x0F,0xFC,0x03,0xF0,0x00,0x00},	// 0x44
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0x03,0x30,0x03,0x00,0x00},	// 0x45
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0xC3,0x00,0x03,0x00,0x03,0x00,0x00},	// 0x46
{0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1C,0x0E,0x38,0x07,0x30,0x03,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x3F,0xC7,0x3F,0xC6,0x00,0x00},	// 0x47
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x3F,0xFF,0x3F,0xFF,0x00,0x00},	// 0x48
{0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x03,0x30,0x03,0x3F,0xFF,0x3F,0xFF,0x30,0x03,0x30,0x03,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x49
{0x00,0x00,0x0E,0x00,0x1E,0x00,0x38,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x38,0x00,0x1F,0xFF,0x07,0xFF,0x00,0x00},	// 0x4A
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0xC0,0x01,0xE0,0x03,0xF0,0x07,0x38,0x0E,0x1C,0x1C,0x0E,0x38,0x07,0x30,0x03,0x00,0x00},	// 0x4B
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x00,0x00},	// 0x4C
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0x1E,0x00,0x78,0x01,0xE0,0x01,0xE0,0x00,0x78,0x00,0x1E,0x3F,0xFF,0x3F,0xFF,0x00,0x00},	// 0x4D
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0x0E,0x00,0x38,0x00,0xF0,0x03,0xC0,0x07,0x00,0x1C,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0x00},	// 0x4E
{0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1C,0x0E,0x38,0x07,0x30,0x03,0x30,0x03,0x38,0x07,0x1C,0x0E,0x0F,0xFC,0x03,0xF0,0x00,0x00},	// 0x4F
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x01,0x83,0x01,0x83,0x01,0x83,0x01,0x83,0x01,0x83,0x01,0xC7,0x00,0xFE,0x00,0x7C,0x00,0x00},	// 0x50
{0x00,0x00,0x03,0xF0,0x0F,0xFC,0x1C,0x0E,0x38,0x07,0x30,0x03,0x36,0x03,0x3E,0x07,0x1C,0x0E,0x3F,0xFC,0x33,0xF0,0x00,0x00},	// 0x51
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x01,0x83,0x01,0x83,0x03,0x83,0x07,0x83,0x0F,0x83,0x1D,0xC7,0x38,0xFE,0x30,0x7C,0x00,0x00},	// 0x52
{0x00,0x00,0x0C,0x3C,0x1C,0x7E,0x38,0xE7,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x30,0xC3,0x39,0xC7,0x1F,0x8E,0x0F,0x0C,0x00,0x00},	// 0x53
{0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x03,0x00,0x03,0x3F,0xFF,0x3F,0xFF,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x00,0x00,0x00},	// 0x54
{0x00,0x00,0x07,0xFF,0x1F,0xFF,0x38,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x38,0x00,0x1F,0xFF,0x07,0xFF,0x00,0x00},	// 0x55
{0x00,0x00,0x00,0x07,0x00,0x3F,0x01,0xF8,0x0F,0xC0,0x3E,0x00,0x3E,0x00,0x0F,0xC0,0x01,0xF8,0x00,0x3F,0x00,0x07,0x00,0x00},	// 0x56
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x1C,0x00,0x06,0x00,0x03,0x80,0x03,0x80,0x06,0x00,0x1C,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0x00},	// 0x57
{0x00,0x00,0x30,0x03,0x3C,0x0F,0x0E,0x1C,0x03,0x30,0x01,0xE0,0x01,0xE0,0x03,0x30,0x0E,0x1C,0x3C,0x0F,0x30,0x03,0x00,0x00},	// 0x58
{0x00,0x00,0x00,0x03,0x00,0x0F,0x00,0x3C,0x00,0xF0,0x3F,0xC0,0x3F,0xC0,0x00,0xF0,0x00,0x3C,0x00,0x0F,0x00,0x03,0x00,0x00},	// 0x59
{0x00,0x00,0x30,0x03,0x3C,0x03,0x3E,0x03,0x33,0x03,0x31,0xC3,0x30,0xE3,0x30,0x33,0x30,0x1F,0x30,0x0F,0x30,0x03,0x00,0x00},	// 0x5A
{0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0x03,0x30,0x03,0x30,0x03,0x30,0x03,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x5B
{0x00,0x00,0x00,0x0E,0x00,0x1C,0x00,0x38,0x00,0x70,0x00,0xE0,0x01,0xC0,0x03,0x80,0x07,0x00,0x0E,0x00,0x1C,0x00,0x18,0x00},	// 0x5C
{0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x03,0x30,0x03,0x30,0x03,0x30,0x03,0x3F,0xFF,0x3F,0xFF,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x5D
{0x00,0x00,0x00,0x60,0x00,0x70,0x00,0x38,0x00,0x1C,0x00,0x0E,0x00,0x07,0x00,0x0E,0x00,0x1C,0x00,0x38,0x00,0x70,0x00,0x60},	// 0x5E
{0x00,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xC0,0x00},	// 0x5F
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x00,0x7E,0x00,0x4E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x60
{0x00,0x00,0x1C,0x00,0x3E,0x40,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x3F,0xE0,0x3F,0xC0,0x00,0x00},	// 0x61
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x30,0xC0,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x60,0x38,0xE0,0x1F,0xC0,0x0F,0x80,0x00,0x00},	// 0x62
{0x00,0x00,0x0F,0x80,0x1F,0xC0,0x38,0xE0,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x60,0x18,0xC0,0x08,0x80,0x00,0x00},	// 0x63
{0x00,0x00,0x0F,0x80,0x1F,0xC0,0x38,0xE0,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0xE0,0x30,0xC0,0x3F,0xFF,0x3F,0xFF,0x00,0x00},	// 0x64
{0x00,0x00,0x0F,0x80,0x1F,0xC0,0x3B,0xE0,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x13,0xC0,0x01,0x80,0x00,0x00},	// 0x65
{0x00,0x00,0x00,0xC0,0x00,0xC0,0x3F,0xFC,0x3F,0xFE,0x00,0xC7,0x00,0xC3,0x00,0xC3,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x66
{0x00,0x00,0x03,0x80,0xC7,0xC0,0xCE,0xE0,0xCC,0x60,0xCC,0x60,0xCC,0x60,0xCC,0x60,0xE6,0x60,0x7F,0xE0,0x3F,0xE0,0x00,0x00},	// 0x67
{0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x00,0xC0,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xE0,0x3F,0xC0,0x3F,0x80,0x00,0x00,0x00,0x00},	// 0x68
{0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x00,0x30,0x60,0x3F,0xEC,0x3F,0xEC,0x30,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x69
{0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x00,0xE0,0x00,0xC0,0x00,0xC0,0x60,0xFF,0xEC,0x7F,0xEC,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x6A
{0x00,0x00,0x00,0x00,0x3F,0xFF,0x3F,0xFF,0x03,0x00,0x07,0x80,0x0F,0xC0,0x1C,0xE0,0x38,0x60,0x30,0x00,0x00,0x00,0x00,0x00},	// 0x6B
{0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x00,0x30,0x03,0x3F,0xFF,0x3F,0xFF,0x30,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x6C
{0x00,0x00,0x3F,0xE0,0x3F,0xC0,0x00,0xE0,0x00,0xE0,0x3F,0xC0,0x3F,0xC0,0x00,0xE0,0x00,0xE0,0x3F,0xC0,0x3F,0x80,0x00,0x00},	// 0x6D
{0x00,0x00,0x00,0x00,0x3F,0xE0,0x3F,0xE0,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xE0,0x3F,0xC0,0x3F,0x80,0x00,0x00},	// 0x6E
{0x00,0x00,0x0F,0x80,0x1F,0xC0,0x38,0xE0,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x60,0x38,0xE0,0x1F,0xC0,0x0F,0x80,0x00,0x00},	// 0x6F
{0x00,0x00,0xFF,0xE0,0xFF,0xE0,0x0C,0x60,0x18,0x60,0x18,0x60,0x18,0x60,0x18,0x60,0x1C,0xE0,0x0F,0xC0,0x07,0x80,0x00,0x00},	// 0x70
{0x00,0x00,0x07,0x80,0x0F,0xC0,0x1C,0xE0,0x18,0x60,0x18,0x60,0x18,0x60,0x18,0x60,0x0C,0x60,0xFF,0xE0,0xFF,0xE0,0x00,0x00},	// 0x71
{0x00,0x00,0x00,0x00,0x3F,0xE0,0x3F,0xE0,0x00,0xC0,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xE0,0x00,0xC0,0x00,0x00},	// 0x72
{0x00,0x00,0x11,0xC0,0x33,0xE0,0x33,0x60,0x33,0x60,0x33,0x60,0x33,0x60,0x3F,0x60,0x1E,0x40,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x73
{0x00,0x00,0x00,0x60,0x00,0x60,0x1F,0xFE,0x3F,0xFE,0x30,0x60,0x30,0x60,0x30,0x60,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x74
{0x00,0x00,0x0F,0xE0,0x1F,0xE0,0x38,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x18,0x00,0x3F,0xE0,0x3F,0xE0,0x00,0x00},	// 0x75
{0x00,0x00,0x00,0x60,0x01,0xE0,0x07,0x80,0x1E,0x00,0x38,0x00,0x38,0x00,0x1E,0x00,0x07,0x80,0x01,0xE0,0x00,0x60,0x00,0x00},	// 0x76
{0x00,0x00,0x07,0xE0,0x1F,0xE0,0x38,0x00,0x1C,0x00,0x0F,0xE0,0x0F,0xE0,0x1C,0x00,0x38,0x00,0x1F,0xE0,0x07,0xE0,0x00,0x00},	// 0x77
{0x00,0x00,0x30,0x60,0x38,0xE0,0x1D,0xC0,0x0F,0x80,0x07,0x00,0x0F,0x80,0x1D,0xC0,0x38,0xE0,0x30,0x60,0x00,0x00,0x00,0x00},	// 0x78
{0x00,0x00,0x00,0x00,0x00,0x60,0x81,0xE0,0xE7,0x80,0x7E,0x00,0x1E,0x00,0x07,0x80,0x01,0xE0,0x00,0x60,0x00,0x00,0x00,0x00},	// 0x79
{0x00,0x00,0x30,0x60,0x38,0x60,0x3C,0x60,0x36,0x60,0x33,0x60,0x31,0xE0,0x30,0xE0,0x30,0x60,0x30,0x20,0x00,0x00,0x00,0x00},	// 0x7A
{0x00,0x00,0x00,0x00,0x00,0x80,0x01,0xC0,0x1F,0xFC,0x3F,0x7E,0x70,0x07,0x60,0x03,0x60,0x03,0x60,0x03,0x00,0x00,0x00,0x00},	// 0x7B
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0xBF,0x3F,0xBF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x7C
{0x00,0x00,0x00,0x00,0x60,0x03,0x60,0x03,0x60,0x03,0x70,0x07,0x3F,0x7E,0x1F,0xFC,0x01,0xC0,0x00,0x80,0x00,0x00,0x00,0x00},	// 0x7D
{0x00,0x00,0x00,0x10,0x00,0x18,0x00,0x0C,0x00,0x04,0x00,0x0C,0x00,0x18,0x00,0x10,0x00,0x18,0x00,0x0C,0x00,0x04,0x00,0x00},	// 0x7E
{0x00,0x00,0x0F,0x00,0x0F,0x80,0x0C,0xC0,0x0C,0x60,0x0C,0x30,0x0C,0x30,0x0C,0x60,0x0C,0xC0,0x0F,0x80,0x0F,0x00,0x00,0x00},	// 0x7F
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x78,0x00,0xFC,0x00,0xCC,0x00,0xCC,0x00,0xFC,0x00,0x78,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x80 °-sign
};


//Lcd help data
//Display buffer
char *oldbuf = (char*)"         ";
int bcolor = DARK_BLUE; //Standard Background Color

//ILI9341 LCD Basic Functions
void lcd_write(uint8_t, uint8_t);
void lcd_init(void);
void lcd_set_xy(int, int);
void lcd_cls(int);
void lcd_draw_pixel(int);
void lcd_putchar(int, int, int, int, int, int);
void lcd_putstring(int, int, char*, int, int, int);

//Extended LCD functions
void show_frequency(long, int, int);
void show_mem_number(int);
void show_sideband(int, int);
void show_voltage(int);
void show_temp(int);
void show_vfo(int, int);
void show_split(int, int);
void show_scan_status(int, int);
void show_tone(int, int);
void show_agc(int, int);
void draw_meter_scale(int, int);
void reset_smax(void);
void smeter(int, int);
void show_msg(char*, int);

//STRING FUNCTIONS
int int2asc(long, int, char*, int);
int strlen(char *s);

//MISC
int main(void);

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


//Write character from font set to destination on screen
void lcd_putchar(int x, int y, int c, int size, int fcolor, int bcolor)
{
    int x0;
    int t0, t1, t2, t3, u;
       
    x0 = x;
    for(t0 = 0; t0 < FONTWIDTH * 2; t0 += 2)
    { 
		for(t1 = 0; t1 < size; t1++)
		{
		    u = xchar[c][t0 + 1] + (xchar[c][t0] << 8);
		    lcd_set_xy(x0, y);
		    for(t2 = 16; t2 >= 0; t2--)
		    {
			    if(u & (1 << t2))
			    {
				    for(t3 = 0; t3 < size; t3++)
				    {
		                lcd_draw_pixel(fcolor);
		            }
		        }    
		        else
		        {
		            for(t3 = 0; t3 < size; t3++)
				    {
		                lcd_draw_pixel(bcolor);
		            }
		        }
		    }
		    x0++;
		}    
	}	
}	

//Print String to LCD
void lcd_putstring(int x, int y, char *text, int size, int fc, int bc)
{
	int t1 = 0, x0;
	
	x0 = x;
	while(text[t1])
	{
		lcd_putchar(x0, y, text[t1], size, fc, bc);
		x0 += (size * FONTWIDTH);
		t1++;
	}
}		

//Convert a number to a string and print it
//col, row: Coordinates, Num: int or long to be displayed
//dec: Set position of decimal separator
//
//inv: Set to 1 if inverted charactor is required
void lcd_putnumber(int x, int y, long num, int dec, int lsize, int fc, int bc)
{
    char *s = (char*)malloc(16);
	if(s != NULL)
	{
	    int2asc(num, dec, s, 16);
	    lcd_putstring(x, y, s, lsize, fc, bc);
	    free(s);
	}	
	else
	{
		lcd_putstring(x, y, (char*)"Error", lsize, fc, bc);
	}	
}


//////////////////////
 // STRING FUNCTIONS //
//////////////////////
//INT 2 ASC
int int2asc(long num, int dec, char *buf, int buflen)
{
    int i, c, xp = 0, neg = 0;
    long n, dd = 1E09;

    if(!num)
	{
	    *buf++ = '0';
		*buf = 0;
		return 1;
	}	
		
    if(num < 0)
    {
     	neg = 1;
	    n = num * -1;
    }
    else
    {
	    n = num;
    }

    //Fill buffer with \0
    for(i = 0; i < buflen; i++)
    {
	    *(buf + i) = 0;
    }

    c = 9; //Max. number of displayable digits
    while(dd)
    {
	    i = n / dd;
	    n = n - i * dd;
	
	    *(buf + 9 - c + xp) = i + 48;
	    dd /= 10;
	    if(c == dec && dec)
	    {
	        *(buf + 9 - c + ++xp) = '.';
	    }
	    c--;
    }

    //Search for 1st char different from '0'
    i = 0;
    while(*(buf + i) == 48)
    {
	    *(buf + i++) = 32;
    }

    //Add minus-sign if neccessary
    if(neg)
    {
	    *(buf + --i) = '-';
    }

    //Eleminate leading spaces
    c = 0;
    while(*(buf + i))
    {
	    *(buf + c++) = *(buf + i++);
    }
    *(buf + c) = 0;
	
	return c;
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
    
    RCC->AHB1ENR |= (1 << 0);                   //GPIOA clock enable
    RCC->AHB1ENR |= (1 << 1);                   //GPIOB clock enable
    RCC->APB1ENR |= (1 << 14);                  //Enable SPI2 clock, bit 14 in APB1ENR

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
    lcd_cls(bcolor);
    lcd_putstring(0, 120, (char*) "DK7IH STM32 SPI demo with", 1, YELLOW, bcolor);
    lcd_putstring(0, 100, (char*) "IL9341 (C) 2021", 1, WHITE, bcolor);
    lcd_putstring(0, 80, (char*) "73 de DK7IH", 2, LIGHTGREEN, bcolor);
    
    while(1)
    {
		//Perform CLS just for fun :-)
		GPIOC->ODR &= ~(1 << 13);
        
        GPIOC->ODR |= (1 << 13); //Blink LED
        delay_ms(1000);
           
    }

    return 0;
}
