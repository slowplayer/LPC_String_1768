/******************************************************************************/
/* LCD.h: Graphic LCD function prototypes and defines                        */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2009 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#ifndef _LCD_H
#define _LCD_H
#include "sys.h"

extern u16 BACK_COLOR, POINT_COLOR ;
/*------------------------------------------------------------------------------
  Color coding
  LCD is coded:   15..11 red, 10..5 green, 4..0 blue  (unsigned short)  LCD_R5, LCD_G6, LCD_B5   
  original coding: 17..12 red, 11..6 green, 5..0 blue                    ORG_R6,  ORG_G6,  ORG_B6

  ORG_R1..5 = LCD_R0..4,  ORG_R0 = LCD_R4
  ORG_G0..5 = LCD_G0..5,
  ORG_B1..5 = LCD_B0..4,  ORG_B0 = LCD_B4
 *----------------------------------------------------------------------------*/
                            
/* LCD RGB color definitions                                                 */
#define Black           0x0000		/*   0,   0,   0 */
#define Navy            0x000F      /*   0,   0, 128 */
#define DarkGreen       0x03E0      /*   0, 128,   0 */
#define DarkCyan        0x03EF      /*   0, 128, 128 */
#define Maroon          0x7800      /* 128,   0,   0 */
#define Purple          0x780F      /* 128,   0, 128 */
#define Olive           0x7BE0      /* 128, 128,   0 */
#define LightGrey       0xC618      /* 192, 192, 192 */
#define DarkGrey        0x7BEF      /* 128, 128, 128 */
#define Blue            0x001F      /*   0,   0, 255 */
#define Green           0x07E0      /*   0, 255,   0 */
#define Cyan            0x07FF      /*   0, 255, 255 */
#define Red             0xF800      /* 255,   0,   0 */
#define Magenta         0xF81F      /* 255,   0, 255 */
#define Yellow          0xFFE0      /* 255, 255, 0   */
#define White           0xFFFF      /* 255, 255, 255 */
#define WHITE           0XFFFF
#define RED             0xF800
#define BLUE            0x001F


//9320/9325 LCD¼Ä´æÆ÷  
#define R0             0x00
#define R1             0x01
#define R2             0x02
#define R3             0x03
#define R4             0x04
#define R5             0x05
#define R6             0x06
#define R7             0x07
#define R8             0x08
#define R9             0x09
#define R10            0x0A
#define R12            0x0C
#define R13            0x0D
#define R14            0x0E
#define R15            0x0F
#define R16            0x10
#define R17            0x11
#define R18            0x12
#define R19            0x13
#define R20            0x14
#define R21            0x15
#define R22            0x16
#define R23            0x17
#define R24            0x18
#define R25            0x19
#define R26            0x1A
#define R27            0x1B
#define R28            0x1C
#define R29            0x1D
#define R30            0x1E
#define R31            0x1F
#define R32            0x20
#define R33            0x21
#define R34            0x22
#define R36            0x24
#define R37            0x25
#define R40            0x28
#define R41            0x29
#define R43            0x2B
#define R45            0x2D
#define R48            0x30
#define R49            0x31
#define R50            0x32
#define R51            0x33
#define R52            0x34
#define R53            0x35
#define R54            0x36
#define R55            0x37
#define R56            0x38
#define R57            0x39
#define R59            0x3B
#define R60            0x3C
#define R61            0x3D
#define R62            0x3E
#define R63            0x3F
#define R64            0x40
#define R65            0x41
#define R66            0x42
#define R67            0x43
#define R68            0x44
#define R69            0x45
#define R70            0x46
#define R71            0x47
#define R72            0x48
#define R73            0x49
#define R74            0x4A
#define R75            0x4B
#define R76            0x4C
#define R77            0x4D
#define R78            0x4E
#define R79            0x4F
#define R80            0x50
#define R81            0x51
#define R82            0x52
#define R83            0x53
#define R96            0x60
#define R97            0x61
#define R106           0x6A
#define R118           0x76
#define R128           0x80
#define R129           0x81
#define R130           0x82
#define R131           0x83
#define R132           0x84
#define R133           0x85
#define R134           0x86
#define R135           0x87
#define R136           0x88
#define R137           0x89
#define R139           0x8B
#define R140           0x8C
#define R141           0x8D
#define R143           0x8F
#define R144           0x90
#define R145           0x91
#define R146           0x92
#define R147           0x93
#define R148           0x94
#define R149           0x95
#define R150           0x96
#define R151           0x97
#define R152           0x98
#define R153           0x99
#define R154           0x9A
#define R157           0x9D
#define R192           0xC0
#define R193           0xC1
#define R229           0xE5	

extern void LCD_Init           (void);
extern void LCD_WindowMax      (void);
extern void LCD_PutPixel       (unsigned int x, unsigned int y);
extern void LCD_SetTextColor   (unsigned short color);
extern void LCD_SetBackColor   (unsigned short color);
extern void LCD_Clear          (unsigned short color);
extern void LCD_DrawChar       (unsigned int x, unsigned int y, unsigned short *c);
extern void LCD_DisplayChar    (unsigned int ln, unsigned int col, unsigned char  c);
extern void LCD_DisplayString  (unsigned int ln, unsigned int col, unsigned char *s);
extern void LCD_ClearLn        (unsigned int ln);
extern void LCD_Bargraph       (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int val);
extern void LCD_Bitmap         (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned char *bitmap);
extern void LCD_Bmp            (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned char *bmp);

extern void LCD_Fill(u8 xsta,u16 ysta,u8 xend,u16 yend,u16 color);
extern void LCD_DrawPoint(u16 x,u16 y);
extern void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);
extern void LCD_DrawRectangle(u8 x1, u16 y1, u8 x2, u16 y2);
extern void Draw_Circle(u8 x0,u16 y0,u8 r);
extern void LCD_ShowChar(u8 x,u16 y,u8 num,u8 size,u8 mode);
extern void LCD_ShowNum(u8 x,u16 y,u32 num,u8 len,u8 size);
extern void LCD_ShowString(u8 x,u16 y,const u8 *p);
extern void LCD_Show2Num(u8 x,u16 y,u16 num,u8 len,u8 size,u8 mode);
extern void LCD_Showcn(unsigned int x,unsigned int y,unsigned char *data);
#endif

