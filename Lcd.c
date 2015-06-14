/******************************************************************************/
/* LCD_SPI_LPC1700.c: LPC1700 low level Graphic LCD (320x240 pixels) driven  */
/*                     with SPI functions                                     */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2009 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#include <lpc17xx.h>
#include "LCD.h"
#include "Font_24x16.h"

u16 POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF; 
static unsigned short DeviceCode;
/*********************** Hardware specific configuration **********************/

/* 8bit to 16bit LCD Interface 
   
   PINS: 
   - EN      = P0.19
   - LE      = P0.20
   - DIR     = P0.21
   - CS      = P0.22
   - RS      = P0.23 
   - WR      = P0.24 
   - RD      = P0.25
   - DB[0.7] = P2.0...P2.7
   - DB[8.15]= P2.0...P2.7                                                     */

#define PIN_EN		(1 << 19)
#define PIN_LE		(1 << 20)
#define PIN_DIR		(1 << 21)
#define PIN_CS      (1 << 22)
#define PIN_RS		(1 << 23)
#define PIN_WR		(1 << 24)
#define PIN_RD		(1 << 25)   

/*------------------------- Speed dependant settings -------------------------*/

/* If processor works on high frequency delay has to be increased, it can be 
   increased by factor 2^N by this constant                                   */
#define DELAY_2N    18

/*---------------------- Graphic LCD size definitions ------------------------*/

#define WIDTH       320                 /* Screen Width (in pixels)           */
#define HEIGHT      240                 /* Screen Hight (in pixels)           */
#define BPP         16                  /* Bits per pixel                     */
#define BYPP        ((BPP+7)/8)         /* Bytes per pixel                    */

/*--------------- Graphic LCD interface hardware definitions -----------------*/

/* Pin EN setting to 0 or 1                                                   */
#define LCD_EN(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_EN) : (LPC_GPIO0->FIOCLR = PIN_EN));
/* Pin LE setting to 0 or 1                                                   */
#define LCD_LE(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_LE) : (LPC_GPIO0->FIOCLR = PIN_LE));
/* Pin DIR setting to 0 or 1                                                   */
#define LCD_DIR(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_DIR) : (LPC_GPIO0->FIOCLR = PIN_DIR));
/* Pin CS setting to 0 or 1                                                   */
#define LCD_CS(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_CS) : (LPC_GPIO0->FIOCLR = PIN_CS));
/* Pin RS setting to 0 or 1                                                   */
#define LCD_RS(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_RS) : (LPC_GPIO0->FIOCLR = PIN_RS));
/* Pin WR setting to 0 or 1                                                   */
#define LCD_WR(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_WR) : (LPC_GPIO0->FIOCLR = PIN_WR));
/* Pin RD setting to 0 or 1                                                   */
#define LCD_RD(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_RD) : (LPC_GPIO0->FIOCLR = PIN_RD));
 
/*---------------------------- Global variables ------------------------------*/

/******************************************************************************/
static volatile unsigned short TextColor = Black, BackColor = White;


/************************ Local auxiliary functions ***************************/

/*******************************************************************************
* Delay in while loop cycles                                                   *
*   Parameter:    cnt:    number of while cycles to delay                      *
*   Return:                                                                    *
*******************************************************************************/

static void delay (int cnt) 
{

  cnt <<= DELAY_2N;
  while (cnt--);
}

__asm void wait()
{
    nop
    BX lr
}

void wait_delay(int count)
{
  while(count--);
}

static __inline unsigned char lcd_send (unsigned short byte) 
{

  LPC_GPIO2->FIODIR |= 0x000000ff;  //P2.0...P2.7 Output
  LCD_DIR(1)		   				//Interface A->B
  wait();
  LCD_EN(0)	                        //Enable 2A->2B
  LPC_GPIO2->FIOPIN =  byte;        //Write D0..D7
  LCD_LE(1)                         
  wait();
  wait();
  LCD_LE(0)							//latch D0..D7
  LPC_GPIO2->FIOPIN =  byte >> 8;   //Write D8..D15 
  wait();
  return(1);
}

static __inline void wr_cmd (unsigned char c) 
{

  LCD_RS(0)
  LCD_RD(1)
  lcd_send(c);
  LCD_WR(0)
  wait();
  LCD_WR(1)
}

static __inline void wr_dat (unsigned short c) 
{

  LCD_RS(1)
  LCD_RD(1)
  lcd_send(c);
  LCD_WR(0)
  wait();
  LCD_WR(1)
}



static __inline void wr_dat_start (void) 
{
  LCD_RS(1)
}

static __inline void wr_dat_stop (void) 
{
  LCD_CS(1)
}

static __inline void wr_dat_only (unsigned short c) 
{
  lcd_send(c);
  LCD_WR(0)
  wait();
  LCD_WR(1)
}


static __inline void wr_reg (unsigned char reg, unsigned short val) 
{
  LCD_CS(0)
  wr_cmd(reg);
  wr_dat(val);
  LCD_CS(1)
}

void LCD_Init (void) { 
  /* Configure the LCD Control pins */

  LPC_GPIO2->FIODIR   |= 0x000000FF;
  LPC_GPIO0->FIODIR   |= 0x03f80000;					/*P0.19--P0.25           		*/
  LPC_GPIO0->FIOSET    = 0x03f80000;					/*设置P0.19--P0.25 为输出		*/

  delay(5);                            				 	/* Delay 50 ms           		*/

 // DeviceCode = rd_reg(0x00);

		//************* Start Initial Sequence **********//
		wr_reg(0x00, 0x0001); // Start internal OSC.
		wr_reg(0x01, 0x3B3F); // Driver output control, RL=0;REV=1;GD=1;BGR=0;SM=0;TB=1
		wr_reg(0x02, 0x0600); // set 1 line inversion
		//************* Power control setup ************/
		wr_reg(0x0C, 0x0007); // Adjust VCIX2 output voltage
		wr_reg(0x0D, 0x0006); // Set amplitude magnification of VLCD63
		wr_reg(0x0E, 0x3200); // Set alternating amplitude of VCOM
		wr_reg(0x1E, 0x00BB); // Set VcomH voltage
		wr_reg(0x03, 0x6A64); // Step-up factor/cycle setting  7b30
		//************ RAM position control **********/
		wr_reg(0x0F, 0x0000); // Gate scan position start at G0.
		wr_reg(0x44, 0xEF00); // Horizontal RAM address position
		wr_reg(0x45, 0x0000); // Vertical RAM address start position
		wr_reg(0x46, 0x013F); // Vertical RAM address end position
		// ----------- Adjust the Gamma Curve ----------//
		wr_reg(0x30, 0x0000);
		wr_reg(0x31, 0x0706);
		wr_reg(0x32, 0x0206);
		wr_reg(0x33, 0x0300);
		wr_reg(0x34, 0x0002);
		wr_reg(0x35, 0x0000);
		wr_reg(0x36, 0x0707);
		wr_reg(0x37, 0x0200);
		wr_reg(0x3A, 0x0908);
		wr_reg(0x3B, 0x0F0D);
		//************* Special command **************/
		wr_reg(0x28, 0x0006); // Enable test command
		wr_reg(0x2F, 0x12EB); // RAM speed tuning
		wr_reg(0x26, 0x7000); // Internal Bandgap strength
		wr_reg(0x20, 0xB0E3); // Internal Vcom strength
		wr_reg(0x27, 0x0044); // Internal Vcomh/VcomL timing
		wr_reg(0x2E, 0x7E45); // VCOM charge sharing time  
		//************* Turn On display ******************/
		wr_reg(0x10, 0x0000); // Sleep mode off.
		delay(8); // Wait 30mS
		wr_reg(0x11, 0x6878); // Entry mode setup. 262K type B, take care on the data bus with 16it only 
		wr_reg(0x07, 0x0033); // Display ON	*/
	
}



/*******************************************************************************
* Set draw window region to whole screen                                       *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

void LCD_WindowMax (void) 
{
  wr_reg(0x50, 0);                      /* Horizontal GRAM Start Address      */
  wr_reg(0x51, HEIGHT-1);               /* Horizontal GRAM End   Address (-1) */
  wr_reg(0x52, 0);                      /* Vertical   GRAM Start Address      */
  wr_reg(0x53, WIDTH-1);                /* Vertical   GRAM End   Address (-1) */
}
/*
*********************************************************************************************************
* Description: 	Setting the coordinate of cursor
* Arguments  : 	(Xpos , Ypos) the coordinate
* Returns    : 	None
*********************************************************************************************************
*/
void LCD_SetCursor(u8 Xpos, u16 Ypos)
{
 	if(DeviceCode==0x8999||DeviceCode==0x9919)
	{
		wr_reg(0x004E, Xpos);
		wr_reg(0X004F, Ypos);
	}
	else 
	{
	wr_reg(R32, Xpos);
	wr_reg(R33, Ypos);
	}
}
/*******************************************************************************
* Clear display                                                                *
*   Parameter:      color:    display clearing color                           *
*   Return:                                                                    *
*******************************************************************************/

void LCD_Clear (unsigned short color) 
{
  unsigned int   i;

  LCD_WindowMax();

  LCD_SetCursor(0,0);
  LCD_CS(0)			  //写数据要先置0
  wr_cmd(0x22);
  wr_dat_start();
  for(i = 0; i < (WIDTH*HEIGHT); i++)
    wr_dat_only(color);
  wr_dat_stop();
}
/*******************************************************************************
* Draw a pixel in foreground color                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*   Return:                                                                    *
*******************************************************************************/

void LCD_PutPixel (unsigned int x, unsigned int y) 
{
  LCD_SetCursor(WIDTH-1-x,y);	   
  LCD_CS(0)
  wr_cmd(0x22);				 
  wr_dat(TextColor);	
  LCD_CS(1)
}


/*******************************************************************************
* Set foreground color                                                         *
*   Parameter:      color:    foreground color                                 *
*   Return:                                                                    *
*******************************************************************************/

void LCD_SetTextColor (unsigned short color) 
{
  TextColor = color;
}


/*******************************************************************************
* Set background color                                                         *
*   Parameter:      color:    background color                                 *
*   Return:                                                                    *
*******************************************************************************/

void LCD_SetBackColor (unsigned short color) 
{
  BackColor = color;
}
/*******************************************************************************
* Draw character on given position                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   c:        pointer to character bitmap                      *
*   Return:                                                                    *
*******************************************************************************/

void LCD_DrawChar (unsigned int x, unsigned int y, unsigned short *c) 
{
  int idx = 0, i, j;

  x = WIDTH-x-CHAR_W;
  wr_reg(0x50, y);                      /* Horizontal GRAM Start Address      */
  wr_reg(0x51, y+CHAR_H-1);             /* Horizontal GRAM End   Address (-1) */
  wr_reg(0x52, x);                      /* Vertical   GRAM Start Address      */
  wr_reg(0x53, x+CHAR_W-1);             /* Vertical   GRAM End   Address (-1) */

  LCD_SetCursor(0,0);					
  LCD_CS(0)								
  wr_cmd(0x22);						
  wr_dat_start();					
  for (j = 0; j < CHAR_H; j++) {
    for (i = CHAR_W-1; i >= 0; i--) {
      if((c[idx] & (1 << i)) == 0x00) {
        wr_dat_only(BackColor);
      } else {
        wr_dat_only(TextColor);
      }
    }
    c++;
  }
  wr_dat_stop();
}


/*******************************************************************************
* Disply character on given line                                               *
*   Parameter:      ln:       line number                                      *
*                   col:      column number                                    *
*                   c:        ascii character                                  *
*   Return:                                                                    *
*******************************************************************************/

void LCD_DisplayChar (unsigned int ln, unsigned int col, unsigned char c) 
{
  c -= 32;
  LCD_DrawChar(col * CHAR_W, ln * CHAR_H, (unsigned short *)&Font_24x16[c * CHAR_H]);
}


/*******************************************************************************
* Disply string on given line                                                  *
*   Parameter:      ln:       line number                                      *
*                   col:      column number                                    *
*                   s:        pointer to string                                *
*   Return:                                                                    *
*******************************************************************************/

void LCD_DisplayString (unsigned int ln, unsigned int col, unsigned char *s) 
{
  LCD_WindowMax();
  while (*s) 
  {
    LCD_DisplayChar(ln, col++, *s++);
  }
}


/*******************************************************************************
* Clear given line                                                             *
*   Parameter:      ln:       line number                                      *
*   Return:                                                                    *
*******************************************************************************/

void LCD_ClearLn (unsigned int ln) 
{
  LCD_WindowMax();
  LCD_DisplayString(ln, 0, "                    ");
}

/*******************************************************************************
* Draw bargraph                                                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        maximum width of bargraph (in pixels)            *
*                   val:      value of active bargraph (in 1/1024)             *
*   Return:                                                                    *
*******************************************************************************/

void LCD_Bargraph (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int val) 
{
  int i,j;

  x = WIDTH-x-w;
  wr_reg(0x50, y);                      /* Horizontal GRAM Start Address      */
  wr_reg(0x51, y+CHAR_H-1);             /* Horizontal GRAM End   Address (-1) */
  wr_reg(0x52, x);                      /* Vertical   GRAM Start Address      */
  wr_reg(0x53, x+w-1);                  /* Vertical   GRAM End   Address (-1) */

  val = (val * w) >> 10;                /* Scale value for 24x12 characters   */
  LCD_SetCursor(0,0);
  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat_start();
  for (i = 0; i < h; i++) {
    for (j = w-1; j >= 0; j--) {
      if(j >= val) {
        wr_dat_only(BackColor);
      } else {
        wr_dat_only(TextColor);
      }
    }
  }
  wr_dat_stop();
}


/*******************************************************************************
* Display graphical bitmap image at position x horizontally and y vertically   *
* (This function is optimized for 16 bits per pixel format, it has to be       *
*  adapted for any other bits per pixel format)                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        width of bitmap                                  *
*                   h:        height of bitmap                                 *
*                   bitmap:   address at which the bitmap data resides         *
*   Return:                                                                    *
*******************************************************************************/

void LCD_Bitmap (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned char *bitmap) 
{
  unsigned int    i, j;
  unsigned short *bitmap_ptr = (unsigned short *)bitmap;

  x = WIDTH-x-w;
  wr_reg(0x50, y);                      /* Horizontal GRAM Start Address      */
  wr_reg(0x51, y+h-1);                  /* Horizontal GRAM End   Address (-1) */
  wr_reg(0x52, x);                      /* Vertical   GRAM Start Address      */
  wr_reg(0x53, x+w-1);                  /* Vertical   GRAM End   Address (-1) */

  wr_reg(0x20, y);
  wr_reg(0x21, x);
  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat_start();
  for (j = 0; j < h; j++) {
    bitmap_ptr += w-1;
    for (i = 0; i < w; i++) {
      wr_dat_only(*bitmap_ptr--);
    }
    bitmap_ptr += w+1;
  }
  wr_dat_stop();
}


/*******************************************************************************
* Display graphical bmp file image at position x horizontally and y vertically *
* (This function is optimized for 16 bits per pixel format, it has to be       *
*  adapted for any other bits per pixel format)                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        width of bitmap                                  *
*                   h:        height of bitmap                                 *
*                   bmp:      address at which the bmp data resides            *
*   Return:                                                                    *
*******************************************************************************/

void LCD_Bmp (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned char *bmp) 
{
  unsigned int    i, j;
  unsigned short *bitmap_ptr = (unsigned short *)bmp;

  x = WIDTH-x-w;
//  wr_reg(0x50, y);                      /* Horizontal GRAM Start Address      */
//  wr_reg(0x51, y+h-1);                  /* Horizontal GRAM End   Address (-1) */
//  wr_reg(0x52, x);                      /* Vertical   GRAM Start Address      */
//  wr_reg(0x53, x+w-1);                  /* Vertical   GRAM End   Address (-1) */
	wr_reg(0x44,y|((y+h-1)<<8));
	wr_reg(0x45,x);
	wr_reg(0x46,x+w-1);
//  wr_reg(0x20, y);
//  wr_reg(0x21, x);
	wr_reg(0x004E, y);
	wr_reg(0X004F, x);
  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat_start();
  bitmap_ptr += (h*w)-1;
  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      wr_dat_only(*bitmap_ptr--);
    }
  }
  wr_dat_stop();
}
/******************************************************************************/

 
/*
*********************************************************************************************************
* Description: 	Write start GRAM Command
* Arguments  : 	None
* Returns    : 	None
*********************************************************************************************************
*/
void LCD_WriteRAM_Prepare(void)
{
	wr_cmd(R34);
}
/*
*********************************************************************************************************
* Description: 	In designated areas within the specified color display 
* Arguments  : 	(xsta ,ysta) start coordinate
*				(xend ,yend) end   coordinate
*				color        I don't know  (ˇ-ˇ）
* Returns    : 	None
*********************************************************************************************************
*/
void LCD_Fill(u8 xsta,u16 ysta,u8 xend,u16 yend,u16 color)
{                    
    u32 n;
	/*设置窗口*/										
	wr_reg(R80, xsta); 							/*水平方向GRAM起始地址*/
	wr_reg(R81, xend); 							/*水平方向GRAM结束地址*/
	wr_reg(R82, ysta); 							/*垂直方向GRAM起始地址*/
	wr_reg(R83, yend); 							/*垂直方向GRAM结束地址*/	
	LCD_SetCursor(xsta,ysta);					/*设置光标位置		  */
  LCD_CS(0)	  
	LCD_WriteRAM_Prepare();  					/*开始写入GRAM		  */
  wr_dat_start();	 	   	   
	n=(u32)(yend-ysta+1)*(xend-xsta+1);    
	while(n--){wr_dat_only(color);}				/*显示所填充的颜色.   */
	/*恢复设置*/
	wr_reg(R80, 0x0000); 						/*水平方向GRAM起始地址*/
	wr_reg(R81, 0x00EF); 						/*水平方向GRAM结束地址*/
	wr_reg(R82, 0x0000); 						/*垂直方向GRAM起始地址*/
	wr_reg(R83, 0x013F); 						/*垂直方向GRAM结束地址*/	    
} 
/*
*********************************************************************************************************
* Description: 	DrawPoint
* Arguments  : 	(x ,y) The coordinate
* Returns    : 	None
*********************************************************************************************************
*/
void LCD_DrawPoint (u16 x,u16 y ) 
{

  LCD_WindowMax();

  LCD_SetCursor(x,y);
  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat_start();
  wr_dat_only(POINT_COLOR);
  wr_dat_stop();
}
/*
*********************************************************************************************************
* Description: 	Draw lines 
* Arguments  : 	(x1 ,y1) start coordinate
*				(x2 ,y2) end   coordinate
* Returns    : 	None
*********************************************************************************************************
*/ 
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 

	delta_x=x2-x1; 											/*计算坐标增量		*/ 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; 									/*设置单步方向		*/ 
	else if(delta_x==0)incx=0;								/*垂直线 	  		*/
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;								/*水平线 	  		*/
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; 					/*选取基本增量坐标轴*/ 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )								/*画线输出 			*/
	{  
		LCD_DrawPoint(uRow,uCol);							/*画点 				*/
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}   
/*
*********************************************************************************************************
* Description: 	Draw rectanguar 
* Arguments  : 	(x1 ,y1) (x2 ,y2) The diagonal Level vertical 
* Returns    : 	None
*********************************************************************************************************
*/ 
void LCD_DrawRectangle(u8 x1, u16 y1, u8 x2, u16 y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}

/*
*********************************************************************************************************
* Description: 	In designated areas within the specified circle 
* Arguments  : 	(x0 ,y0) center coordinate
*				 r       radius coordinate
* Returns    : 	None
*********************************************************************************************************
*/
void Draw_Circle(u8 x0,u16 y0,u8 r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             								/*判断下个点位置的标志*/
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a);                 
		LCD_DrawPoint(x0+b,y0-a);                    
		LCD_DrawPoint(x0-a,y0+b);                
		LCD_DrawPoint(x0-b,y0-a);                    
		LCD_DrawPoint(x0-a,y0-b);                    
		LCD_DrawPoint(x0+b,y0+a);                      
		LCD_DrawPoint(x0+a,y0-b);          
		LCD_DrawPoint(x0+a,y0+b);          
		LCD_DrawPoint(x0-b,y0+a);             
		a++;
		/*使用Bresenham算法画圆*/    
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 
		LCD_DrawPoint(x0+a,y0+b);
	}
} 
/*
*********************************************************************************************************
* Description: 	DrawPoint or white
* Arguments  : 	(x ,y) The coordinate
* Returns    : 	None
*********************************************************************************************************
*/
void LCD_DrawPoint1 (u16 x,u16 y ,u16 color) 
{

  LCD_WindowMax();

  LCD_SetCursor(x,y);
  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat_start();
  wr_dat_only(color);
  wr_dat_stop();
}
/*
*********************************************************************************************************
* Description: 	In designated areas within the specified character 
* Arguments  : 	(x0 ,y0) The coordinate
*				 num     The value
*				size     size (16 or 12)
*               mode     superposition(1) or not 0
* Returns    : 	None
*********************************************************************************************************
*/
void LCD_ShowChar(u8 x,u16 y,u8 num,u8 size,u8 mode)
{       
#define MAX_CHAR_POSX 232
#define MAX_CHAR_POSY 304 
    u8 temp;
    u8 pos,t;      
    if(x>MAX_CHAR_POSX||y>MAX_CHAR_POSY)return;	    
	/*设置窗口*/										
	wr_reg(R80,x);           							/*水平方向GRAM起始地址 */
	wr_reg(R81,x+(size/2-1));							/*水平方向GRAM结束地址 */
	wr_reg(R82,y);           							/*垂直方向GRAM起始地址 */
	wr_reg(R83,y+size-1);    							/*垂直方向GRAM结束地址 */	
	LCD_SetCursor(x,y);            						/*设置光标位置  	   */
	LCD_WriteRAM_Prepare();        						/*开始写入GRAM	   	   */
	num=num-' ';										/*得到偏移后的值	   */
	if(!mode) 											/*非叠加方式		   */
	{
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];
			else temp=asc2_1608[num][pos];		 
			for(t=0;t<size/2;t++)
		    {                 
		        if(temp&0x01)
				{
       				LCD_DrawPoint1(x+t,y+pos,POINT_COLOR);
				}
				else
				{
       				LCD_DrawPoint1(x+t,y+pos,BACK_COLOR);			
				}
				// wr_dat(BACK_COLOR);	        
		        temp>>=1; 
		    }
		}	
	}else												/*叠加方式   */
	{
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];		/*调用1206字体*/
			else temp=asc2_1608[num][pos];		 		/*调用1608字体*/
			for(t=0;t<size/2;t++)
		    {                 
		        if(temp&0x01)LCD_DrawPoint(x+t,y+pos);	/*画一个点*/     
		        temp>>=1; 
		    }
		}
	}	    
	/*恢复窗体大小*/	 
	wr_reg(R80, 0x0000); 								/*水平方向GRAM起始地址*/
	wr_reg(R81, 0x00EF); 								/*水平方向GRAM结束地址*/
	wr_reg(R82, 0x0000); 								/*垂直方向GRAM起始地址*/
	wr_reg(R83, 0x013F); 								/*垂直方向GRAM结束地址*/
}
  
/*
*********************************************************************************************************
* Description: 	m^n function
* Arguments  : 	m n
* Returns    : 	None
*********************************************************************************************************
*/
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}	

/*
*********************************************************************************************************
* Description: 	Show num (from 0 to 4294967295)
* Arguments  : 	(x ,y) The coordinate
*				 num     The value
*				 len	 The number of num
*				size     size (16 or 12)
* Returns    : 	None
*********************************************************************************************************
*/	 
void LCD_ShowNum(u8 x,u16 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,0); 
	}
} 
/*
*********************************************************************************************************
* Description: 	Show string (from 0 to 4294967295)
* Arguments  : 	(x ,y) The coordinate
*				 *p    The value
* Returns    : 	None
*********************************************************************************************************
*/	
void LCD_ShowString(u8 x,u16 y,const u8 *p)
{         
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;LCD_Clear(White);}
        LCD_ShowChar(x,y,*p,16,0);
        x+=8;
        p++;
    }  
}
/*
*********************************************************************************************************
* Description: 	Show num (from 0 to 99)
* Arguments  : 	(x ,y) The coordinate
*				 num     The value
*				 len	 The number of num
*				size     size (16 or 12)
*				mode     superposition(1) or not 0
* Returns    : 	None
*********************************************************************************************************
*/	 
void LCD_Show2Num(u8 x,u16 y,u16 num,u8 len,u8 size,u8 mode)
{         	
	u8 t,temp;						   
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,mode); 
	}
}
void LCD_Showcn(unsigned int x,unsigned int y,unsigned char *data)
{
		int i,j,k;
	//	uint64_t *ptr=(uint64_t *)data;
		unsigned short *ptr=(unsigned short *)data;
		unsigned short temp;
//		unsigned char temp;
		x=WIDTH-x-64;
		
		wr_reg(0x44,y|((y+64-1)<<8));
		wr_reg(0x45,x);
		wr_reg(0x46,x+64-1);
		
		wr_reg(0x004E,y);
		wr_reg(0x004f,x);
		LCD_CS(0)
		wr_cmd(0x22);
		wr_dat_start();
	for(i=0;i<64;i++)
	{
		ptr+=4;
		for(j=0;j<4;j++)
		{
			ptr--;
			temp=*ptr;
			for(k=0;k<8;k++)
				if((temp&(1<<(k+8)))==0x0000)
					wr_dat_only(0xffff);
				else
					wr_dat_only(0x0000);
			for(k=0;k<8;k++)
				if((temp&(1<<k))==0x0000)
					wr_dat_only(0xffff);
				else
					wr_dat_only(0x0000);
			
		}
		ptr+=4;
	}
		wr_dat_stop();
}
		
