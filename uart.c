#include <LPC17xx.h>
#include "uart.h"
#include <stdio.h>
#include <stdarg.h>

#define FOSC  12000000 //振荡器频率
#define FCCLK  (FOSC*6)  //主时钟频率
#define FCCO		(FCCLK*4) //PLL时钟
#define FPCLK		25000000  //外设时钟频率

#define UART0_BPS		115200
unsigned char data[520];
uint32_t Rxlen=0;

void UART0_Init(void)
{
		uint16_t	usFdiv;
		
		LPC_PINCON->PINSEL0|=(1<<4);		//P0.2为TXD0；
		LPC_PINCON->PINSEL0|=(1<<6);		//P0.3为RXD0；
		
		LPC_SC->PCONP=LPC_SC->PCONP|0x08;  //打开串口0；
		
		LPC_UART0->LCR=0x83;
		usFdiv=(FPCLK/16)/UART0_BPS;	//设置波特率
		LPC_UART0->DLM=usFdiv/256;
		LPC_UART0->DLL=usFdiv%256;
		LPC_UART0->LCR=0x03;
		LPC_UART0->FCR=0x07|(0x02<<6);   //设置8字节FIFO
		LPC_UART0->IER=0x01;   //接收中断
		NVIC_EnableIRQ(UART0_IRQn);
		NVIC_SetPriority(UART0_IRQn,NVIC_PRIORTY_UART0);
}
int UART0_SendByte(unsigned char ucData)
{
		while(!(LPC_UART0->LSR&0x20));
		return (LPC_UART0->THR=ucData);
}
int UART0_GetChar(void)
{
		while(!(LPC_UART0->LSR&0x01));
		return (LPC_UART0->RBR);
}
void UART0_SendString(unsigned char *s)
{
		while(*s!=0)
		{
				UART0_SendByte(*s++);
		}
}
void UART0_SendChar(uint16_t disp)
{
		uint16_t dispbuf[4];
		uint8_t i;
	
		dispbuf[3]=disp%10+'0';
		dispbuf[2]=disp/10%10+'0';
		dispbuf[1]=disp/100%10+'0';
		dispbuf[0]=disp/1000%10+'0';
		for(i=0;i<4;i++)
				UART0_SendByte(dispbuf[i]);
}
void UART0_IRQHandler()
{
		volatile uint32_t iir,count,i;
		count=16;
		iir=LPC_UART0->IIR;
		switch(iir&0x0f)
		{	
			case 0x02:break;  //发送中断
			case 0x04:				//接收中断
			case 0x0c:				//接收超时
						if(LPC_UART0->LSR&0x01)
									data[Rxlen++]=LPC_UART0->RBR;
						break;
			default :break;
		}
}