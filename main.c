#include <LPC17xx.h>
#include "uart.h"
#include "lcd.h"
extern unsigned char data[520];
extern uint32_t Rxlen;
int main()
{
	SystemInit();
	LCD_Init();
	UART0_Init();
	while(Rxlen<512);
	LCD_Showcn(100,100,data);
}