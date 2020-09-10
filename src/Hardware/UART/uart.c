#include "uart.h"

volatile u16 gu16Data = 0;
volatile u16 isFirstByte = 0;	// 0:第0个byte 高byte; 1:第1个byte 低byte;
volatile u8 UartFlag = 0;	// 0 :没有发送，1发送了一个数据;

/************************************************************
 函 数 名  : UART_Init
 功能描述  : 串口初始化
 				标准 UART 工作模式 1：8 位 UART，波特率可变
				73MHz,波特率115200,SREL=1004
 输入参数  : 无
 输出参数  : void
************************************************************/
void UART_Init()
{
	SFRPAGE = 0x00;

	SCON |= 0x50;	// 使用8位uart; 允许接受
	TCON |= 0x01;	// uart0 的波特率产生选择:使用S0RELL和S0RELH寄存器计数产生波特率;不使用双倍波特率
	SRELH = 0x03;	// 24MHz,波特率为115200,则SREL为1017
	SRELL = 0xf9;
}

#if(EN_UART_TX_ADC)
/*****************************************************************************
 函 数 名  : UART_TX_EXT
 功能描述  : 串口发送功能
 输入参数  : u16data
 输出参数  : void
*****************************************************************************/
void UART_TX_EXT(u16  u16data)
{
	volatile u8 u8Flag = 0;
	volatile u8 u8SfrPage = 0;

	u8SfrPage = SFRPAGE;
	
	SFRPAGE = 0x00;
	if (0x1 == UartFlag)
	{
		u8Flag = TI;

		if(u8Flag != 0x00)	// 上次的发送完成
		{
			TI = 0;
			if(isFirstByte == 0x00)
			{
				gu16Data = u16data;
				isFirstByte = 0x01;
				SBUF = (gu16Data >> 8);   // 送入缓冲区
			}
			else
			{
				isFirstByte = 0x00;
				SBUF = (gu16Data);   // 送入缓冲区
			}
		}
	}
	else
	{
		SFRPAGE = 0x00;
		SBUF = 0x00;          	 //送入缓冲区
		UartFlag = 1;
	}
	SFRPAGE = u8SfrPage;
}
#endif

#if(EN_UART_STR_Ps||EN_UART_STR_Pss)
/************************************************************
 函 数 名  : UART_TX
 功能描述  : 串口发送函数
 输入参数  : u8data
 输出参数  : void
************************************************************/
void UART_TX(u8 u8data)
{
	volatile u8 u8SfrPage = 0;

	u8SfrPage = SFRPAGE;

	SFRPAGE = 0x00;
	SBUF = u8data;	// 送入缓冲区
	while(TI == 0);	// 等待发送完毕
	TI = 0;	// 软件清零

	SFRPAGE = u8SfrPage;
}

/*****************************************************************************
 函 数 名  : UART_SEND_STR
 功能描述  : 发送字符串
 输入参数  : char *myStr , u16   myNumber
 输出参数  : void
*****************************************************************************/
void UART_SEND_STR(char *myStr,u16 myNumber)
{
	volatile u16 i = 0;
	volatile u16 nTmp = 0;
	volatile u8 u8SfrPage = 0; 

	u8SfrPage = SFRPAGE;
	
	SFRPAGE = 0x00;
	SBUF = ' '; 
	while(TI == 0); 
	TI = 0; 
	SBUF = ' '; 
	while(TI == 0); 
	TI = 0; 
	
	while(*myStr!='\0')
	{
		SBUF = *myStr++; 
		while(TI == 0);
		TI = 0; 
	}
	
	if(myNumber != NONE)
	{
		for(i = 10000;i >= 1;i = i/10)
		{
			nTmp = myNumber/i;
			if(nTmp >=1 ) UART_TX((u8)(nTmp%10 + 0x30));
		}
	}

	SFRPAGE = u8SfrPage;
}
#endif

