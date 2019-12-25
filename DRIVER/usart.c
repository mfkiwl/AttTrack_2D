#include "usart.h"
#include "stm32h7xx_hal.h"
#include "string.h"
#include "global.h"

#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

#define RXBUFFERSIZE   1 //缓存大小
uint8_t aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲



uint16_t USART_RX_STA;         		//接收状态标记	

UART_HandleTypeDef UART1_Handler; //UART句柄

void Usart_Enable_Init(void);
void uart_init(uint32_t bound);
//********************************************************************************

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
//#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;   
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)  
{ 	
	while((USART1->ISR&0X40)==0);//循环发送,直到发送完毕   
	USART1->TDR=(uint8_t)ch;      
	return ch;
}
#endif 

//********************************************************************************

void Usart_Init(void)
{
	Usart_Enable_Init();
	uart_init(115200);
}

//void Usart_SendMessage(uint8_t* buff, uint16_t length)
//{
//	//printf("\r\n您发送的消息为:\r\n");
//	HAL_UART_Transmit(&UART1_Handler, buff, length, 1000);	//发送接收到的数据
//	//HAL_UART_Transmit(&UART1_Handler,"get here\r\n",strlen("get here\r\n"),1000);	//发送接收到的数据
//	while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)!=SET);		//等待发送结束
//}

// Send string to UART
void Usart_SendMessage(uint8_t *pstr)
{
	while (*pstr != 0x00)
	{
		/* Loop until transmit data register is empty */
		HAL_UART_Transmit(&UART1_Handler, pstr, 1, 1000);	
		while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)!=SET);	
		pstr++;
	}
	return;
}

//********************************************************************************
void Usart_Enable_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_GPIOC_CLK_ENABLE();                 //开启GPIOC时钟

	GPIO_Initure.Pin = GPIO_PIN_9;                //PC9
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;      //推挽输出
	GPIO_Initure.Pull = GPIO_PULLUP;              //上拉
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;    //高速
	HAL_GPIO_Init(GPIOC,&GPIO_Initure);
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);	//PC9置1
}


//初始化IO 串口1 
//bound:波特率
void uart_init(uint32_t bound)
{	
	//UART 初始化设置
	UART1_Handler.Instance=USART1;					    //USART1
	UART1_Handler.Init.BaudRate=bound;				    //波特率
	UART1_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART1_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART1_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART1_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART1_Handler);					    //HAL_UART_Init()会使能UART1
	
	HAL_UART_Receive_IT(&UART1_Handler, (uint8_t *)aRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
} 

 
//UART底层初始化，时钟使能，引脚配置，中断配置
//此函数会被HAL_UART_Init()调用
//huart:串口句柄
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART1)//如果是串口1，进行串口1 MSP初始化
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_USART1_CLK_ENABLE();			//使能USART1时钟
	
		GPIO_Initure.Pin=GPIO_PIN_6;			//PA9  -->PA6 2019.9.9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//高速
		GPIO_Initure.Alternate=GPIO_AF7_USART1;	//复用为USART1
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化PA9

		GPIO_Initure.Pin=GPIO_PIN_7;			//PA10  -->PA7 2019.9.9
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化PA10
		
#if EN_USART1_RX
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(USART1_IRQn,3,3);			//抢占优先级3，子优先级3
#endif	
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//如果是串口1
	{
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}
	}
}
 
//串口1中断服务程序
void USART1_IRQHandler(void)                	
{ 
	uint32_t timeout=0;
    uint32_t maxDelay=0x1FFFF;
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntEnter();    
#endif
	
	HAL_UART_IRQHandler(&UART1_Handler);	//调用HAL库中断处理公用函数
	
	timeout=0;
    while (HAL_UART_GetState(&UART1_Handler)!=HAL_UART_STATE_READY)//等待就绪
	{
        timeout++;////超时处理
        if(timeout>maxDelay) break;		
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&UART1_Handler,(uint8_t *)aRxBuffer, RXBUFFERSIZE)!=HAL_OK)//一次处理完成之后，重新开启中断并设置RxXferCount为1
	{
        timeout++; //超时处理
        if(timeout>maxDelay) break;	
	}
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntExit();  											 
#endif
} 












