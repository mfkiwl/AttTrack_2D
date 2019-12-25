#include "fdcan.h"
#include "hal.h"
#include "global.h"  
#include "can_protocol.h"



FDCAN_HandleTypeDef FDCAN1_Handler;
FDCAN_RxHeaderTypeDef FDCAN1_RxHeader;
FDCAN_TxHeaderTypeDef FDCAN1_TxHeader;

void FDCAN1_Enable_Init(void);
uint8_t FDCAN1_Mode_Init(uint16_t presc,uint8_t ntsjw,uint16_t ntsg1,uint8_t ntsg2,uint32_t mode);
uint8_t FDCAN1_Send_Msg(uint32_t Id, uint8_t* msg, uint32_t len);
uint8_t FDCAN_EXT_Send_Msg(uint32_t Id, uint8_t* msg, uint32_t len);
uint8_t FDCAN1_Receive_Msg(uint8_t *buf);

//********************************************************************************
void Fdcan_Init(void)
{
	FDCAN1_Enable_Init();
	FDCAN1_Mode_Init(10,8,31,8,FDCAN_MODE_NORMAL);        //正常模式,波特率500Kbps
}

void Fdcan_SendMessage(uint32_t id, uint8_t *msg, uint16_t len)
{
	switch(len)
	{
		case 1:
			FDCAN1_Send_Msg(id, msg, FDCAN_DLC_BYTES_1);
			break;
		case 2:
			FDCAN1_Send_Msg(id, msg, FDCAN_DLC_BYTES_2);
			break;
		case 3:
			FDCAN1_Send_Msg(id, msg, FDCAN_DLC_BYTES_3);
			break;
		case 4:
			FDCAN1_Send_Msg(id, msg, FDCAN_DLC_BYTES_4);
			break;
		case 5:
			FDCAN1_Send_Msg(id, msg, FDCAN_DLC_BYTES_5);
			break;
		case 6:
			FDCAN1_Send_Msg(id, msg, FDCAN_DLC_BYTES_6);
			break;
		case 7:
			FDCAN1_Send_Msg(id, msg, FDCAN_DLC_BYTES_7);
			break;
		case 8:
			FDCAN1_Send_Msg(id, msg, FDCAN_DLC_BYTES_8);
			break;
		default:
			break;
	}
}


void FdcanExt_SendMessage(uint32_t id, uint8_t *msg, uint16_t len)
{
	switch(len)
	{
		case 1:
			FDCAN_EXT_Send_Msg(id, msg, FDCAN_DLC_BYTES_1);
			break;
		case 2:
			FDCAN_EXT_Send_Msg(id, msg, FDCAN_DLC_BYTES_2);
			break;
		case 3:
			FDCAN_EXT_Send_Msg(id, msg, FDCAN_DLC_BYTES_3);
			break;
		case 4:
			FDCAN_EXT_Send_Msg(id, msg, FDCAN_DLC_BYTES_4);
			break;
		case 5:
			FDCAN_EXT_Send_Msg(id, msg, FDCAN_DLC_BYTES_5);
			break;
		case 6:
			FDCAN_EXT_Send_Msg(id, msg, FDCAN_DLC_BYTES_6);
			break;
		case 7:
			FDCAN_EXT_Send_Msg(id, msg, FDCAN_DLC_BYTES_7);
			break;
		case 8:
			FDCAN_EXT_Send_Msg(id, msg, FDCAN_DLC_BYTES_8);
			break;
		default:
			break;
	}
}

void Fdcan_ReadMessage(uint8_t *buf)
{
	FDCAN1_Receive_Msg(buf);
}

//********************************************************************************
void FDCAN1_Enable_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_GPIOC_CLK_ENABLE();                 //开启GPIOC时钟

	GPIO_Initure.Pin = GPIO_PIN_1;                //PC1
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;      //推挽输出
	GPIO_Initure.Pull = GPIO_PULLUP;              //上拉
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;    //高速
	HAL_GPIO_Init(GPIOC,&GPIO_Initure);
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);	//PC1置1
}

uint8_t FDCAN1_Mode_Init(uint16_t presc,uint8_t ntsjw,uint16_t ntsg1,uint8_t ntsg2,uint32_t mode)
{
    FDCAN_FilterTypeDef FDCAN1_RXFilter;
        
    //初始化FDCAN1
    HAL_FDCAN_DeInit(&FDCAN1_Handler);                              //先清除以前的设置
    FDCAN1_Handler.Instance=FDCAN1;
    //FDCAN1_Handler.Init.FrameFormat=FDCAN_FRAME_CLASSIC;            //传统模式
	  FDCAN1_Handler.Init.FrameFormat=FDCAN_FRAME_FD_NO_BRS;
    FDCAN1_Handler.Init.Mode=mode;                                  //回环测试
    //FDCAN1_Handler.Init.AutoRetransmission=DISABLE;                 //关闭自动重传！传统模式下一定要关闭！！！
	  FDCAN1_Handler.Init.AutoRetransmission=ENABLE;
    //FDCAN1_Handler.Init.TransmitPause=DISABLE;                      //关闭传输暂停
	  FDCAN1_Handler.Init.TransmitPause=ENABLE;  
    //FDCAN1_Handler.Init.ProtocolException=DISABLE;                  //关闭协议异常处理
	  FDCAN1_Handler.Init.ProtocolException=ENABLE;
    FDCAN1_Handler.Init.NominalPrescaler=presc;                     //分频系数
    FDCAN1_Handler.Init.NominalSyncJumpWidth=ntsjw;                 //重新同步跳跃宽度
    FDCAN1_Handler.Init.NominalTimeSeg1=ntsg1;                      //tsg1范围:2~256
    FDCAN1_Handler.Init.NominalTimeSeg2=ntsg2;                      //tsg2范围:2~128
    FDCAN1_Handler.Init.MessageRAMOffset=0;                         //信息RAM偏移
    FDCAN1_Handler.Init.StdFiltersNbr=0;                            //标准信息ID滤波器编号
    FDCAN1_Handler.Init.ExtFiltersNbr=0;                            //扩展信息ID滤波器编号
    FDCAN1_Handler.Init.RxFifo0ElmtsNbr=1;                          //接收FIFO0元素编号
    FDCAN1_Handler.Init.RxFifo0ElmtSize=FDCAN_DATA_BYTES_8;         //接收FIFO0元素大小：8字节
    FDCAN1_Handler.Init.RxBuffersNbr=0;                             //接收缓冲编号
    FDCAN1_Handler.Init.TxEventsNbr=0;                              //发送事件编号
    FDCAN1_Handler.Init.TxBuffersNbr=0;                             //发送缓冲编号
    FDCAN1_Handler.Init.TxFifoQueueElmtsNbr=1;                      //发送FIFO序列元素编号
    FDCAN1_Handler.Init.TxFifoQueueMode=FDCAN_TX_FIFO_OPERATION;    //发送FIFO序列模式
    FDCAN1_Handler.Init.TxElmtSize=FDCAN_DATA_BYTES_8;              //发送大小:8字节
    if(HAL_FDCAN_Init(&FDCAN1_Handler)!=HAL_OK) return 1;           //初始化FDCAN
  
    //配置RX滤波器   
    FDCAN1_RXFilter.IdType=FDCAN_STANDARD_ID;                       //标准ID
    FDCAN1_RXFilter.FilterIndex=0;                                  //滤波器索引                   
    FDCAN1_RXFilter.FilterType=FDCAN_FILTER_MASK;                   //滤波器类型
    FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
    FDCAN1_RXFilter.FilterID1=0x0000;                               //32位ID
    FDCAN1_RXFilter.FilterID2=0x0000;                               //如果FDCAN配置为传统模式的话，这里是32位掩码
    if(HAL_FDCAN_ConfigFilter(&FDCAN1_Handler,&FDCAN1_RXFilter)!=HAL_OK) return 2;//滤波器初始化
    HAL_FDCAN_Start(&FDCAN1_Handler);                               //开启FDCAN
    HAL_FDCAN_ActivateNotification(&FDCAN1_Handler,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
    return 0;
}


//FDCAN1底层驱动，引脚配置，时钟使能
//HAL_FDCAN_Init()调用
//hsdram:FDCAN1句柄
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
    GPIO_InitTypeDef GPIO_Initure;
    RCC_PeriphCLKInitTypeDef FDCAN_PeriphClk;
    
    __HAL_RCC_FDCAN_CLK_ENABLE();             //使能FDCAN1时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();			        //开启GPIOA时钟
	
    //FDCAN1时钟源配置为PLL1Q
    FDCAN_PeriphClk.PeriphClockSelection=RCC_PERIPHCLK_FDCAN;
    FDCAN_PeriphClk.FdcanClockSelection=RCC_FDCANCLKSOURCE_PLL;
    HAL_RCCEx_PeriphCLKConfig(&FDCAN_PeriphClk);
    
    GPIO_Initure.Pin=GPIO_PIN_11|GPIO_PIN_12;       //PA11,12
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;              //推挽复用
    GPIO_Initure.Pull=GPIO_PULLUP;                  //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_LOW;//GPIO_SPEED_FREQ_MEDIUM;      //超高速  
    GPIO_Initure.Alternate=GPIO_AF9_FDCAN1;         //复用为CAN1
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);             //初始化
    
#if FDCAN1_RX0_INT_ENABLE     
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn,1,2);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
#endif	
}

//此函数会被HAL_FDCAN_DeInit调用
//hfdcan:fdcan句柄
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* hfdcan)
{
    __HAL_RCC_FDCAN_FORCE_RESET();       //复位FDCAN1
    __HAL_RCC_FDCAN_RELEASE_RESET();     //停止复位
    
#if FDCAN1_RX0_INT_ENABLE   
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
#endif
}

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8),可设置为FDCAN_DLC_BYTES_2~FDCAN_DLC_BYTES_8				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
uint8_t FDCAN1_Send_Msg(uint32_t Id, uint8_t* msg, uint32_t len)
{	
    FDCAN1_TxHeader.Identifier = Id;                           //32位ID
    FDCAN1_TxHeader.IdType = FDCAN_STANDARD_ID;                  //标准ID
    FDCAN1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;              //数据帧
    FDCAN1_TxHeader.DataLength = len;                            //数据长度
    FDCAN1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;            
    FDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;               //关闭速率切换
    FDCAN1_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;                //传统的CAN模式
    FDCAN1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;     //无发送事件
    FDCAN1_TxHeader.MessageMarker = 0;                           
    
    if(HAL_FDCAN_AddMessageToTxFifoQ(&FDCAN1_Handler,&FDCAN1_TxHeader,msg)!=HAL_OK) return 1;//发送
    return 0;	
}


//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8),可设置为FDCAN_DLC_BYTES_2~FDCAN_DLC_BYTES_8				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
uint8_t FDCAN_EXT_Send_Msg(uint32_t Id, uint8_t* msg, uint32_t len)
{	
    FDCAN1_TxHeader.Identifier = Id;                           //32位ID
    FDCAN1_TxHeader.IdType = FDCAN_EXTENDED_ID;                  //标准ID
    FDCAN1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;              //数据帧
    FDCAN1_TxHeader.DataLength = len;                            //数据长度
    FDCAN1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;            
    FDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;               //关闭速率切换
    FDCAN1_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;                //传统的CAN模式
    FDCAN1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;     //无发送事件
    FDCAN1_TxHeader.MessageMarker = 0;                           
    
    if(HAL_FDCAN_AddMessageToTxFifoQ(&FDCAN1_Handler,&FDCAN1_TxHeader,msg)!=HAL_OK) return 1;//发送
    return 0;	
}


//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
uint8_t FDCAN1_Receive_Msg(uint8_t *buf)
{	
    if(HAL_FDCAN_GetRxMessage(&FDCAN1_Handler,FDCAN_RX_FIFO0,&FDCAN1_RxHeader,buf)!=HAL_OK)return 0;//接收数据
	return FDCAN1_RxHeader.DataLength>>16;	
}

#if FDCAN1_RX0_INT_ENABLE  
//FDCAN1中断服务函数
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&FDCAN1_Handler);
}

//FIFO0回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
		uint8_t Rxdata[8] = {0};
		
    if((RxFifo0ITs&FDCAN_IT_RX_FIFO0_NEW_MESSAGE)!=RESET)   //FIFO1新数据中断
    {
        //提取FIFO0中接收到的数据
        HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&FDCAN1_RxHeader,Rxdata);
        //printf("id:%#x\r\n",FDCAN1_RxHeader.Identifier);
        //printf("len:%d\r\n",FDCAN1_RxHeader.DataLength>>16);
			
				FDCAN_ProtocolProcess(FDCAN1_RxHeader, Rxdata);
					
        HAL_FDCAN_ActivateNotification(hfdcan,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
    }
}
#endif	















