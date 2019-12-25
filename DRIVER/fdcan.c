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
	FDCAN1_Mode_Init(10,8,31,8,FDCAN_MODE_NORMAL);        //����ģʽ,������500Kbps
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
	
	__HAL_RCC_GPIOC_CLK_ENABLE();                 //����GPIOCʱ��

	GPIO_Initure.Pin = GPIO_PIN_1;                //PC1
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;      //�������
	GPIO_Initure.Pull = GPIO_PULLUP;              //����
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;    //����
	HAL_GPIO_Init(GPIOC,&GPIO_Initure);
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);	//PC1��1
}

uint8_t FDCAN1_Mode_Init(uint16_t presc,uint8_t ntsjw,uint16_t ntsg1,uint8_t ntsg2,uint32_t mode)
{
    FDCAN_FilterTypeDef FDCAN1_RXFilter;
        
    //��ʼ��FDCAN1
    HAL_FDCAN_DeInit(&FDCAN1_Handler);                              //�������ǰ������
    FDCAN1_Handler.Instance=FDCAN1;
    //FDCAN1_Handler.Init.FrameFormat=FDCAN_FRAME_CLASSIC;            //��ͳģʽ
	  FDCAN1_Handler.Init.FrameFormat=FDCAN_FRAME_FD_NO_BRS;
    FDCAN1_Handler.Init.Mode=mode;                                  //�ػ�����
    //FDCAN1_Handler.Init.AutoRetransmission=DISABLE;                 //�ر��Զ��ش�����ͳģʽ��һ��Ҫ�رգ�����
	  FDCAN1_Handler.Init.AutoRetransmission=ENABLE;
    //FDCAN1_Handler.Init.TransmitPause=DISABLE;                      //�رմ�����ͣ
	  FDCAN1_Handler.Init.TransmitPause=ENABLE;  
    //FDCAN1_Handler.Init.ProtocolException=DISABLE;                  //�ر�Э���쳣����
	  FDCAN1_Handler.Init.ProtocolException=ENABLE;
    FDCAN1_Handler.Init.NominalPrescaler=presc;                     //��Ƶϵ��
    FDCAN1_Handler.Init.NominalSyncJumpWidth=ntsjw;                 //����ͬ����Ծ���
    FDCAN1_Handler.Init.NominalTimeSeg1=ntsg1;                      //tsg1��Χ:2~256
    FDCAN1_Handler.Init.NominalTimeSeg2=ntsg2;                      //tsg2��Χ:2~128
    FDCAN1_Handler.Init.MessageRAMOffset=0;                         //��ϢRAMƫ��
    FDCAN1_Handler.Init.StdFiltersNbr=0;                            //��׼��ϢID�˲������
    FDCAN1_Handler.Init.ExtFiltersNbr=0;                            //��չ��ϢID�˲������
    FDCAN1_Handler.Init.RxFifo0ElmtsNbr=1;                          //����FIFO0Ԫ�ر��
    FDCAN1_Handler.Init.RxFifo0ElmtSize=FDCAN_DATA_BYTES_8;         //����FIFO0Ԫ�ش�С��8�ֽ�
    FDCAN1_Handler.Init.RxBuffersNbr=0;                             //���ջ�����
    FDCAN1_Handler.Init.TxEventsNbr=0;                              //�����¼����
    FDCAN1_Handler.Init.TxBuffersNbr=0;                             //���ͻ�����
    FDCAN1_Handler.Init.TxFifoQueueElmtsNbr=1;                      //����FIFO����Ԫ�ر��
    FDCAN1_Handler.Init.TxFifoQueueMode=FDCAN_TX_FIFO_OPERATION;    //����FIFO����ģʽ
    FDCAN1_Handler.Init.TxElmtSize=FDCAN_DATA_BYTES_8;              //���ʹ�С:8�ֽ�
    if(HAL_FDCAN_Init(&FDCAN1_Handler)!=HAL_OK) return 1;           //��ʼ��FDCAN
  
    //����RX�˲���   
    FDCAN1_RXFilter.IdType=FDCAN_STANDARD_ID;                       //��׼ID
    FDCAN1_RXFilter.FilterIndex=0;                                  //�˲�������                   
    FDCAN1_RXFilter.FilterType=FDCAN_FILTER_MASK;                   //�˲�������
    FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //������0������FIFO0  
    FDCAN1_RXFilter.FilterID1=0x0000;                               //32λID
    FDCAN1_RXFilter.FilterID2=0x0000;                               //���FDCAN����Ϊ��ͳģʽ�Ļ���������32λ����
    if(HAL_FDCAN_ConfigFilter(&FDCAN1_Handler,&FDCAN1_RXFilter)!=HAL_OK) return 2;//�˲�����ʼ��
    HAL_FDCAN_Start(&FDCAN1_Handler);                               //����FDCAN
    HAL_FDCAN_ActivateNotification(&FDCAN1_Handler,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
    return 0;
}


//FDCAN1�ײ��������������ã�ʱ��ʹ��
//HAL_FDCAN_Init()����
//hsdram:FDCAN1���
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
    GPIO_InitTypeDef GPIO_Initure;
    RCC_PeriphCLKInitTypeDef FDCAN_PeriphClk;
    
    __HAL_RCC_FDCAN_CLK_ENABLE();             //ʹ��FDCAN1ʱ��
    __HAL_RCC_GPIOA_CLK_ENABLE();			        //����GPIOAʱ��
	
    //FDCAN1ʱ��Դ����ΪPLL1Q
    FDCAN_PeriphClk.PeriphClockSelection=RCC_PERIPHCLK_FDCAN;
    FDCAN_PeriphClk.FdcanClockSelection=RCC_FDCANCLKSOURCE_PLL;
    HAL_RCCEx_PeriphCLKConfig(&FDCAN_PeriphClk);
    
    GPIO_Initure.Pin=GPIO_PIN_11|GPIO_PIN_12;       //PA11,12
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;              //���츴��
    GPIO_Initure.Pull=GPIO_PULLUP;                  //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_LOW;//GPIO_SPEED_FREQ_MEDIUM;      //������  
    GPIO_Initure.Alternate=GPIO_AF9_FDCAN1;         //����ΪCAN1
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);             //��ʼ��
    
#if FDCAN1_RX0_INT_ENABLE     
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn,1,2);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
#endif	
}

//�˺����ᱻHAL_FDCAN_DeInit����
//hfdcan:fdcan���
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* hfdcan)
{
    __HAL_RCC_FDCAN_FORCE_RESET();       //��λFDCAN1
    __HAL_RCC_FDCAN_RELEASE_RESET();     //ֹͣ��λ
    
#if FDCAN1_RX0_INT_ENABLE   
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
#endif
}

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8),������ΪFDCAN_DLC_BYTES_2~FDCAN_DLC_BYTES_8				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
uint8_t FDCAN1_Send_Msg(uint32_t Id, uint8_t* msg, uint32_t len)
{	
    FDCAN1_TxHeader.Identifier = Id;                           //32λID
    FDCAN1_TxHeader.IdType = FDCAN_STANDARD_ID;                  //��׼ID
    FDCAN1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;              //����֡
    FDCAN1_TxHeader.DataLength = len;                            //���ݳ���
    FDCAN1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;            
    FDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;               //�ر������л�
    FDCAN1_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;                //��ͳ��CANģʽ
    FDCAN1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;     //�޷����¼�
    FDCAN1_TxHeader.MessageMarker = 0;                           
    
    if(HAL_FDCAN_AddMessageToTxFifoQ(&FDCAN1_Handler,&FDCAN1_TxHeader,msg)!=HAL_OK) return 1;//����
    return 0;	
}


//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8),������ΪFDCAN_DLC_BYTES_2~FDCAN_DLC_BYTES_8				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
uint8_t FDCAN_EXT_Send_Msg(uint32_t Id, uint8_t* msg, uint32_t len)
{	
    FDCAN1_TxHeader.Identifier = Id;                           //32λID
    FDCAN1_TxHeader.IdType = FDCAN_EXTENDED_ID;                  //��׼ID
    FDCAN1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;              //����֡
    FDCAN1_TxHeader.DataLength = len;                            //���ݳ���
    FDCAN1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;            
    FDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;               //�ر������л�
    FDCAN1_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;                //��ͳ��CANģʽ
    FDCAN1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;     //�޷����¼�
    FDCAN1_TxHeader.MessageMarker = 0;                           
    
    if(HAL_FDCAN_AddMessageToTxFifoQ(&FDCAN1_Handler,&FDCAN1_TxHeader,msg)!=HAL_OK) return 1;//����
    return 0;	
}


//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
uint8_t FDCAN1_Receive_Msg(uint8_t *buf)
{	
    if(HAL_FDCAN_GetRxMessage(&FDCAN1_Handler,FDCAN_RX_FIFO0,&FDCAN1_RxHeader,buf)!=HAL_OK)return 0;//��������
	return FDCAN1_RxHeader.DataLength>>16;	
}

#if FDCAN1_RX0_INT_ENABLE  
//FDCAN1�жϷ�����
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&FDCAN1_Handler);
}

//FIFO0�ص�����
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
		uint8_t Rxdata[8] = {0};
		
    if((RxFifo0ITs&FDCAN_IT_RX_FIFO0_NEW_MESSAGE)!=RESET)   //FIFO1�������ж�
    {
        //��ȡFIFO0�н��յ�������
        HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&FDCAN1_RxHeader,Rxdata);
        //printf("id:%#x\r\n",FDCAN1_RxHeader.Identifier);
        //printf("len:%d\r\n",FDCAN1_RxHeader.DataLength>>16);
			
				FDCAN_ProtocolProcess(FDCAN1_RxHeader, Rxdata);
					
        HAL_FDCAN_ActivateNotification(hfdcan,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
    }
}
#endif	















