#include "system.h"
#include "hal.h"
#include "global.h"
#include "angle_cal.h"


extern SPI_HandleTypeDef SPI2_Handler;
extern SPI_HandleTypeDef SPI1_Handler; 

void AD7682_Init(void);
uint16_t AD7682_SendRequest(uint16_t Request);
uint16_t AD7682_Conversion(uint8_t channel);
void SCA3300_Init(void);
static uint32_t SCA3300_SendRequest(uint32_t Request);
static void SCA3300_FunCheck(void);
/***************************************Interface function***********************************************************/
void system_init(void)
{
	AD7682_Init();
	SCA3300_Init();
	SCA3300_FunCheck();
//	MachineInit();
}

/**************************************************************************************************************/
void AD7682_Init(void)
{
	hal.ptimer->wait_ms(50);
	AD7682_SendRequest(0xF224);
	hal.ptimer->wait_us(40);
	AD7682_SendRequest(0xF224);
	hal.ptimer->wait_us(40);
	AD7682_SendRequest(0xF224);
	hal.ptimer->wait_us(40);
	AD7682_SendRequest(0xF224);
	hal.ptimer->wait_us(40);
}

uint16_t AD7682_SendRequest(uint16_t Request)
{
	uint16_t Response;
	uint8_t Request_buff[2] = {0};
	uint8_t Response_buff[2] = {0};
	
	Request_buff[0] = Request>>8;
	Request_buff[1] = Request;
	
	hal.pspi->send_message(AD7682, Request_buff, Response_buff, 2);

	Response  = (uint16_t)Response_buff[0]<<8;
	Response |= (uint16_t)Response_buff[1];
	
	return Response;
}

void AD7682_Sampling(void)
{
	Vector3u16 Sampling_Rate = {0};
	
	Sampling_Rate.x = AD7682_Conversion(AD7682_REQUEST_CHANNEL0);
	Sampling_Rate.y = AD7682_Conversion(AD7682_REQUEST_CHANNEL1);
	Sampling_Rate.z = AD7682_Conversion(AD7682_REQUEST_CHANNEL2);

	Sampling_Gyro_Sum.x += Sampling_Rate.x;
	Sampling_Gyro_Sum.y += Sampling_Rate.y;
	Sampling_Gyro_Sum.z += Sampling_Rate.z;
	
	Result_Gyro_Count++;
}

uint16_t AD7682_Conversion(uint8_t channel) //channel is 0.1.2.3
{
	uint16_t AD_result = 0;
	switch (channel)
	{
		case 0:
			AD7682_SendRequest(0xF024);
			hal.ptimer->wait_us(20);
			AD7682_SendRequest(0xF024);
			hal.ptimer->wait_us(20);
			AD7682_SendRequest(0xF024);
			hal.ptimer->wait_us(20);
			AD_result = AD7682_SendRequest(0xF024);
			break;
		case 1:
			AD7682_SendRequest(0xF224);
			hal.ptimer->wait_us(20);
			AD7682_SendRequest(0xF224);
			hal.ptimer->wait_us(20);
			AD7682_SendRequest(0xF224);
			hal.ptimer->wait_us(20);
			AD_result = AD7682_SendRequest(0xF224);
			break;
		case 2:
			AD7682_SendRequest(0xF424);
			hal.ptimer->wait_us(20);
			AD7682_SendRequest(0xF424);
			hal.ptimer->wait_us(20);
			AD7682_SendRequest(0xF424);
			hal.ptimer->wait_us(20);
			AD_result = AD7682_SendRequest(0xF424);
			break;
		case 3:
			AD7682_SendRequest(0xF624);
			hal.ptimer->wait_us(20);
			AD7682_SendRequest(0xF624);
			hal.ptimer->wait_us(20);
			AD7682_SendRequest(0xF624);
			hal.ptimer->wait_us(20);
			AD_result = AD7682_SendRequest(0xF624);
			break;
		default:
			break;
	}
	hal.ptimer->wait_us(20);
	return (AD_result);
}



/**************************************************************************************************************/
void SCA3300_Init(void)
{
	// Sensor power-up
	hal.ptimer->wait_ms(25); // Wait 25 ms until the SCC2000 is accessible via SPI
	//SCA3300_SendRequest(REQ_CHANGE_MODE1);
	hal.ptimer->wait_ms(10); // Wait 25 ms until the SCC2000 is accessible via SPI
	SCA3300_SendRequest(REQ_READ_STAT_SUM);
//	SCA3300_SendRequest(REQ_READ_ACC_X);
//	SCA3300_SendRequest(REQ_READ_ACC_Y);
//	SCA3300_SendRequest(REQ_READ_ACC_Z);
	SCA3300_SendRequest(REQ_READ_STO);
	SCA3300_SendRequest(REQ_SW_RESET);
	hal.ptimer->wait_ms(1000);
}

// Send request to sensor and read back the response for previous request.
static uint32_t SCA3300_SendRequest(uint32_t Request)
{
	uint32_t Response;
	uint8_t Request_buff[4] = {0};
	uint8_t Response_buff[4] = {0};
	
	Request_buff[0] = (uint8_t)(Request>>24);  //pay attention here, is first uint8_t, then left shift
	Request_buff[1] = (uint8_t)(Request>>16);
	Request_buff[2] = (uint8_t)(Request>>8);
	Request_buff[3] = (uint8_t)Request;
	
	hal.pspi->send_message(SCA3300, Request_buff, Response_buff, 4);
	
	Response  = (uint32_t)Response_buff[0]<<24;
	Response |= (uint32_t)Response_buff[1]<<16;
	Response |= (uint32_t)Response_buff[2]<<8;
	Response |= (uint32_t)Response_buff[3];
	
	return Response;
}

void SCA3300_Sampling(void)
{
	uint8_t RSdata = 0;
	Vector3u32 Response_Acc = {0};
	uint32_t Response_Temp = 0;
	uint32_t Response_Sum = 0;
	Vector3s16 Sampling_Acc = {0};
	int16_t Sampling_Temp = 0;
	
//	Response_Temp  = SCA3300_SendRequest(REQ_READ_ACC_X);
//	Response_Acc.x = SCA3300_SendRequest(REQ_READ_ACC_Y);
//	Response_Acc.y = SCA3300_SendRequest(REQ_READ_ACC_Z);
//	Response_Acc.z = SCA3300_SendRequest(REQ_READ_TEMP);	                 
	Response_Sum   = SCA3300_SendRequest(REQ_READ_ACC_X);
	hal.ptimer->wait_us(20);
	Response_Acc.x = SCA3300_SendRequest(REQ_READ_ACC_Y);
	hal.ptimer->wait_us(20);
	Response_Acc.y = SCA3300_SendRequest(REQ_READ_ACC_Z);
	hal.ptimer->wait_us(20);
	Response_Acc.z = SCA3300_SendRequest(REQ_READ_TEMP);
	hal.ptimer->wait_us(20);
	Response_Temp  = SCA3300_SendRequest(REQ_READ_STAT_SUM); 
	hal.ptimer->wait_us(20);	
	
	RSdata = (Response_Sum & RS_FIELD_MASK) >> 24;
	if (1 == RSdata)
	{
		
	}
	else
	{
		
	}
//	sprintf(DebugMessage, "Status Summary: 0x%08X\r\n", Response_Sum);
//	hal.pusart->send_message((uint8_t *)DebugMessage);
	
	RSdata = (Response_Temp & RS_FIELD_MASK) >> 24;
	if (1 == RSdata)
	{
		Sampling_Temp = (Response_Temp & DATA_FIELD_MASK) >> 8;
		Result_Temp_Count++;
		Error_Status &= ~SCA3300_T_ERROR; 
	}
	else
	{
		Error_Status |= SCA3300_T_ERROR; 
		hal.pusart->send_message((uint8_t *)"read temp error!\r\n");
//		SCA3300_SendRequest(REQ_SW_RESET);
	}
	
	RSdata = (Response_Acc.x & RS_FIELD_MASK) >> 24;
	if (1 == RSdata) 
	{
		Sampling_Acc.x = (Response_Acc.x & DATA_FIELD_MASK) >> 8;
		Result_Acc_Count.x++;
		Error_Status &= ~SCA3300_X_ERROR;
	}
	else
	{
		Error_Status |= SCA3300_X_ERROR;
		hal.pusart->send_message((uint8_t *)"read ACC_X error!\r\n");
//		SCA3300_SendRequest(REQ_SW_RESET);
	}

	RSdata = (Response_Acc.y & RS_FIELD_MASK) >> 24;
	if (1 == RSdata)
	{
		Sampling_Acc.y = (Response_Acc.y & DATA_FIELD_MASK) >> 8;
		Result_Acc_Count.y++;
		Error_Status &= ~SCA3300_Y_ERROR;
	}
	else
	{
		Error_Status |= SCA3300_Y_ERROR;
		hal.pusart->send_message((uint8_t *)"read ACC_Y error!\r\n");
//		SCA3300_SendRequest(REQ_SW_RESET);
	}	

	RSdata = (Response_Acc.z & RS_FIELD_MASK) >> 24;
	if (1 == RSdata)
	{
		Sampling_Acc.z = (Response_Acc.z & DATA_FIELD_MASK) >> 8;
		Result_Acc_Count.z++;
		Error_Status &= ~SCA3300_Z_ERROR;
	}
	else
	{
		Error_Status |= SCA3300_Z_ERROR; 
		hal.pusart->send_message((uint8_t *)"read ACC_Z error!\r\n");
//		SCA3300_SendRequest(REQ_SW_RESET);
	}	
	
	Sampling_Acc_Sum.x += Sampling_Acc.x;
	Sampling_Acc_Sum.y += Sampling_Acc.y;
	Sampling_Acc_Sum.z += Sampling_Acc.z;
	Sampling_Temp_Sum  += Sampling_Temp; 
}

	uint32_t Response_Mode;
	uint8_t RSdata;
static void SCA3300_FunCheck(void)
{
	TestStatus StartupOK;
	uint32_t Response_StatSum;

	
	StartupOK = TRUE;
//----------------------------------------------------------------------------------------------
	// Check functionality
	SCA3300_SendRequest(REQ_READ_STAT_SUM);
	Response_StatSum = SCA3300_SendRequest(REQ_READ_STAT_SUM); // Read Status Summary register again to get (0x7DA04966)
	RSdata = (Response_StatSum & RS_FIELD_MASK) >> 24; // the correct status data from off-frame
	if (RSdata != 1) StartupOK = FALSE;
	if (StartupOK == FALSE)
	{		
		while (1)	
		{
			sprintf(DebugMessage, "SCA3300 Request Summary ERROR!! \r\nStatus Summary = 0x%X\r\n", Response_StatSum);
			hal.pusart->send_message((uint8_t *)DebugMessage);
			hal.ptimer->wait_ms(1000);
			Response_StatSum = SCA3300_SendRequest(REQ_READ_STAT_SUM); // Read Status Summary register again to get (0x7DA04966)
			RSdata = (Response_StatSum & RS_FIELD_MASK) >> 24; // the correct status data from off-frame
			if (RSdata == 1) break;
		}
	}

	SCA3300_SendRequest(REQ_CHANGE_MODE2); // First read temp once to get into desired
	Response_Mode = SCA3300_SendRequest(REQ_CHANGE_MODE2); // Read Status Summary register again to get (0x7DA04966)
	RSdata = (Response_Mode & RS_FIELD_MASK) >> 24; // the correct status data from off-frame
	if (RSdata != 1) StartupOK = FALSE;
	if (StartupOK == FALSE)
	{		
//		while (1)	
//		{
			sprintf(DebugMessage, "SCA3300 Change Mode Error!! \r\nMode Response = 0x%X\r\n", Response_Mode);
			hal.pusart->send_message((uint8_t *)DebugMessage);
//			hal.ptimer->wait_ms(1000);
//			Response_Mode = SCA3300_SendRequest(REQ_CHANGE_MODE2); // Read Status Summary register again to get (0x7DA04966)
//			RSdata = (Response_Mode & RS_FIELD_MASK) >> 24; // the correct status data from off-frame
//			if (RSdata == 1) break;
//		}
	}
	
// measurement cycle in off-frame protocol
	SCA3300_SendRequest(REQ_READ_STAT_SUM); 
}

#if 0	
static void sCC2230_Process(void)
{
	if (DataError)
	{
		// In case of error read status registers
		SysTick->CTRL = 0; // Disable systick interrupt to avoid race condition with the
		// interrupt routine
		// Clear the data that is coming from previous frame, request status summary
		SendRequest(REQ_READ_STAT_SUM);
		Response_StatSum = SendRequest(REQ_READ_RATE_STAT1);
		Response_RateStat1 = SendRequest(REQ_READ_RATE_STAT2);
		Response_RateStat2 = SendRequest(REQ_READ_COM_STAT);
	#if PRODUCT_TYPE == SCC
		Response_ComStat1 = SendRequest(REQ_READ_ACC_STAT);
		// Request temperature data to get the measurement loop to
		// continue correctly after reading the status registers
		Response_AccStat = SendRequest(REQ_READ_TEMP);
		sprintf(Buffer, "%08X %08X %08X %08X %08X\r\n", Response_StatSum, Response_RateStat1, Response_RateStat2, Response_AccStat, Response_ComStat1);
	#else
		Response_ComStat1 = SendRequest(REQ_READ_TEMP);
		sprintf(Buffer, "%08X %08X %08X %08X\r\n", Response_StatSum, Response_RateStat1, Response_RateStat2, Response_ComStat1);
	#endif
		
			/* Enable SysTick IRQ and SysTick Timer */
		SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
						 SysTick_CTRL_TICKINT_Msk   |
						 SysTick_CTRL_ENABLE_Msk;  
												
		Print_String(Buffer);
		DataError = FALSE;
	}
	else if (DataReady)
	{
		#if PRODUCT_TYPE == SCC
		sprintf(Buffer, "%5d %5d %5d %5d %5d\r\n", Result_Rate / SCC2230_SAMPLE_TIMES,
				Result_Acc_X / SCC2230_SAMPLE_TIMES, Result_Acc_Y / SCC2230_SAMPLE_TIMES, 
				Result_Acc_Z / SCC2230_SAMPLE_TIMES, Result_Temp / SCC2230_SAMPLE_TIMES);

		Result_Rate_Send = Result_Rate / SCC2230_SAMPLE_TIMES;
		Result_Acc_X_Send = Result_Acc_X / SCC2230_SAMPLE_TIMES;
		Result_Acc_Y_Send = Result_Acc_Y / SCC2230_SAMPLE_TIMES;
		Result_Acc_Z_Send = Result_Acc_Z / SCC2230_SAMPLE_TIMES;

		TxMessage.Data[0] = (uint8_t)Result_Rate_Send;
		TxMessage.Data[1] = (uint8_t)(Result_Rate_Send >> 8);
//		TxMessage.Data[2] = (uint8_t)Result_Acc_X_Send;
//		TxMessage.Data[3] = (uint8_t)(Result_Acc_X_Send >> 8);
//		TxMessage.Data[4] = (uint8_t)Result_Acc_Y_Send;
//		TxMessage.Data[5] = (uint8_t)(Result_Acc_Y_Send >> 8);
//		TxMessage.Data[6] = (uint8_t)Result_Acc_Z_Send;
//		TxMessage.Data[7] = (uint8_t)(Result_Acc_Z_Send >> 8);	
		TxMessage.Data[2] = 0;
		TxMessage.Data[3] = 0;
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = FIRMWARE_VERSION;
		#else
		sprintf(Buffer, "%5d %5d\r\n", Result_Rate / SCC2230_SAMPLE_TIMES, Result_Temp / SCC2230_SAMPLE_TIMES);
		#endif
		Print_String(Buffer);
		CAN_Transmit(CAN, &TxMessage);
		DataReady = FALSE;
	}
}

void sCC2230_StatusRegisterCheck(void)
{
	// In case of error read status registers
	SysTick->CTRL = 0; // Disable systick interrupt to avoid race condition with the
	// interrupt routine
	// Clear the data that is coming from previous frame, request status summary
	SendRequest(REQ_READ_STAT_SUM);
	Response_StatSum = SendRequest(REQ_READ_RATE_STAT1);
	Response_RateStat1 = SendRequest(REQ_READ_RATE_STAT2);
	Response_RateStat2 = SendRequest(REQ_READ_COM_STAT);
#if PRODUCT_TYPE == SCC
	Response_ComStat1 = SendRequest(REQ_READ_ACC_STAT);
	// Request temperature data to get the measurement loop to
	// continue correctly after reading the status registers
	Response_AccStat = SendRequest(REQ_READ_TEMP);
	//sprintf(Buffer, "%08X %08X %08X %08X %08X\r\n", Response_StatSum, Response_RateStat1, Response_RateStat2, Response_AccStat, Response_ComStat1);
#else
	Response_ComStat1 = SendRequest(REQ_READ_TEMP);
	//sprintf(Buffer, "%08X %08X %08X %08X\r\n", Response_StatSum, Response_RateStat1, Response_RateStat2, Response_ComStat1);
#endif
//	sprintf(Buffer, "Status Sum = 0x%X\r\n", Response_StatSum);
//	Print_String(Buffer);
//	sprintf(Buffer, "Rate Status 1 = 0x%X\r\n", Response_RateStat1);
//	Print_String(Buffer);
//	sprintf(Buffer, "Rate Status 2 = 0x%X\r\n", Response_RateStat2);
//	Print_String(Buffer);
//	sprintf(Buffer, "Acceleration Status = 0x%X\r\n", Response_AccStat);
//	Print_String(Buffer);
//	sprintf(Buffer, "Common Status 1 = 0x%X\r\n", Response_ComStat1);
//	Print_String(Buffer);

	TxMessage.Data[0] = 0x24;
	TxMessage.Data[1] = 0x24;
	TxMessage.Data[2] = 0x53;
	TxMessage.Data[3] = 0x59;
	TxMessage.Data[4] = (uint8_t)(Response_StatSum >> 24);
	TxMessage.Data[5] = (uint8_t)(Response_StatSum >> 16);
	TxMessage.Data[6] = (uint8_t)(Response_StatSum >> 8);
	TxMessage.Data[7] = (uint8_t)Response_StatSum;	
	CAN_Transmit(CAN, &TxMessage);
	Wait_ms(1);
	
	TxMessage.Data[0] = 0x24;
	TxMessage.Data[1] = 0x24;
	TxMessage.Data[2] = 0x53;
	TxMessage.Data[3] = 0x5A;
	TxMessage.Data[4] = (uint8_t)(Response_RateStat1 >> 24);
	TxMessage.Data[5] = (uint8_t)(Response_RateStat1 >> 16);
	TxMessage.Data[6] = (uint8_t)(Response_RateStat1 >> 8);
	TxMessage.Data[7] = (uint8_t)Response_RateStat1;	
	CAN_Transmit(CAN, &TxMessage);
	Wait_ms(1);
	
	TxMessage.Data[0] = 0x24;
	TxMessage.Data[1] = 0x24;
	TxMessage.Data[2] = 0x53;
	TxMessage.Data[3] = 0x5B;
	TxMessage.Data[4] = (uint8_t)(Response_RateStat2 >> 24);
	TxMessage.Data[5] = (uint8_t)(Response_RateStat2 >> 16);
	TxMessage.Data[6] = (uint8_t)(Response_RateStat2 >> 8);
	TxMessage.Data[7] = (uint8_t)Response_RateStat2;	
	CAN_Transmit(CAN, &TxMessage);
	Wait_ms(1);
	
	TxMessage.Data[0] = 0x24;
	TxMessage.Data[1] = 0x24;
	TxMessage.Data[2] = 0x53;
	TxMessage.Data[3] = 0x5C;
	TxMessage.Data[4] = (uint8_t)(Response_ComStat1 >> 24);
	TxMessage.Data[5] = (uint8_t)(Response_ComStat1 >> 16);
	TxMessage.Data[6] = (uint8_t)(Response_ComStat1 >> 8);
	TxMessage.Data[7] = (uint8_t)Response_ComStat1;	
	CAN_Transmit(CAN, &TxMessage);
	Wait_ms(1);
	
	TxMessage.Data[0] = 0x24;
	TxMessage.Data[1] = 0x24;
	TxMessage.Data[2] = 0x53;
	TxMessage.Data[3] = 0x5D;
	TxMessage.Data[4] = (uint8_t)(Response_AccStat >> 24);
	TxMessage.Data[5] = (uint8_t)(Response_AccStat >> 16);
	TxMessage.Data[6] = (uint8_t)(Response_AccStat >> 8);
	TxMessage.Data[7] = (uint8_t)Response_AccStat;	
	CAN_Transmit(CAN, &TxMessage);
	Wait_ms(1);
	
		/* Enable SysTick IRQ and SysTick Timer */
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
					 SysTick_CTRL_TICKINT_Msk   |
					 SysTick_CTRL_ENABLE_Msk;  				
}

void sCC2230_ReadAndProcessData(void)
{
	LoopCount = SCC2230_SAMPLE_TIMES + 1;
	while(--LoopCount)
	{
		// Read temperature & accelerations. Note: interleaved reading due to off-frame protocol
		Response_Temp = SendRequest(REQ_READ_ACC_X);
		Response_Acc_X = SendRequest(REQ_READ_ACC_Y);
		Response_Acc_Y = SendRequest(REQ_READ_ACC_Z);
		Response_Acc_Z = SendRequest(REQ_READ_TEMP);

		// Check CRC if necessary
		// if (CalculateCRC(Response_Rate) != (Response_Rate & CRC_FIELD_MASK)) DataError = true;

		// Handle accelerometer data
		Acc_X = (Response_Acc_X & DATA_FIELD_MASK) >> 8;
		RSdata = (Response_Acc_X & RS_FIELD_MASK) >> 24;
		if (RSdata != 1) DataError = TRUE;
		Sum_X += Acc_X;
		Acc_Y = (Response_Acc_Y & DATA_FIELD_MASK) >> 8;
		RSdata = (Response_Acc_Y & RS_FIELD_MASK) >> 24;
		if (RSdata != 1) DataError = TRUE;
		Sum_Y += Acc_Y;
		Acc_Z = (Response_Acc_Z & DATA_FIELD_MASK) >> 8;
		RSdata = (Response_Acc_Z & RS_FIELD_MASK) >> 24;
		if (RSdata != 1) DataError = TRUE;
		Sum_Z += Acc_Z;
		// Handle temperature data
		Temp = (Response_Temp & DATA_FIELD_MASK) >> 8;
		RSdata = (Response_Temp & RS_FIELD_MASK) >> 24;
		if (RSdata != 1) DataError = TRUE;
		Sum_Temp += Temp;
	}
	
	Result_Acc_X = Sum_X;
	Sum_X = 0;
	Result_Acc_Y = Sum_Y;
	Sum_Y = 0;
	Result_Acc_Z = Sum_Z;
	Sum_Z = 0;
	Result_Temp = Sum_Temp;
	Sum_Temp = 0;

	DataReady = TRUE;
}


#endif






