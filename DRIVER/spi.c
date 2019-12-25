#include "spi.h"


#define AD7682_SPI_CS_PIN           GPIO_PIN_4
#define AD7682_SPI_SCK_PIN					GPIO_PIN_5   //GPIOA
#define AD7682_SPI_MISO_PIN         GPIO_PIN_6
#define AD7682_SPI_MOSI_PIN         GPIO_PIN_7

#define SCA3300_SPI_CS_PIN			GPIO_PIN_12
#define SCA3300_SPI_SCK_PIN			GPIO_PIN_13	 //GPIOB
#define SCA3300_SPI_MISO_PIN		GPIO_PIN_14
#define SCA3300_SPI_MOSI_PIN		GPIO_PIN_15

SPI_HandleTypeDef SPI1_Handler;  
SPI_HandleTypeDef SPI2_Handler; 
	

uint8_t SPI1_Init(void);
uint8_t SPI2_Init(void);
/********************** Interface ********************************/
void Spi_Init(void)
{
	SPI1_Init();   //AD7682
	SPI2_Init();   //SCA3300
}

uint8_t Spi_SendMessage(uint8_t id, uint8_t *pRequest, uint8_t *pResponse, uint16_t Size)
{
	if (id > 2)
		return 1;
	
	switch (id)
	{
		case 1:
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);   //chip select 
				while (HAL_SPI_GetState(&SPI1_Handler) == HAL_SPI_STATE_BUSY_TX_RX);
				HAL_SPI_TransmitReceive(&SPI1_Handler, pRequest, pResponse, Size, 1000);
				//HAL_SPI_Transmit(&SPI2_Handler, Request_buff, 4, 1000);
				while (HAL_SPI_GetState(&SPI1_Handler) == HAL_SPI_STATE_BUSY_TX_RX);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);  
			break;
		case 2:
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);   //chip select 
				while (HAL_SPI_GetState(&SPI2_Handler) == HAL_SPI_STATE_BUSY_TX_RX);
				HAL_SPI_TransmitReceive(&SPI2_Handler, pRequest, pResponse, Size, 1000);
				//HAL_SPI_Transmit(&SPI2_Handler, Request_buff, 4, 1000);
				while (HAL_SPI_GetState(&SPI2_Handler) == HAL_SPI_STATE_BUSY_TX_RX);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);  
			break;
		default:
			break;
	}
	return 0;
}

/****************************************************************/
uint8_t SPI1_Init(void)
{
	HAL_SPI_DeInit(&SPI1_Handler);
	SPI1_Handler.Instance = SPI1;
	SPI1_Handler.Init.Mode = SPI_MODE_MASTER;
	SPI1_Handler.Init.Direction = SPI_DIRECTION_2LINES;
	SPI1_Handler.Init.DataSize = SPI_DATASIZE_8BIT;
	SPI1_Handler.Init.CLKPolarity = SPI_POLARITY_LOW;
	SPI1_Handler.Init.CLKPhase = SPI_PHASE_1EDGE;
	SPI1_Handler.Init.NSS = SPI_NSS_SOFT;
	SPI1_Handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	SPI1_Handler.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPI1_Handler.Init.TIMode = SPI_TIMODE_DISABLE;
	SPI1_Handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	SPI1_Handler.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&SPI1_Handler) != HAL_OK)
	{
		//Error_Handler(__FILE__, __LINE__);
		return 1;
	}
	return 0;
}    

uint8_t SPI2_Init(void)
{
	HAL_SPI_DeInit(&SPI2_Handler);
	SPI2_Handler.Instance = SPI2;
	SPI2_Handler.Init.Mode = SPI_MODE_MASTER;
	SPI2_Handler.Init.Direction = SPI_DIRECTION_2LINES;
	SPI2_Handler.Init.DataSize = SPI_DATASIZE_8BIT;
	SPI2_Handler.Init.CLKPolarity = SPI_POLARITY_LOW; 
	SPI2_Handler.Init.CLKPhase = SPI_PHASE_1EDGE;
	SPI2_Handler.Init.NSS = SPI_NSS_SOFT;
	SPI2_Handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;  // main clock is 400MHZ
	SPI2_Handler.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPI2_Handler.Init.TIMode = SPI_TIMODE_DISABLE;
	SPI2_Handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	SPI2_Handler.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&SPI2_Handler) != HAL_OK)
	{
		//Error_Handler(__FILE__, __LINE__);
		return 1;
	}
	return 0;
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	if(hspi->Instance == SPI1)
	{
		/* USER CODE BEGIN SPI1_MspInit 0 */
		/* USER CODE END SPI1_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_SPI1_CLK_ENABLE();
	  
		/**SPI1 GPIO Configuration    
		PA5     ------> SPI1_SCK
		PA6     ------> SPI1_MISO
		PA7     ------> SPI1_MOSI 
		*/	
		GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;// GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	    /* USER CODE BEGIN SPI1_MspInit 1 */
		GPIO_InitStruct.Pin = GPIO_PIN_4;     	//PA4 ------> SPI1_CS 
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; //GPIO_SPEED_FREQ_MEDIUM   GPIO_SPEED_FREQ_HIGH  GPIO_SPEED_FREQ_VERY_HIGH  
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		__HAL_RCC_GPIOA_CLK_ENABLE();  
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);	//PA4ÖÃ1
		/* USER CODE END SPI1_MspInit 1 */
	}
	else if (hspi->Instance == SPI2)
	{
		/* USER CODE BEGIN SPI2_MspInit 0 */
		/* USER CODE END SPI2_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_SPI2_CLK_ENABLE();
	  
		/**SPI2 GPIO Configuration    
		PB13     ------> SPI2_SCK
		PB14     ------> SPI2_MISO
		PB15     ------> SPI2_MOSI 
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;//GPIO_PULLUP;//GPIO_NOPULL;   
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		/* USER CODE BEGIN SPI2_MspInit 1 */
		GPIO_InitStruct.Pin = GPIO_PIN_12;     	//PB12 ------> SPI2_CS 
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		__HAL_RCC_GPIOB_CLK_ENABLE();  
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);	//PB12ÖÃ1
		/* USER CODE END SPI2_MspInit 1 */
	}
}
 
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
	if (hspi->Instance == SPI1)
	{
		__HAL_RCC_SPI1_FORCE_RESET(); 
		__HAL_RCC_SPI1_RELEASE_RESET();
#if SPI1_RX0_INT_ENABLE   
		HAL_NVIC_DisableIRQ(SPI1_IRQn);
#endif
	}
	else if (hspi->Instance == SPI2)
	{
		__HAL_RCC_SPI2_FORCE_RESET(); 
		__HAL_RCC_SPI2_RELEASE_RESET();
#if SPI1_RX0_INT_ENABLE   
		HAL_NVIC_DisableIRQ(SPI2_IRQn);
#endif
	}
}







