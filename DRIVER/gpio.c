#include "gpio.h"

/****************************************************************/  
void IS203_Key_Init(void);


/****************************************************************/  
void Gpio_Init(void)
{
	IS203_Key_Init();
}

void IS203_Key_Init(void)   //PB0 PB1 PB2
{
	GPIO_InitTypeDef GPIO_Initure;
	
	KEY_CLOCK_ENABLE();              

	GPIO_Initure.Pin = KEY_AUTO_PIN;//|GPIO_PIN_1|GPIO_PIN_2;
	//GPIO_Initure.Mode = GPIO_MODE_INPUT;    
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;  	
	GPIO_Initure.Pull = GPIO_PULLUP;              
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;    
	HAL_GPIO_Init(KEY_PORT,&GPIO_Initure);
	
	HAL_GPIO_WritePin(KEY_PORT,GPIO_PIN_0,GPIO_PIN_SET);	//set KEY_AUTO
	HAL_GPIO_WritePin(KEY_PORT,GPIO_PIN_1,GPIO_PIN_SET);	//set KEY_UP
	HAL_GPIO_WritePin(KEY_PORT,GPIO_PIN_2,GPIO_PIN_SET);	//set KEY_DOWN
}


/*
uint8_t Key_Scan(void)	
{
	uint8_t key_value = 0;
	if (!HAL_GPIO_ReadPin(KEY_PORT, KEY_AUTO_PIN))
	{
		key_value |= 0x01;
	}
//	if (!HAL_GPIO_ReadPin(KEY_PORT, KEY_UP_PIN))
//	{
//		key_value |= 0x02;
//	}
//	if (!HAL_GPIO_ReadPin(KEY_PORT, KEY_DOWN_PIN))
//	{
//		key_value |= 0x04;
//	}
		
	return key_value;
}

*/






