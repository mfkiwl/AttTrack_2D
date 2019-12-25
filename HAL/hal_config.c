#include "hal_config.h"
#include "systick.h"
#include "gpio.h"
#include "fdcan.h"
#include "spi.h"
#include "usart.h"
#include "timer.h"
/**********************************************************************************************************/
SYSTICKDriver_st hal_systick = {
	.init = Systick_Init,     
	.millis = System_Millis,
	.micros = System_Micros,
};

/**********************************************************************************************************/
GPIODriver_st hal_gpio = {
	.init = Gpio_Init,
};

/**********************************************************************************************************/
FDCANDriver_st hal_fdcan = {
	.init = Fdcan_Init,
	.send_message = Fdcan_SendMessage,
	.ext_send_message = FdcanExt_SendMessage, 
	.read_message = Fdcan_ReadMessage,
};

/**********************************************************************************************************/
SPIDriver_st hal_spi = {
	.init = Spi_Init,
	.send_message = Spi_SendMessage,
};

/**********************************************************************************************************/
USARTDriver_st hal_usart = {
	.init = Usart_Init,
	.send_message = Usart_SendMessage,
};

/**********************************************************************************************************/
TIMERDriver_st hal_timer = {
	.wait_us = Wait_us,
	.wait_ms = Wait_ms,
};


//------------------End of File----------------------------
