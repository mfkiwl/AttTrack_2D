#include "hal.h"

/**********************************************************************************************************/
HALDriver_st hal = {
  .psystick  = &hal_systick,
  .pgpio     = &hal_gpio,
  .pfdcan    = &hal_fdcan,
  .pspi      = &hal_spi,
  .pusart    = &hal_usart,
  .ptimer    = &hal_timer
};

/**********************************************************************************************************/
void Board_Hardware_Init(void)	//Self Definite Hal layer
{
	if (hal.psystick != NULL)
		hal.psystick->init();
	else
		while (1);
  
	if (hal.pgpio != NULL)
		hal.pgpio->init();
	else
		while (1);
  
	if (hal.pfdcan != NULL)
		hal.pfdcan->init();
	else
		while (1);
  
	if (hal.pspi != NULL)
		hal.pspi->init();
	else
		while (1);
	
	if (hal.pusart != NULL)
		hal.pusart->init();
	else
		while (1);
}

//------------------End of File----------------------------
