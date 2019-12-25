#include "timer.h"

/************************************************************************/
// Wait us, depends on clock frequency so adjust accordingly
void Wait_us(uint16_t us)
{
	uint16_t a = us;
	uint32_t c;
	for (uint16_t i=0; i<a; i++)
	{
		for (uint8_t j=0; j<100; j++)
		{
			c++;
		}
	}
}

// Wait ms
void Wait_ms(uint16_t ms)
{
	uint16_t Count;
	for (Count = 0; Count < ms; Count++) Wait_us(1000);
}



























