#ifndef __TIMER_H__
#define __TIMER_H__


//#include "core_cm7.h"
#include <stdint.h>	


#ifdef __cplusplus
extern "C" {
#endif           /*__cplusplus*/
	

void Wait_us(uint16_t us);
void Wait_ms(uint16_t ms);

#ifdef __cplutplus
}
#endif				/*__cplusplus*/
#endif

