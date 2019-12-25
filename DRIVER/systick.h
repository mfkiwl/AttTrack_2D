#ifndef _SYSTICK_H__
#define _SYSTICK_H__

#include <stdint.h>
#include "stm32h7xx.h"
#include "core_cm7.h"
#include "stm32h7xx_hal.h"


#ifdef __cplusplus
extern "C"{
#endif
	
//********************************************************************************
#define US_PER_MS                      1000
#define TICKS_PER_MICROSECOND          400    //64 count for a microsec
#define SYSTICK_RELOAD_VAL             399999
#define SYSTICK_INTERRUPTS_PER_SECOND  1000   //0.001s = 1 ms
	
typedef void (*ptrProFunc)(void);	
	
/** System elapsed time, in milliseconds */
extern volatile uint32_t systick_uptime_millis;

//********************************************************************************
/**
 *@brief Returns the system uptime, in milliseconds.
 */
static inline uint32_t systick_get_millicount(void) 
{
	return systick_uptime_millis;
}

/**
 *@brief Returns the current value of the SysTick counter.
 */
static inline uint32_t systick_get_tickcount(void) 
{
	return SysTick->VAL;
} 

/**
 *Returns time (in microseconds) since the beginning of program
 *execution.  On overflow, restarts at 0.
 *@see millis()
 */
static inline uint32_t System_Micros(void) 
{ 
	uint32_t ticks;
	uint32_t mili_count;

	ticks = systick_get_tickcount();
	mili_count = systick_get_millicount();
  
	return (mili_count * US_PER_MS) + (SysTick->LOAD + 1 - ticks) / TICKS_PER_MICROSECOND;
}

/**
 *Returns time (in milliseconds) since the beginning of program
 *execution. On overflow, restarts at 0.
 *@see micros()
 */
static inline uint32_t System_Millis(void) 
{
	return systick_get_millicount();
}


/*
//0,不支持os
//1,支持os
#define SYSTEM_SUPPORT_OS		0		//定义系统文件夹是否支持OS	
///////////////////////////////////////////////////////////////////////////////////
//定义一些常用的数据类型短关键字 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  
typedef const int16_t sc16;  
typedef const int8_t sc8;  

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  
typedef __I int16_t vsc16; 
typedef __I int8_t vsc8;   

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  
typedef const uint16_t uc16;  
typedef const uint8_t uc8; 

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  
typedef __I uint16_t vuc16; 
typedef __I uint8_t vuc8;  

#define ON	1
#define OFF	0
#define Write_Through() (*(__IO uint32_t*)0XE000EF9C=1UL<<2) //Cache透写模式
*/

void Systick_Init(void);
void Cache_Enable(void);                                    //使能STM32H7的L1-Cahce
void Stm32_Clock_Init(uint32_t plln,uint32_t pllm,uint32_t pllp,uint32_t pllq); //配置系统时钟
uint8_t Get_ICahceSta(void);//判断I_Cache是否打开
uint8_t Get_DCahceSta(void);//判断I_Dache是否打开



//#if defined(__clang__) //使用V6编译器(clang)
//void __attribute__((noinline)) WFI_SET(void);
//void __attribute__((noinline)) INTX_DISABLE(void);
//void __attribute__((noinline)) INTX_ENABLE(void);
//void __attribute__((noinline)) MSR_MSP(u32 addr);
//#elif defined (__CC_ARM)    //使用V5编译器(ARMCC)
////以下为汇编函数
//void WFI_SET(void);		//执行WFI指令
//void INTX_DISABLE(void);//关闭所有中断
//void INTX_ENABLE(void);	//开启所有中断
//void MSR_MSP(u32 addr);	//设置堆栈地址 
//#endif


#ifdef __cplusplus
} // extern "C"
#endif

#endif

//------------------End of File----------------------------

