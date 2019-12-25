#ifndef __HC_TYPEDEF_H__
#define __HC_TYPEDEF_H__


//#include <stdio.h>
#include "stm32h7xx.h"
#include "core_cm7.h"
#include "stm32h7xx_hal.h"


#ifdef __cplusplus
extern "C" {
#endif           /*__cplusplus*/
	
typedef char        HC_INT8;
typedef short       HC_INT16;
typedef int         HC_INT32;
typedef long long   HC_INT64;

typedef unsigned char       HC_UINT8;
typedef unsigned short      HC_UINT16;
typedef unsigned int        HC_UINT32;
typedef unsigned long long  HC_UINT64;

typedef void HC_VOID;
typedef int  HC_BOOL;

typedef unsigned long HC_ULONG;
typedef double HC_DOUBLE;
typedef float  HC_FLOAT;
typedef long HC_LONG;

#define HC_OK    0
#define HC_ERR  -1

#define HC_TRUE  1
#define HC_FALSE 0

#define HC_IN_PROCESS   1

#define HC_ON   1
#define HC_OFF  0

#define HC_EOF  1

/*HC_OUT, HC_IN, HC_IN_OUT used by function parmeter */
#define HC_OUT
#define HC_IN
#define HC_IN_OUT

/* lw: according to libc stddef.h  */
#ifdef __cplusplus
#define HC_NULL 0
#else
#define HC_NULL ((void *)0)
#endif

#define ARRAY_SIZE(x) ((unsigned int)(sizeof(x)/sizeof((x)[0])))

#define ALIGN_UP(num, size) (((num) + (size - 1)) & (~(size -1)))
#define ALIGN_DOWN(num,size) ((num) & (~(size -1)))


/* convert parmeter x to a string */
#define str(x)  #x

/* convert the value of macro to string  */
#define xstr(x) str(x)

#ifdef __cplutplus
}
#endif				/*__cplusplus*/
#endif

