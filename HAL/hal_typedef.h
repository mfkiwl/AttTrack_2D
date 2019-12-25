#ifndef _HAL_TYPEDEF_H__
#define _HAL_TYPEDEF_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif
 
	
#ifndef NULL
#if 1
  #define NULL   (void *) 0
#else
  #define NULL 0
#endif
#endif

/****************************************************************/  
typedef struct {
  void (*init)(void);
  uint32_t (*millis)(void);
  uint32_t (*micros)(void);
}SYSTICKDriver_st;

/****************************************************************/
typedef struct {
  void (*init)(void);
}GPIODriver_st;

/****************************************************************/
typedef struct {
	void (*init)(void); 
	void (*send_message)(uint32_t id, uint8_t* buff, uint16_t length); 
	void (*ext_send_message)(uint32_t id, uint8_t* buff, uint16_t length); 
	void (*read_message)(uint8_t* buff); 
}FDCANDriver_st;

/****************************************************************/
typedef struct {
	void (*init)(void); 
	uint8_t (*send_message)(uint8_t id, uint8_t* prequest, uint8_t* presponse, uint16_t length); 
}SPIDriver_st;

/****************************************************************/
typedef struct {
  void (*init)(void);
  int16_t (*available)(void); 
  int16_t (*read)(void); 
  void (*flush)(void);
  void (*send_message)(uint8_t *pstr);
}USARTDriver_st;

/****************************************************************/
typedef struct {
  void (*wait_us)(uint16_t us);
  void (*wait_ms)(uint16_t ms);
}TIMERDriver_st;

/****************************************************************/
typedef struct {
  SYSTICKDriver_st  *psystick;
  GPIODriver_st     *pgpio;
  FDCANDriver_st    *pfdcan;
  SPIDriver_st      *pspi;
  USARTDriver_st    *pusart;
  TIMERDriver_st    *ptimer;
}HALDriver_st;


#ifdef __cplusplus
} // extern "C"
#endif

#endif

//------------------End of File----------------------------
