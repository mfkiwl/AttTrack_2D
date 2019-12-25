#ifndef _SCHEDULER_H__
#define _SCHEDULER_H__


#include <stdint.h>
#include "hal.h"


#ifdef __cplusplus
extern "C"{
#endif	
	
#define DEBUG_FAST_LOOP_TIME    1
#define DEBUG_100HZ_LOOP_TIME   1	
	
/********************************************************************************************************/  
/*config the main loop rate*/
#define MAIN_LOOP_100HZ    100   
#define MAIN_LOOP_200HZ    200   
#define MAIN_LOOP_400HZ    400   
#define MAIN_LOOP_1000HZ   1000 
  
#define MAIN_LOOP_RATE     MAIN_LOOP_1000HZ

#if (MAIN_LOOP_RATE == MAIN_LOOP_100HZ)  
/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in microseconds)
 */
 #define MAIN_LOOP_MICROS     10000
 #define MAIN_LOOP_SECONDS    0.01
 #define MAIN_LOOP_THRESHOLD  0.1f
  
 //uint 10ms
 #define FRE_100HZ         1     // 10ms
 #define FRE_50HZ          2     // 20ms
 #define FRE_25HZ          4     // 40ms
 #define FRE_10HZ          10    // 100ms
 #define FRE_5HZ           20    // 200ms
 #define FRE_4HZ           25    // 250ms
 #define FRE_3HZ           33    // 333.3ms
 #define FRE_2HZ           50    // 500ms
 #define FRE_1HZ           100   // 1000ms  ----->1S
 #define FRE_05HZ          200   // 2000ms  ----->2S
 #define FRE_025HZ         400   // 4000ms  ----->4S
 #define FRE_02HZ          500   // 5000ms  ----->5S
 #define FRE_01HZ          1000  // 10000ms ----->10S

#elif (MAIN_LOOP_RATE == MAIN_LOOP_200HZ) 
/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 5ms units) and the maximum time they are expected to take (in microseconds)
 */
 
 #define MAIN_LOOP_MICROS     5000
 #define MAIN_LOOP_SECONDS    0.005
 #define MAIN_LOOP_THRESHOLD  0.05f
 
 //uint 5ms
 #define FRE_200HZ         1     // 5ms
 #define FRE_100HZ         2     // 10ms
 #define FRE_50HZ          4     // 20ms
 #define FRE_25HZ          8     // 40ms
 #define FRE_10HZ          20    // 100ms
 #define FRE_5HZ           40    // 200ms
 #define FRE_4HZ           50    // 250ms
 #define FRE_3HZ           67    // 333.3ms
 #define FRE_2HZ           100   // 500ms
 #define FRE_1HZ           200   // 1000ms  ----->1S
 #define FRE_05HZ          400   // 2000ms  ----->2S
 #define FRE_025HZ         800   // 4000ms  ----->4S
 #define FRE_02HZ          1000  // 5000ms  ----->5S
 #define FRE_01HZ          2000  // 10000ms ----->10S 
 
#elif (MAIN_LOOP_RATE == MAIN_LOOP_400HZ) 

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 2.5ms units) and the maximum time they are expected to take (in microseconds)
 */
 #define MAIN_LOOP_MICROS     2500
 #define MAIN_LOOP_SECONDS    0.0025f
 #define MAIN_LOOP_THRESHOLD  0.025f
 
 //uint 2.5ms
 #define FRE_400HZ         1     // 2.5ms
 #define FRE_200HZ         2     // 5ms
 #define FRE_100HZ         4     // 10ms
 #define FRE_50HZ          8     // 20ms
 #define FRE_25HZ          16    // 40ms
 #define FRE_20HZ          20    // 50ms
 #define FRE_10HZ          40    // 100ms
 #define FRE_5HZ           80    // 200ms
 #define FRE_4HZ           100   // 250ms
 #define FRE_3HZ           133   // 333.3ms
 #define FRE_2HZ           200   // 500ms
 #define FRE_1HZ           400   // 1000ms  ----->1S
 #define FRE_05HZ          800   // 2000ms  ----->2S
 #define FRE_025HZ         1600  // 4000ms  ----->4S
 #define FRE_02HZ          2000  // 5000ms  ----->5S
 #define FRE_01HZ          4000  // 10000ms ----->10S

#elif (MAIN_LOOP_RATE == MAIN_LOOP_1000HZ) 
/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 1ms units) and the maximum time they are expected to take (in microseconds)
 */
 #define MAIN_LOOP_MICROS     1000
 #define MAIN_LOOP_SECONDS    0.001
 #define MAIN_LOOP_THRESHOLD  0.01f
 
 //uint 1ms
 #define FRE_1000HZ        1     // 1ms
 #define FRE_500HZ         2     // 2ms
 #define FRE_250HZ         4     // 4ms
 #define FRE_200HZ         5     // 5ms
 #define FRE_125HZ         8     // 8ms
 #define FRE_100HZ         10    // 10ms
 #define FRE_50HZ          20    // 20ms
 #define FRE_40HZ          25    // 25ms
 #define FRE_25HZ          40    // 40ms
 #define FRE_20HZ          50    // 50ms
 #define FRE_10HZ          100   // 100ms
 #define FRE_5HZ           200   // 200ms
 #define FRE_4HZ           250   // 250ms
 #define FRE_3HZ           333   // 333.3ms
 #define FRE_2HZ           500   // 500ms
 #define FRE_1HZ           1000  // 1000ms  ----->1S
 #define FRE_05HZ          2000  // 2000ms  ----->2S
 #define FRE_025HZ         4000  // 4000ms  ----->4S
 #define FRE_02HZ          5000  // 5000ms  ----->5S
 #define FRE_01HZ          10000 // 10000ms ----->10S
 
#endif

/********************************************************************************************************/
/*
  A task scheduler for main loops

  Sketches should call scheduler.init() on startup, then call
  scheduler.tick() at regular intervals (typically every 10ms). 

  To run tasks use scheduler.run(), passing the amount of time that
  the scheduler is allowed to use before it must return
 */
typedef void (*task_fn_t)(void);

typedef struct 
{
  task_fn_t function;
  uint16_t interval_ticks;
  uint16_t max_time_micros;
}task_property_st;

typedef struct 
{
  // progmem list of tasks to run
  const task_property_st *tasks;
  
  // current running task, or -1 if none. Used to debug stuck tasks
  int8_t current_task;
  
  // number of tasks in _tasks list
  uint8_t num_tasks;

  // number of 'ticks' that have passed (number of times that
  // tick() has been called
  uint16_t tick_counter;

  // tick counter at the time we last ran each task
  uint16_t *last_run;

  // number of microseconds allowed for the current task
  uint32_t task_time_allowed;

  // the time in microseconds when the task started
  uint32_t task_time_started;

  // number of spare microseconds accumulated
  uint32_t spare_micros;

  // number of ticks that _spare_micros is counted over
  uint8_t spare_ticks;
  
}task_st;

/********************************************************************************************************/ 
static inline uint32_t scheduler_micros()
{
  return hal.psystick->micros();
}	

static inline uint32_t get_systick_deltaT(uint32_t microStart, uint32_t microEnd)
{
  if (microStart > microEnd){
    //overflow, devo correggere
    return (microEnd + (0xFFFFFFFF - microStart));
  }
  return (microEnd - microStart);
}

/********************************************************************************************************/  

// read something the size of a word
static inline uint16_t pgm_read_word(const void *s) {
	return *(const uint16_t *)s;
}

// read something the size of a pointer. This makes the menu code more
// portable
static inline uintptr_t pgm_read_pointer(const void *s) {
	return *(const uintptr_t *)s;
}

/********************************************************************************************************/
// initialise scheduler
void task_init(const task_property_st *tasks, uint8_t num_tasks);

//// return the number of microseconds available for the current task
uint16_t task_time_available_usec(void);

// return load average, as a number between 0 and 1. 1 means
// 100% load. Calculated from how much spare time we have at the
// end of a run()
float task_load_average(uint32_t tick_time_usec);

// Attach a callback to be called from the tasks handler.
// To detach a callback, call this function again with a null argument.
void set_task_base_loop_callback(void (*callback)(uint32_t param));

//tasks handler.
void do_tasks(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif

//------------------End of File----------------------------







