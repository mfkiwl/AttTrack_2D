#include "scheduler.h"
#include <stdlib.h>
#include <string.h>

/***********************************************************************/
// System Timers,Time in microseconds of base control loop
static uint32_t _base_loopTimer = 0;

// TaskTable
static task_st _taskTable;

/********************************************************************************************************/
// task base loop callback function
static void (*_task_base_loop_callback)(uint32_t param) = NULL;

// call when one tick has passed
static void _task_tick(void);

// run the tasks. Call this once per 'tick'. 
// time_available is the amount of time available to run 
// tasks in microseconds
static void _task_run(uint16_t time_available);

/***********************************************************************/
// initialize scheduler
void task_init(const task_property_st *tasks, uint8_t num_tasks)
{
	_taskTable.tasks = tasks;
	_taskTable.num_tasks = num_tasks;

	_taskTable.last_run = malloc((sizeof(uint16_t) * num_tasks));

	if (_taskTable.last_run != NULL)
	{
		memset(_taskTable.last_run, 0, (sizeof(_taskTable.last_run[0]) * _taskTable.num_tasks));
		_taskTable.tick_counter = 0;
	} 
	else 
	{
		while (1);
	}
}

/************************************************************************/
// Attach a callback to be called from the tasks handler.
// To detach a callback, call this function again with a null argument.
void set_task_base_loop_callback(void (*callback)(uint32_t param)) 
{
  _task_base_loop_callback = callback;
  _base_loopTimer = scheduler_micros();
}


/***********************************************************************/
//tasks handler.
#define SCH_TIMER_TEST    1
#if SCH_TIMER_TEST
uint32_t gT = 0;    //record main loop time
#endif
uint32_t time_available = 0;
void do_tasks(void)
{
  // Get the current system time
  uint32_t tnow = scheduler_micros();
  uint32_t main_loop_time = get_systick_deltaT(_base_loopTimer, tnow);
  
  if (main_loop_time >= MAIN_LOOP_MICROS)
  //if (main_loop_time >= 1000)
  { 
#if SCH_TIMER_TEST
    gT = main_loop_time;
#endif
    // Update base loop timer
    _base_loopTimer = tnow;
    
    // Execute the fast loop
    _task_base_loop_callback(main_loop_time);

    // tell the scheduler one tick has passed
    _task_tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until _task_tick() is called again
    time_available = (tnow + MAIN_LOOP_MICROS) - scheduler_micros();
    _task_run(time_available);   // use up left time
  }
}

/********************************************************************************************************/
// call when one tick has passed
void _task_tick(void)
{
  _taskTable.tick_counter++;
}

/********************************************************************************************************/
// run the tasks. Call this once per 'tick'. 
// time_available is the amount of time available to run 
// tasks in microseconds
void _task_run(uint16_t time_available)
{
  uint32_t run_started_usec = scheduler_micros();
  uint32_t now = run_started_usec;

  for (uint8_t i = 0; i < _taskTable.num_tasks; i++) 
  {
    uint16_t dt = _taskTable.tick_counter - _taskTable.last_run[i];
  
    uint16_t interval_ticks = pgm_read_word(&_taskTable.tasks[i].interval_ticks);
  
    if (dt >= interval_ticks) 
    {
      // this task is due to run. Do we have enough time to run it?
      _taskTable.task_time_allowed = pgm_read_word(&_taskTable.tasks[i].max_time_micros);

      if (dt >= (interval_ticks * 2)) 
	  {
        //
      }
      
      if (_taskTable.task_time_allowed <= time_available) 
      {
        // run it
        _taskTable.task_time_started = now;
        task_fn_t func = (task_fn_t)pgm_read_pointer(&_taskTable.tasks[i].function);
        _taskTable.current_task = i;
        func();
        _taskTable.current_task = -1;
        
        // record the tick counter when we ran. This drives
        // when we next run the event
        _taskTable.last_run[i] = _taskTable.tick_counter;
        
        // work out how long the event actually took
        now = scheduler_micros();
      
        uint32_t time_taken = now - _taskTable.task_time_started;
        
        if (time_taken > _taskTable.task_time_allowed) {
          //
        }
        
        if (time_taken >= time_available) {
            goto update_spare_ticks;
        }
        time_available -= time_taken;
      }
    }
  }

  // update number of spare microseconds
  _taskTable.spare_micros += time_available;

update_spare_ticks:
  _taskTable.spare_ticks++;
  if (_taskTable.spare_ticks == 32) {
    _taskTable.spare_ticks /= 2;
    _taskTable.spare_micros /= 2;
  }
}

/********************************************************************************************************/
// return the number of microseconds available for the current task
uint16_t task_time_available_usec(void)
{
  uint32_t dt = scheduler_micros() - _taskTable.task_time_started;
  
  if (dt > _taskTable.task_time_allowed) {
    return 0;
  }
  
  return _taskTable.task_time_allowed - dt;
}

/********************************************************************************************************/
// return load average, as a number between 0 and 1. 1 means
// 100% load. Calculated from how much spare time we have at the
// end of a run()
float task_load_average(uint32_t tick_time_usec)
{
  if (_taskTable.spare_ticks == 0) {
    return 0.0f;
  }
  
  uint32_t used_time = tick_time_usec - (_taskTable.spare_micros/_taskTable.spare_ticks);
  
  return used_time / (float)tick_time_usec;
}

















