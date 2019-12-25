#include "hc_type.h" 
#include "hal.h"
#include "scheduler.h"
#include "tasks.h"
#include "fastloop.h"
#include "system.h"

/************************************************/
/*
  Scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called and the maximum 
  time they are expected to take (in microseconds)
*/
static const task_property_st _scheduler_tasks[] = {
  { run_100hz_task, FRE_100HZ, 50 },      // function name,  interval_ticks,  max_time_micros
};

/**************************************************************************************/
/*
  Board hardware and system tasks initialization
*/
void Setup(void)
{
	// Initializes the board hardware 
	Board_Hardware_Init();

	// Initializes sensors and parameters
	system_init();

	// Initializes the tasks
	task_init(_scheduler_tasks, (sizeof(_scheduler_tasks)/sizeof(_scheduler_tasks[0])));

	// Setting the base loop callback function
	set_task_base_loop_callback(fast_loop);
}

/***************************************************************************************/
/* 
  Top-level logic,the main function interface
*/
int main(void)
{
	// only do once
  Setup();	
	
	while(1)
	{	
		do_tasks();
	}
}






