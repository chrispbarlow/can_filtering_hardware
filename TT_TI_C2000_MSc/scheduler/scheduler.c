/*
 * scheduler.c
 *
 *  Created on: 11 Feb 2013
 *      Author: chris.barlow
 */
#include "scheduler.h"

static Uint16 interruptCount = 0;
static Uint16 interruptNesting = 0;


void scheduler_Init(void)
{
	/* Tasks with offset = 0 run on first tick, not before */
	Uint16 i;
	for(i = 0; i < tasksInSchedule; i++)
	{
		Tasks[i].task_delay++;
	}
}

void scheduler_onTick(void)
{
	Uint16 i;
	interruptCount++;

	for(i = 0; i < tasksInSchedule; i++)
	{
		Tasks[i].task_delay--;									/* task delay decremented until it reaches zero (time to run) */

#if (Sched_Config_HybridTasks == TRUE)
		if((Tasks[i].task_hybridControl == IN_ISR) && (Tasks[i].task_delay <= 0))
		{
			Tasks[i].task_delay = Tasks[i].task_period;			/* Reload task_delay */
			(*Tasks[i].task_function)();						/* Call task function */
		}
#endif
	}
}



void scheduler_taskDispatcher(void)
{
	Uint16 i;

	interruptDisable();
	while(interruptCount > 0) 	/* Safety net */
	{
		interruptCount--;

		for(i = 0; i < tasksInSchedule; i++)
		{
#if (Sched_Config_HybridTasks == TRUE)
			if(Tasks[i].task_hybridControl == IN_SCHEDULER)
#endif
			{
				if(Tasks[i].task_delay <= 0)
				{
					Tasks[i].task_delay = Tasks[i].task_period;		/* Reload task_delay */

					interruptEnable();
					(*Tasks[i].task_function)();							/* Call task function */
					interruptDisable();
				}
			}
		}
	}
	interruptEnable();

	goIdle(); /* Idle mode puts the processor to sleep, but allows waking on the timer interrupt */
}

void interruptEnable(void)
{
	if(interruptNesting > 0)
	{
		interruptNesting--;
	}

	if(interruptNesting == 0)
	{
		EINT;
	}
}

void interruptDisable(void)
{
	DINT;
	interruptNesting++;
}

void goIdle(void)
{
	asm(" IDLE");
}
