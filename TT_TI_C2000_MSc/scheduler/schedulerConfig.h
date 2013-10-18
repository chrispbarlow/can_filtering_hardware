/*
 * schedule.h
 *
 *  Created on: 8 Feb 2013
 *      Author: chris.barlow
 */

#ifndef SCHEDULE_H_
#define SCHEDULE_H_

#define TICK_PERIOD_us (500)

#define Sched_Config_HybridTasks TRUE

#include "../Tasks/handleCAN/handleCAN.h"
#include "../Tasks/receiveCAN/receiveCAN.h"
#include "../Tasks/controlSCI/controlSCI.h"


/* Function pointer for task array */
typedef void (*task_function_t)(void);

typedef enum{IN_SCHEDULER, IN_ISR} hybridOptions_t;

/* Task properties */
typedef struct
{
	task_function_t 	task_function;			/* function pointer */
	int  				task_period;			/* period in ticks */
	int  				task_delay;				/* initial offset in ticks */
	hybridOptions_t		task_hybridControl;		/* execute from ISR */
} task_t;

extern volatile task_t Tasks[];

extern const unsigned int tasksInSchedule;

void Tasks_Init(void);

#endif /* SCHEDULE_H_ */
