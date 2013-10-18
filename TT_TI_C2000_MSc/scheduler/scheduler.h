/*
 * scheduler.h
 *
 *  Created on: 11 Feb 2013
 *      Author: chris.barlow
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "../TT_Main.h"
#include "schedulerConfig.h"

void scheduler_Init(void);
void scheduler_onTick(void);
void scheduler_taskDispatcher(void);

void interruptEnable(void);
void interruptDisable(void);
void goIdle(void);


#endif /* SCHEDULER_H_ */
