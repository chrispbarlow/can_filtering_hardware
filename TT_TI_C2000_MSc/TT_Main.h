/*
 * TT_Main.h
 *
 *  Created on: 8 Feb 2013
 *      Author: chris.barlow
 */

#ifndef TT_MAIN_H_
#define TT_MAIN_H_

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "scheduler/scheduler.h"

#define EVERY_TICK (1)

// Prototype statements for functions found within this file.
interrupt void cpu_timer0_isr(void);
void cpuInit(void);
void gpioInit(void);
void timerStart(void);


#endif /* TT_MAIN_H_ */
