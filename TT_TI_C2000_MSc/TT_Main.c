/*###########################################################################
*
* FILE:    TT_main.c
*
* TITLE:   C code entry point
*
**###########################################################################*/


#include "TT_Main.h"

#include "global.h"
#include "Lib/CAN/CAN.h"

Uint32 tickCount_G;


void main(void){
	tickCount_G = 0;

	cpuInit();
	gpioInit();
	scheduler_Init();
	CAN_Init(500,0);
	Tasks_Init();
	timerStart();

	while(EVERY_TICK){
		scheduler_taskDispatcher();
	}
}


interrupt void cpu_timer0_isr(void){
	// Acknowledge this interrupt to receive more interrupts from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

	scheduler_onTick();
}


void cpuInit(void){
	InitSysCtrl();

	/* Disable CPU interrupts */
	DINT;

	/* Initialize the PIE control registers to their default state.
	* The default state is all PIE interrupts disabled and flags
	* are cleared.
	* This function is found in the DSP2833x_PieCtrl.c file. */
	InitPieCtrl();

	/* Disable CPU interrupts and clear all CPU interrupt flags: */
	IER = 0x0000;
	IFR = 0x0000;

	/* Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR). */
	InitPieVectTable();

	/* Timer interrupt mapping */
	EALLOW;  /* This is needed to write to EALLOW protected registers */
	PieVectTable.TINT0 = &cpu_timer0_isr;
	EDIS;    /* This is needed to disable write to EALLOW protected registers */

	InitCpuTimers();

	/* Write the LPM code value */
	EALLOW;
	if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 1){ 	/* Only enter Idle mode when PLL is not in limp mode. */
		SysCtrlRegs.LPMCR0.bit.LPM = 0x0000;  	/* LPM mode = Idle - can be woken by timer interrupt */
	}

	/* Enable interrupts */
	EDIS;
}

void timerStart(void){
	// Configure CPU-Timer 0. 1 ms ticks.
	ConfigCpuTimer(&CpuTimer0, 150, TICK_PERIOD_us);

	// To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
	// of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in DSP2833x_CpuTimers.h), the
	// below settings must also be updated.

	CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0


	// Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
	// which is connected to CPU-Timer 1, and CPU int 14, which is connected
	// to CPU-Timer 2:
	IER |= M_INT1;
	IER |= M_INT13;
	IER |= M_INT14;

	// Enable TINT0 in the PIE: Group 1 interrupt 7
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

	// Enable global Interrupts and higher priority real-time debug events:
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM
}

void gpioInit(void){
	/* Task - specific GPIO initialisations should be in the relevant task_init() function. */

	/* GPIOs reserved for CAN:
		GPIO30 for CANRXA operation
		GPIO31 for CANTXA operation
		GPIO8 for CANTXB operation
		GPIO10 for CANRXB operation
	*/
}



//===========================================================================
// No more.
//===========================================================================
