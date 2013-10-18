/***********************************************************************************************************
 * controlSCI.h
 *
 * Controls the serial data transfer between device and desktop application via the TI SCI port
 *
 * Created on: 25 June 2013
 *      Author: chris.barlow
 * *********************************************************************************************************/

#ifndef CONTROLSCI_H_
#define CONTROLSCI_H_

#define SEQ_TX_CHUNK_SIZE		(10)
#define SEQ_TX_CHUNK_SPACING	(10)

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

/***********************************************************************************************************
 * Initialisation - called once when the device boots, before the scheduler starts.
 * *********************************************************************************************************/
void controlSCI_init(void);


/***********************************************************************************************************
 * Update function - called periodically from scheduler
 * *********************************************************************************************************/
void controlSCI_update(void);



#endif /* CONTROLSCI_H_ */
