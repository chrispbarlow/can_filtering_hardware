/***********************************************************************************************************
 *  receiveCAN.h
 *  	checks the status of mailboxes. When a message is pending, the data is read
 *  	and the dynamic filter mechanism updates the mailbox to the next valid CAN ID
 *
 *  Created on: 19 Jun 2013
 *      Author: chris.barlow
 * *********************************************************************************************************/

#ifndef REVCEIVECAN_H_
#define REVCEIVECAN_H_

#include "../../Lib/CAN/CAN.h"


/***********************************************************************************************************
 * Initialisation - called once when the device boots, before the scheduler starts.
 * *********************************************************************************************************/
void receiveCAN_init(void);


/***********************************************************************************************************
 * Update function - called periodically from scheduler
 * *********************************************************************************************************/
void receiveCAN_update(void);

#endif /* REVCEIVECAN_H_ */
