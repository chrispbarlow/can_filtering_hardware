/*
 * handleCAN - updates the CAN mailbox state machine.
 *
 * MUST BE EXECUTED AT LEAST TWICE AS FREQUENTLY AS ANY CAN Tx AND Rx TASKS.
 *
 *  Created on: 11 Feb 2013
 *      Author: chris.barlow
 */
#include "handleCAN.h"

void handleCAN_init(void)
{

}

void handleCAN_update(void)
{
	Uint16 i;

	for(i=0;i<2;i++){
		updateMailboxes(CANPORT_A);
	}
}
