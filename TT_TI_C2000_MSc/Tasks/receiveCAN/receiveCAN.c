/***********************************************************************************************************
 *  receiveCAN.c
 *  	checks the status of mailboxes. When a message is pending, the data is read
 *  	and the dynamic filter mechanism updates the mailbox to the next valid CAN ID
 *
 *  Created on: 19 Jun 2013
 *      Author: chris.barlow
 * *********************************************************************************************************/

#include "../../global.h"
#include "receiveCAN.h"
#include <stdio.h>
#include "../../CAN_Exchange/CAN_Rx_Filter_global.h"


/***********************************************************************************************************
 * Initialisation - called once when the device boots, before the scheduler starts.
 * *********************************************************************************************************/
void receiveCAN_init(void){
	/* mailboxes are configured in _update when first logging list is received from desktop app */
	updateSequenceRequired_G = INIT;
}

/***********************************************************************************************************
 * Update function - called periodically from scheduler
 * *********************************************************************************************************/
void receiveCAN_update(void){
	static Uint16 mailBox = 0;
	Uint16 sequenceIndex_received, sequenceIndex_new;

	/* updateSequenceRequired_G controls the sequence update mechanism when a new logging list is transmitted to the device */
	switch(updateSequenceRequired_G){
	/* Do nothing until first logging list arrival (RESET)*/
	default:
	case INIT:
		break;

	/* controlSCI will initiate RESET when new logging list is received */
	case RESET:

		/* Ensure all mailboxes are disabled */
		for(mailBox = 0; mailBox < NUM_MAILBOXES_MAX; mailBox++){
			disableMailbox(CANPORT_A, mailBox);
		}

		mailBox = 0;
		updateSequenceRequired_G = UPDATE;
		break;

	/* Set up mailboxes for initial filter conditions */
	case UPDATE:
		/* Direct copy of first filterSize_G IDs in the sequence */
		initFilter(mailBox);
		mailBoxFilterShadow_G[mailBox].mailboxTimeout = MAILBOX_DECAY_TIME;

		/* Initialising one mailBox per tick ensures all mailboxes are initialised before moving to RUN (mainly so that we can printf some debug info) */
		mailBox++;
		if(mailBox == filterSize_G){
			updateSequenceRequired_G = RUN;
		}
		break;

	/* Checking for CAN messages and updating filters - normal running conditions */
	case RUN:
		/* look through mailboxes for pending messages */
		for(mailBox=0; mailBox<filterSize_G; mailBox++){
			updateSingleMailbox(CANPORT_A, mailBox);

			if(checkMailboxState(CANPORT_A, mailBox) == RX_PENDING){
				disableMailbox(CANPORT_A, mailBox);

				/* read the CAN data into buffer (Nothing is done with the data, but nice to do this for realistic timing) */
				readRxMailbox(CANPORT_A, mailBox, CAN_RxMessages_G[mailBoxFilterShadow_G[mailBox].sequenceIndex_mapped].canData.rawData);

				/* Count message hits */
				CAN_RxMessages_G[mailBoxFilterShadow_G[mailBox].sequenceIndex_mapped].counter++;

/* Unsure whether mailbox decay is helpful. It appears not to make much difference with the segmentation */
#ifdef DECAY_LOGIC
			}
			else if(mailBoxFilterShadow_G[mailBox].mailboxTimeout > 0){
				mailBoxFilterShadow_G[mailBox].mailboxTimeout--;
			}

			if(mailBoxFilterShadow_G[mailBox].mailboxTimeout == 0){
				mailBoxFilterShadow_G[mailBox].mailboxTimeout = MAILBOX_DECAY_TIME;
#endif

				/* update the filter for next required ID  */
				updateFilter(mailBox);	/* Mailbox is re-enabled in configureRxMailbox() - this is done last to help prevent new message arrivals causing erroneous hits mid-way through process*/
			}
		}

		break;
	}
}


