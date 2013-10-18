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
	Uint16 sequenceIndex_received;
	int16 sequenceIndex_new;

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
		if(filterSize_G == 0){	/* Dynamic filterSize calculation if 0 is received from configuration app */
			filterSize_G = numRxCANMsgs_G/FILTERSIZE_RATIO;
			if((numRxCANMsgs_G%FILTERSIZE_RATIO)!=0){
				filterSize_G += 1;
			}
		}
		updateSequenceRequired_G = UPDATE;
		break;

	/* Set up mailboxes for initial filter conditions */
	case UPDATE:

		/* Direct copy of first filterSize_G IDs in the sequence */
		updateFilter(mailBox,mailBox);
		mailBoxFilterShadow_G[mailBox].mailboxTimeout = MAILBOX_DECAY_TIME;
		//printf("%d: %d %d\n",mailBox, CAN_RxMessages_G[mailBox].timer, CAN_RxMessages_G[mailBox].timer_reload);

		/* Initialising one mailBox per tick ensures all mailboxes are initialised before moving to RUN (mainly so that we can printf some debug info) */
		mailBox++;
		if(mailBox == filterSize_G){
			getNextSequenceIndex(0); /* Calling here re-initialises the sequencePointer */
			getNextSequenceIndex(1); /* Calling here re-initialises the sequencePointer */
			updateSequenceRequired_G = RUN;
			//printf("T:%d", mailBox);
		}
		break;

	/* Checking for CAN messages and updating filters - normal running conditions */
	case RUN:
		/* look through mailboxes for pending messages */
		for(mailBox=0; mailBox<filterSize_G; mailBox++){

			if(checkMailboxState(CANPORT_A, mailBox) == RX_PENDING){
				mailBoxFilterShadow_G[mailBox].mailboxTimeout = 0;
				disableMailbox(CANPORT_A, mailBox);

				/* Find message pointer from mailbox shadow */
				sequenceIndex_received = mailBoxFilterShadow_G[mailBox].sequenceIndex_mapped;

				/* read the CAN data into buffer (Nothing is done with the data, but nice to do this for realistic timing) */
				readRxMailbox(CANPORT_A, mailBox, CAN_RxMessages_G[sequenceIndex_received].canData.rawData);

				/* Count message hits */
				CAN_RxMessages_G[sequenceIndex_received].counter++;
			}
			else if(mailBoxFilterShadow_G[mailBox].mailboxTimeout > 0){
//				mailBoxFilterShadow_G[mailBox].mailboxTimeout--;			/* Unsure whether decay is helpful. It appears not to make much difference with the segmentation */
			}

			if(mailBoxFilterShadow_G[mailBox].mailboxTimeout == 0){

				disableMailbox(CANPORT_A, mailBox);

				/* ID scheduling and duplication control */
				sequenceIndex_new = getNextSequenceIndex(mailBox);

				/* update the filter for next required ID  */
				updateFilter(mailBox, sequenceIndex_new);	/* Mailbox is re-enabled in configureRxMailbox() - this is done last to help prevent new message arrivals causing erroneous hits mid-way through process*/

				mailBoxFilterShadow_G[mailBox].mailboxTimeout = MAILBOX_DECAY_TIME;
			}

		}
		break;
	}
}


