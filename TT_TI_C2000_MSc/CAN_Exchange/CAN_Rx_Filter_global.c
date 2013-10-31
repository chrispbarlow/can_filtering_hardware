/***********************************************************************************************************
 * CAN_Rx_global.c
 *
 * Global CAN message receive buffers and control variables
 *
 *  Created on: 7 Mar 2013
 *      Author: chris.barlow
 * *********************************************************************************************************/

#include "CAN_Rx_Filter_global.h"

/* Filter shadow is necessary due to being unable to read a mailbox's ID from registry */
filterShadow_t mailBoxFilterShadow_G[NUM_MESSAGES_MAX];

/* The main CAN Rx buffer - holds all received data in logging sequence order */
canRxMessage_t CAN_RxMessages_G[NUM_MESSAGES_MAX];

/* The logging list - list of CAN IDs transmitted to device in logging sequence order */
logging_list_t loggingList_G[NUM_MESSAGES_MAX];

/* The global sequence update state */
updateFlags_t updateSequenceRequired_G = INIT;

/* Global control variables */
Uint16 numRxCANMsgs_G = 0;
Uint16 filterSize_G = 0;
Uint16 numSegments_G = 0;

/* Filter segment array */
filterSegment_t segments[NUM_FILTER_SEGMENTS_MAX];
static Uint16 segmentNumber = 0;


/***********************************************************************************************************
 * Copies sequence details from temporary buffers to global message sequence array.
 * Since we don't know where in the sequence we will start, the schedule timer for all messages is set to 1.
 * *********************************************************************************************************/
void buildSequence(Uint16 listSize){
	Uint16 i, cycleTime_min = 0;

	segmentNumber = 0;

	/* Finds the minimum cycle time in the logging list */
 	cycleTime_min = 0;
 	numRxCANMsgs_G = listSize;
 	printf("msgs:%u\n",numRxCANMsgs_G);
 	for(i=0;i<listSize;i++){

 		/* Segments are assigned dynamically -
 		 * Limitations: ID's must be sent to the device ordered by cycle time, lowest - highest.
 		 */
 		if(loggingList_G[i].cycleTime_LLRx > cycleTime_min){
 			cycleTime_min = loggingList_G[i].cycleTime_LLRx;
 			newSegment(i);
 		}

		CAN_RxMessages_G[i].canID = loggingList_G[i].canID_LLRx;
		CAN_RxMessages_G[i].canData.rawData[0] = 0;
		CAN_RxMessages_G[i].canData.rawData[1] = 0;
		CAN_RxMessages_G[i].canDLC = loggingList_G[i].canDLC_LLRx;

		/* Force all timers to 1 for first iteration - level playing field */
		CAN_RxMessages_G[i].duplicates = 1;
		CAN_RxMessages_G[i].counter = 0;
 	}

 	/* final call to newSegment initialises the end point of the last segment */
	newSegment(listSize);
	numSegments_G = segmentNumber;
 }



/***********************************************************************************************************
 * Controls the scheduling of the IDs in the filter.
 * *********************************************************************************************************/
Uint16 getNextSequenceIndex(Uint16 mailbox_num){
	Uint16 sequenceIndex_next = 0;
	Uint16 segment;

	segment =  findSegment(mailbox_num);
	sequenceIndex_next = segments[segment].sequenceIndex;

	/* Find next required CAN ID in sequence */
	do{
		/* Wrap search */
		if(sequenceIndex_next < segments[segment].sequenceEnd){
			sequenceIndex_next++;
		}
		else{
			sequenceIndex_next = segments[segment].sequenceStart;
		}

		/* ID not already in mailbox, decrement 'schedule' timer (timer sits between -DUPLICATES ALLOWED and 0 while ID is in one or more mailboxes) */
		if(CAN_RxMessages_G[sequenceIndex_next].duplicates <= DUPLICATES_LIMIT){
			segments[segment].sequenceIndex = sequenceIndex_next;
			CAN_RxMessages_G[sequenceIndex_next].duplicates++;
		}
		else{
			CAN_RxMessages_G[sequenceIndex_next].duplicates = CAN_RxMessages_G[sequenceIndex_next].duplicates;
			segments[segment].sequenceIndex = segments[segment].sequenceIndex;
		}

	}	/* Search will abort if all messages have been checked */
	while(sequenceIndex_next != segments[segment].sequenceIndex);

	return sequenceIndex_next;
}


/***********************************************************************************************************
 * Replaces the ID in the filter at location filterPointer, with ID from sequence at location sequencePointer.
 * *********************************************************************************************************/
void updateFilter(Uint16 filterIndex){
	Uint16 sequenceIndex_new = 0;
	Uint16 segment;

	sequenceIndex_new = getNextSequenceIndex(filterIndex);
//
//	/* Find next required CAN ID in sequence */
//	do{
//		/* Wrap search */
//		if(sequenceIndex_new < segments[segment].sequenceEnd){
//			sequenceIndex_new++;
//		}
//		else{
//			sequenceIndex_new = segments[segment].sequenceStart;
//		}
//
//		/* ID not already in mailbox, decrement 'schedule' timer (timer sits between -DUPLICATES ALLOWED and 0 while ID is in one or more mailboxes) */
//		if(CAN_RxMessages_G[sequenceIndex_new].duplicates <= DUPLICATES_LIMIT){
//			segments[segment].sequenceIndex = sequenceIndex_new;
//			CAN_RxMessages_G[sequenceIndex_new].duplicates++;
//		}
//		else{
//			segments[segment].sequenceIndex = segments[segment].sequenceIndex;
//			CAN_RxMessages_G[sequenceIndex_new].duplicates = CAN_RxMessages_G[sequenceIndex_new].duplicates;
//		}
//
//	}	/* Search will abort if all messages have been checked */
//	while(sequenceIndex_new != segments[segment].sequenceIndex);


	/* Message scheduling */
	CAN_RxMessages_G[mailBoxFilterShadow_G[filterIndex].sequenceIndex_mapped].duplicates = 0;								/* We allow more duplicates if a message is accepted by the filter (regardless of the number of duplicates already present) */

	/* ID replacement in shadow */
	mailBoxFilterShadow_G[filterIndex].canID_mapped = CAN_RxMessages_G[sequenceIndex_new].canID;
	mailBoxFilterShadow_G[filterIndex].sequenceIndex_mapped = sequenceIndex_new;

	/* Real ID replacement - also re-enables mailbox*/
	configureRxMailbox(CANPORT_A, filterIndex, ID_STD, CAN_RxMessages_G[sequenceIndex_new].canID, CAN_RxMessages_G[sequenceIndex_new].canDLC);

}

/***********************************************************************************************************
 * Sets the mailbox at filterIndex to initial state
 * *********************************************************************************************************/
void initFilter(Uint16 filterIndex){
	Uint16 sequenceIndex_init, segmentIndex;

	segmentIndex =  findSegment(filterIndex);
	sequenceIndex_init = segments[segmentIndex].sequenceIndex++;

	/* Replicates the duplicates mechanism for first use of ID */
	CAN_RxMessages_G[sequenceIndex_init].duplicates = 0;

	/* ID replacement in shadow */
	mailBoxFilterShadow_G[filterIndex].canID_mapped = CAN_RxMessages_G[sequenceIndex_init].canID;
	mailBoxFilterShadow_G[filterIndex].sequenceIndex_mapped = sequenceIndex_init;

	/* Real ID replacement - also re-enables mailbox*/
	configureRxMailbox(CANPORT_A, filterIndex, ID_STD, CAN_RxMessages_G[sequenceIndex_init].canID, CAN_RxMessages_G[sequenceIndex_init].canDLC);
}

/***********************************************************************************************************
 * Returns the filter segment that matches the requested mailbox. *
 * *********************************************************************************************************/
Uint16 findSegment(Uint16 mailbox){
	Uint16 segmentNumber = 0, i;

	for(i = 0; i < numSegments_G; i++){
		if((mailbox >= segments[i].filterStart) && (mailbox <= segments[i].filterEnd)){
			segmentNumber = i;
		}
	}

	return segmentNumber;
}


/***********************************************************************************************************
 * Ends previous segment, and begins a new one. *
 * *********************************************************************************************************/
void newSegment(Uint16 SequenceIndex){
	Uint16 filterIndex = 0;
	printf("I:%u",SequenceIndex);

	/* Dynamically assigns a space in the filter depending on the current SequenceIndex location and the FILTERSIZE_RATIO */
	filterIndex = SequenceIndex/FILTERSIZE_RATIO;
	if((filterIndex%FILTERSIZE_RATIO)!=0){
		filterIndex += 1;
	}

	/* Makes sure the filter doesn't overflow */
	if(filterIndex > NUM_MAILBOXES_MAX){
		segments[segmentNumber].filterEnd = (NUM_MAILBOXES_MAX-1);
	}
	else{
		segments[segmentNumber].filterEnd = (filterIndex-1);
	}

	/* Global used for filter looping */
	filterSize_G = (segments[segmentNumber].filterEnd + 1);

	/* Set sequence end point */
	segments[segmentNumber].sequenceEnd = (SequenceIndex-1);

	printf("Seg:%uSE:%uFE:%u\n",segmentNumber,segments[segmentNumber].sequenceEnd,segments[segmentNumber].filterEnd);

	/* Don't increment on initial function call */
	if(SequenceIndex > 0){
		segmentNumber++;
		printf("Seg:%u\n",segmentNumber);
	}

	/* Start a new segment if there are logging list items left */
	if((SequenceIndex < numRxCANMsgs_G) && (segmentNumber < (NUM_FILTER_SEGMENTS_MAX-1))){
		segments[segmentNumber].filterStart = filterIndex;
		segments[segmentNumber].sequenceStart = SequenceIndex;
		segments[segmentNumber].sequenceIndex = SequenceIndex;
		printf("SS:%uFS:%u\n",segments[segmentNumber].sequenceStart,segments[segmentNumber].filterStart);
	}

}
