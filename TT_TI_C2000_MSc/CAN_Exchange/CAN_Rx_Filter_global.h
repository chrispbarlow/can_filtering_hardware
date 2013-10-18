/***********************************************************************************************************
 * CAN_Rx_global.c
 *
 * Global CAN message receive buffers and control variables
 *
 *  Created on: 7 Mar 2013
 *      Author: chris.barlow
 * *********************************************************************************************************/

#ifndef CAN_RX_GLOBAL_H_
#define CAN_RX_GLOBAL_H_

#define NUM_MAILBOXES_MAX 	(32)
#define NUM_MESSAGES_MAX 	(100)
#define MAILBOX_DECAY_TIME	(100)
#define NUM_FILTER_SEGMENTS	(2)

#define DUPLICATES_LIMIT 	(1)		/* Controls the number of duplicates for each ID allowed to be added to filter between arrivals of that ID */
#define FILTERSIZE_RATIO	(2)

#include <DSP2833x_Device.h>
#include "../Lib/CAN/CAN.h"
#include "../global.h"

typedef enum{INIT,RUN,RESET,UPDATE} updateFlags_t;
extern updateFlags_t updateSequenceRequired_G;

/*******************************************************************************
 * CAN Rx Message array
 * Assigns messages to logging sequence position
 * *****************************************************************************/

/* If we were doing anything with the CAN data received, the individual bytes of the message could be accessed from here */
struct CAN_DATA_VARS_RX{

	/*byte variable examples */
	Uint16 DB3:8;
	Uint16 DB2:8;
	Uint16 DB1:8;
	Uint16 DB0:8;

	Uint16 DB7:8;
	Uint16 DB6:8;
	Uint16 DB5:8;
	Uint16 DB4:8;
};

/* Map CAN message Bytes with raw data array (2 x 32bit array) */
typedef union {
	Uint32 rawData[2];
	struct CAN_DATA_VARS_RX dataBytes;
}canData_t;


typedef struct {
	Uint32 canID;			/* CAN ID */
	canData_t canData;		/* CAN Data */
	Uint16 canDLC;			/* Message byte length */
	int16 timer;			/* Used for message scheduling */
	int16 timer_reload;
	Uint32 counter;			/* Used to track message hits */
} canRxMessage_t;

extern canRxMessage_t CAN_RxMessages_G[];


/*******************************************************************************
 * The filterShadow holds a copy of the current ID - mailbox mapping.
 * Necessary because we can't read the mailbox ID directly from the registry
 * *****************************************************************************/
typedef struct{
	Uint32 canID_mapped;				/* CAN ID currently being held in mailbox */
	Uint16 sequenceIndex_mapped;		/* Index of CAN_RxMessages[] mapped to mailbox */
	Uint16 mailboxTimeout;
}filterShadow_t;

extern filterShadow_t mailBoxFilterShadow_G[];


/*******************************************************************************
 * The logging list - list of CAN IDs transmitted to device in logging sequence order
 * *****************************************************************************/

typedef struct{
	Uint16 canID_LLRx;
	Uint16 canDLC_LLRx;
	Uint16 cycleTime_LLRx;
} logging_list_t;
extern logging_list_t loggingList_G[];


/*******************************************************************************
 * The segment structure - used to map segment space in the filter with part of the sequence,
 * and to keep track of the multiple sequence indexes required for segmentation.
 * *****************************************************************************/
typedef struct{
	Uint16 filterStart		: 8;
	Uint16 filterEnd		: 8;
	Uint16 sequenceStart	: 8;
	Uint16 sequenceEnd		: 8;
	Uint16 sequenceIndex	: 8;
} filterSegment_t;



/*******************************************************************************
 * Control values set dynamically when logging list is received
 * *****************************************************************************/
extern Uint16 numRxCANMsgs_G;
extern Uint16 filterSize_G;


/***********************************************************************************************************
 * Copies sequence details from temporary buffers to global message sequence array.
 * Since we don't know where in the sequence we will start, the schedule timer for all messages is set to 1.
 *
 * Arguments:
 * 		listSize -- number of messages in sequence.
 * *********************************************************************************************************/
void buildSequence(Uint16 listSize);


/***********************************************************************************************************
 * Controls the scheduling of the IDs in the filter.
 * Returns the next valid sequence index to use in the filter.
 * *********************************************************************************************************/
int16 getNextSequenceIndex(Uint16 mailbox_num);

/***********************************************************************************************************
 * Returns the filter segment that matches the requested mailbox. *
 * *********************************************************************************************************/
Uint16 findSegment(Uint16 mailbox);

/***********************************************************************************************************
 * Replaces the ID in the filter at location filterPointer, with ID from sequence at location sequencePointer.
 * Arguments:
 * 		filterIndex -- the Index of the filter mailbox to modify
 * 		sequenceIndex_replace -- The new sequence index to use in the filter
 * *********************************************************************************************************/
void updateFilter(Uint16 filterIndex, int16 sequenceIndex_replace);


#endif /* CAN_RX_GLOBAL_H_ */
