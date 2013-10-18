/*
 * CAN.h
 *
 *  Created on: 13 Feb 2013
 *      Author: chris.barlow
 */

#ifndef CAN_H_
#define CAN_H_

#define CANPORT_A (0)
#define CANPORT_B (1)

#define ID_STD (0)
#define ID_EXT (1)

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File


typedef enum {CAN_TX, CAN_RX} mailboxDirection_t;
typedef enum {DISABLED, TX_FREE, RX_FREE, TX_PENDING, RX_PENDING, TX_SENT, RX_ARRIVAL, TX_ERR} messageObjectStates_t;

typedef struct{
	messageObjectStates_t 		mailboxState;		/* High-level Tx / Rx State of message object */
	volatile struct MBOX *		mailbox;			/* Mailbox */
	volatile union CANLAM_REG * LAMRegs; 			/* Local acceptance mask */
	volatile Uint32 * 			ECanaMOTORegs;		/* Message Object Timeout */
	volatile Uint32 * 			ECanaMOTSRegs;		/* Message Object Time Stamp */
} messageObject_t;



typedef struct{
	volatile struct ECAN_REGS *canRegs;						/* Control and status registers */
	messageObject_t message_Objects[32];						/* Message objects */
} canPort_t;

void CAN_Init(Uint16 baudA, Uint16 baudB);
void CAN_Test(void);

/* find a mailbox of a given messageObjectState */
int16 findMailbox(canPort_t * port, messageObjectStates_t state);

/* returns -1 if mailbox already enabled, 0 on successful configuration */
int16 configureRxMailbox(char port, char mbNum, char IDE, Uint32 canID, Uint16 dataLength);
int16 configureTxMailbox(char port, char mbNum, char IDE, Uint32 canID, Uint16 dataLength);

/* takes data from byte array and loads into selected mailbox (mbNum) */
int16 loadTxMailbox(char port, char mbNum, Uint32 data[]);

int16 commitSendMailbox(char port, char mbNum);

void disableMailbox(char port, char mbNum);

int16 readRxMailbox(char port, char mbNum, Uint32 data[]);


/* updates the mailbox state machine */
void updateMailboxes(char port); /* This will probably become the CANTx_update() task */

messageObjectStates_t checkMailboxState(char port, char mbNum);
Uint32 getMailboxID(char port, char mbNum);
Uint32 getCANErrors(char port);

void InitCanA(Uint16 baud);		// Initialize eCAN-A module
void InitCanB(Uint16 baud);		// Initialize eCAN-B module


#endif /* CAN_H_ */
