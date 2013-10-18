/*############################################################################################

 ASSUMPTIONS:

    This program requires the DSP2833x header files.

    Both CAN ports of the 2833x DSP need to be connected
    to each other (via CAN transceivers)

       eCANA is on GPIO31 (CANTXA)  and
                   GPIO30 (CANRXA)

       eCANB is on GPIO8  (CANTXB)  and
                   GPIO10 (CANRXB)

	CAN arrives as:					B0, B1, B2, B3, B4, B5, B6, B7

	eCAN is by default set to DBO = 1, meaning data bytes are stored as follows:
	CANMDL: B3, B2, B1, B0
	CANMDH: B7, B6, B5, B4


 DESCRIPTION:

    This example TRANSMITS data to another CAN module using MAILBOX5
    This program could either loop forever or transmit "n" # of times,
    where "n" is the TXCOUNT value.

    This example can be used to check CAN-A and CAN-B. Since CAN-B is
    initialized in DSP2833x_ECan.c, it will acknowledge all frames
    transmitted by the node on which this code runs. Both CAN ports of
    the 2833x DSP need to be connected to each other (via CAN transceivers)

############################################################################################*/

#include "CAN.h"
#include <stdio.h>


#define TXCOUNT  100  // Transmission will take place (TXCOUNT) times..
long      i;
long 	  loopcount = 0;

static const Uint32 bitSelect_32 = 0x00000001;
static struct ECAN_REGS canRegsShadow;

canPort_t CAN_Ports[2] = {
		{
				&ECanaRegs,
				{
					{ DISABLED,	&ECanaMboxes.MBOX0,		&ECanaLAMRegs.LAM0,		&ECanaMOTORegs.MOTO0,	&ECanaMOTSRegs.MOTS0	},
					{ DISABLED,	&ECanaMboxes.MBOX1,		&ECanaLAMRegs.LAM1,		&ECanaMOTORegs.MOTO1,	&ECanaMOTSRegs.MOTS1	},
					{ DISABLED,	&ECanaMboxes.MBOX2,		&ECanaLAMRegs.LAM2,		&ECanaMOTORegs.MOTO2,	&ECanaMOTSRegs.MOTS2	},
					{ DISABLED,	&ECanaMboxes.MBOX3,		&ECanaLAMRegs.LAM3,		&ECanaMOTORegs.MOTO3,	&ECanaMOTSRegs.MOTS3	},
					{ DISABLED,	&ECanaMboxes.MBOX4,		&ECanaLAMRegs.LAM4,		&ECanaMOTORegs.MOTO4,	&ECanaMOTSRegs.MOTS4	},
					{ DISABLED,	&ECanaMboxes.MBOX5,		&ECanaLAMRegs.LAM5,		&ECanaMOTORegs.MOTO5,	&ECanaMOTSRegs.MOTS5	},
					{ DISABLED,	&ECanaMboxes.MBOX6,		&ECanaLAMRegs.LAM6,		&ECanaMOTORegs.MOTO6,	&ECanaMOTSRegs.MOTS6	},
					{ DISABLED,	&ECanaMboxes.MBOX7,		&ECanaLAMRegs.LAM7,		&ECanaMOTORegs.MOTO7,	&ECanaMOTSRegs.MOTS7	},
					{ DISABLED,	&ECanaMboxes.MBOX8,		&ECanaLAMRegs.LAM8,		&ECanaMOTORegs.MOTO8,	&ECanaMOTSRegs.MOTS8	},
					{ DISABLED,	&ECanaMboxes.MBOX9,		&ECanaLAMRegs.LAM9,		&ECanaMOTORegs.MOTO9,	&ECanaMOTSRegs.MOTS9	},
					{ DISABLED,	&ECanaMboxes.MBOX10,	&ECanaLAMRegs.LAM10,	&ECanaMOTORegs.MOTO10,	&ECanaMOTSRegs.MOTS10	},
					{ DISABLED,	&ECanaMboxes.MBOX11,	&ECanaLAMRegs.LAM11,	&ECanaMOTORegs.MOTO11,	&ECanaMOTSRegs.MOTS11	},
					{ DISABLED,	&ECanaMboxes.MBOX12,	&ECanaLAMRegs.LAM12,	&ECanaMOTORegs.MOTO12,	&ECanaMOTSRegs.MOTS12	},
					{ DISABLED,	&ECanaMboxes.MBOX13,	&ECanaLAMRegs.LAM13,	&ECanaMOTORegs.MOTO13,	&ECanaMOTSRegs.MOTS13	},
					{ DISABLED,	&ECanaMboxes.MBOX14,	&ECanaLAMRegs.LAM14,	&ECanaMOTORegs.MOTO14,	&ECanaMOTSRegs.MOTS14	},
					{ DISABLED,	&ECanaMboxes.MBOX15,	&ECanaLAMRegs.LAM15,	&ECanaMOTORegs.MOTO15,	&ECanaMOTSRegs.MOTS15	},
					{ DISABLED,	&ECanaMboxes.MBOX16,	&ECanaLAMRegs.LAM16,	&ECanaMOTORegs.MOTO16,	&ECanaMOTSRegs.MOTS16	},
					{ DISABLED,	&ECanaMboxes.MBOX17,	&ECanaLAMRegs.LAM17,	&ECanaMOTORegs.MOTO17,	&ECanaMOTSRegs.MOTS17	},
					{ DISABLED,	&ECanaMboxes.MBOX18,	&ECanaLAMRegs.LAM18,	&ECanaMOTORegs.MOTO18,	&ECanaMOTSRegs.MOTS18	},
					{ DISABLED,	&ECanaMboxes.MBOX19,	&ECanaLAMRegs.LAM19,	&ECanaMOTORegs.MOTO19,	&ECanaMOTSRegs.MOTS19	},
					{ DISABLED,	&ECanaMboxes.MBOX20,	&ECanaLAMRegs.LAM20,	&ECanaMOTORegs.MOTO20,	&ECanaMOTSRegs.MOTS20	},
					{ DISABLED,	&ECanaMboxes.MBOX21,	&ECanaLAMRegs.LAM21,	&ECanaMOTORegs.MOTO21,	&ECanaMOTSRegs.MOTS21	},
					{ DISABLED,	&ECanaMboxes.MBOX22,	&ECanaLAMRegs.LAM22,	&ECanaMOTORegs.MOTO22,	&ECanaMOTSRegs.MOTS22	},
					{ DISABLED,	&ECanaMboxes.MBOX23,	&ECanaLAMRegs.LAM23,	&ECanaMOTORegs.MOTO23,	&ECanaMOTSRegs.MOTS23	},
					{ DISABLED,	&ECanaMboxes.MBOX24,	&ECanaLAMRegs.LAM24,	&ECanaMOTORegs.MOTO24,	&ECanaMOTSRegs.MOTS24	},
					{ DISABLED,	&ECanaMboxes.MBOX25,	&ECanaLAMRegs.LAM25,	&ECanaMOTORegs.MOTO25,	&ECanaMOTSRegs.MOTS25	},
					{ DISABLED,	&ECanaMboxes.MBOX26,	&ECanaLAMRegs.LAM26,	&ECanaMOTORegs.MOTO26,	&ECanaMOTSRegs.MOTS26	},
					{ DISABLED,	&ECanaMboxes.MBOX27,	&ECanaLAMRegs.LAM27,	&ECanaMOTORegs.MOTO27,	&ECanaMOTSRegs.MOTS27	},
					{ DISABLED,	&ECanaMboxes.MBOX28,	&ECanaLAMRegs.LAM28,	&ECanaMOTORegs.MOTO28,	&ECanaMOTSRegs.MOTS28	},
					{ DISABLED,	&ECanaMboxes.MBOX29,	&ECanaLAMRegs.LAM29,	&ECanaMOTORegs.MOTO29,	&ECanaMOTSRegs.MOTS29	},
					{ DISABLED,	&ECanaMboxes.MBOX30,	&ECanaLAMRegs.LAM30,	&ECanaMOTORegs.MOTO30,	&ECanaMOTSRegs.MOTS30	},
					{ DISABLED,	&ECanaMboxes.MBOX31,	&ECanaLAMRegs.LAM31,	&ECanaMOTORegs.MOTO31,	&ECanaMOTSRegs.MOTS31	}
				}
		},
		{
				&ECanbRegs,
				{
					{ DISABLED,	&ECanbMboxes.MBOX0,		&ECanbLAMRegs.LAM0,		&ECanbMOTORegs.MOTO0,	&ECanbMOTSRegs.MOTS0	},
					{ DISABLED,	&ECanbMboxes.MBOX1,		&ECanbLAMRegs.LAM1,		&ECanbMOTORegs.MOTO1,	&ECanbMOTSRegs.MOTS1	},
					{ DISABLED,	&ECanbMboxes.MBOX2,		&ECanbLAMRegs.LAM2,		&ECanbMOTORegs.MOTO2,	&ECanbMOTSRegs.MOTS2	},
					{ DISABLED,	&ECanbMboxes.MBOX3,		&ECanbLAMRegs.LAM3,		&ECanbMOTORegs.MOTO3,	&ECanbMOTSRegs.MOTS3	},
					{ DISABLED,	&ECanbMboxes.MBOX4,		&ECanbLAMRegs.LAM4,		&ECanbMOTORegs.MOTO4,	&ECanbMOTSRegs.MOTS4	},
					{ DISABLED,	&ECanbMboxes.MBOX5,		&ECanbLAMRegs.LAM5,		&ECanbMOTORegs.MOTO5,	&ECanbMOTSRegs.MOTS5	},
					{ DISABLED,	&ECanbMboxes.MBOX6,		&ECanbLAMRegs.LAM6,		&ECanbMOTORegs.MOTO6,	&ECanbMOTSRegs.MOTS6	},
					{ DISABLED,	&ECanbMboxes.MBOX7,		&ECanbLAMRegs.LAM7,		&ECanbMOTORegs.MOTO7,	&ECanbMOTSRegs.MOTS7	},
					{ DISABLED,	&ECanbMboxes.MBOX8,		&ECanbLAMRegs.LAM8,		&ECanbMOTORegs.MOTO8,	&ECanbMOTSRegs.MOTS8	},
					{ DISABLED,	&ECanbMboxes.MBOX9,		&ECanbLAMRegs.LAM9,		&ECanbMOTORegs.MOTO9,	&ECanbMOTSRegs.MOTS9	},
					{ DISABLED,	&ECanbMboxes.MBOX10,	&ECanbLAMRegs.LAM10,	&ECanbMOTORegs.MOTO10,	&ECanbMOTSRegs.MOTS10	},
					{ DISABLED,	&ECanbMboxes.MBOX11,	&ECanbLAMRegs.LAM11,	&ECanbMOTORegs.MOTO11,	&ECanbMOTSRegs.MOTS11	},
					{ DISABLED,	&ECanbMboxes.MBOX12,	&ECanbLAMRegs.LAM12,	&ECanbMOTORegs.MOTO12,	&ECanbMOTSRegs.MOTS12	},
					{ DISABLED,	&ECanbMboxes.MBOX13,	&ECanbLAMRegs.LAM13,	&ECanbMOTORegs.MOTO13,	&ECanbMOTSRegs.MOTS13	},
					{ DISABLED,	&ECanbMboxes.MBOX14,	&ECanbLAMRegs.LAM14,	&ECanbMOTORegs.MOTO14,	&ECanbMOTSRegs.MOTS14	},
					{ DISABLED,	&ECanbMboxes.MBOX15,	&ECanbLAMRegs.LAM15,	&ECanbMOTORegs.MOTO15,	&ECanbMOTSRegs.MOTS15	},
					{ DISABLED,	&ECanbMboxes.MBOX16,	&ECanbLAMRegs.LAM16,	&ECanbMOTORegs.MOTO16,	&ECanbMOTSRegs.MOTS16	},
					{ DISABLED,	&ECanbMboxes.MBOX17,	&ECanbLAMRegs.LAM17,	&ECanbMOTORegs.MOTO17,	&ECanbMOTSRegs.MOTS17	},
					{ DISABLED,	&ECanbMboxes.MBOX18,	&ECanbLAMRegs.LAM18,	&ECanbMOTORegs.MOTO18,	&ECanbMOTSRegs.MOTS18	},
					{ DISABLED,	&ECanbMboxes.MBOX19,	&ECanbLAMRegs.LAM19,	&ECanbMOTORegs.MOTO19,	&ECanbMOTSRegs.MOTS19	},
					{ DISABLED,	&ECanbMboxes.MBOX20,	&ECanbLAMRegs.LAM20,	&ECanbMOTORegs.MOTO20,	&ECanbMOTSRegs.MOTS20	},
					{ DISABLED,	&ECanbMboxes.MBOX21,	&ECanbLAMRegs.LAM21,	&ECanbMOTORegs.MOTO21,	&ECanbMOTSRegs.MOTS21	},
					{ DISABLED,	&ECanbMboxes.MBOX22,	&ECanbLAMRegs.LAM22,	&ECanbMOTORegs.MOTO22,	&ECanbMOTSRegs.MOTS22	},
					{ DISABLED,	&ECanbMboxes.MBOX23,	&ECanbLAMRegs.LAM23,	&ECanbMOTORegs.MOTO23,	&ECanbMOTSRegs.MOTS23	},
					{ DISABLED,	&ECanbMboxes.MBOX24,	&ECanbLAMRegs.LAM24,	&ECanbMOTORegs.MOTO24,	&ECanbMOTSRegs.MOTS24	},
					{ DISABLED,	&ECanbMboxes.MBOX25,	&ECanbLAMRegs.LAM25,	&ECanbMOTORegs.MOTO25,	&ECanbMOTSRegs.MOTS25	},
					{ DISABLED,	&ECanbMboxes.MBOX26,	&ECanbLAMRegs.LAM26,	&ECanbMOTORegs.MOTO26,	&ECanbMOTSRegs.MOTS26	},
					{ DISABLED,	&ECanbMboxes.MBOX27,	&ECanbLAMRegs.LAM27,	&ECanbMOTORegs.MOTO27,	&ECanbMOTSRegs.MOTS27	},
					{ DISABLED,	&ECanbMboxes.MBOX28,	&ECanbLAMRegs.LAM28,	&ECanbMOTORegs.MOTO28,	&ECanbMOTSRegs.MOTS28	},
					{ DISABLED,	&ECanbMboxes.MBOX29,	&ECanbLAMRegs.LAM29,	&ECanbMOTORegs.MOTO29,	&ECanbMOTSRegs.MOTS29	},
					{ DISABLED,	&ECanbMboxes.MBOX30,	&ECanbLAMRegs.LAM30,	&ECanbMOTORegs.MOTO30,	&ECanbMOTSRegs.MOTS30	},
					{ DISABLED,	&ECanbMboxes.MBOX31,	&ECanbLAMRegs.LAM31,	&ECanbMOTORegs.MOTO31,	&ECanbMOTSRegs.MOTS31	}
				}
		}
};

void CAN_Init(Uint16 baudA, Uint16 baudB){
   if(baudA > 0){
	   InitECanaGpio();
	   InitCanA(baudA);		// Initialize eCAN-A module
   }
   if(baudB > 0){
	   InitECanbGpio();
	   InitCanB(baudB);		// Initialize eCAN-B module
   }
}

void CAN_Test(void){
	configureTxMailbox(CANPORT_A, 25, ID_EXT, 0x15555555, 8);

	Uint32 canTxData[2] = {0x00010203, 0x04050607};
	/* Write to the mailbox RAM field */
	loadTxMailbox(CANPORT_A, 25, canTxData);

	/* Begin transmitting */
	for(i=0; i < TXCOUNT; i++){
       commitSendMailbox(CANPORT_A, 25);

       do{
    	   updateMailboxes(CANPORT_A);
       } while(CAN_Ports[CANPORT_A].message_Objects[25].mailboxState != TX_FREE);

       loopcount ++;
    }
}

/* find a mailbox of a given messageObjectState */
int16 findMailbox(canPort_t * port, messageObjectStates_t state){
	Uint16 i;
	int16 freePort = -1;

	for(i = 0; i < 32; i++)	{
		if((port->message_Objects[i].mailboxState = state) && (freePort == -1)){
			freePort = i;
		}
	}

	return freePort;
}

Uint32 getCANErrors(char port){
	return CAN_Ports[port].canRegs->CANES.all;
}


/* Always returns 0  */
int16 configureRxMailbox(char port, char mbNum, char IDE, Uint32 canID, Uint16 dataLength){
	int16 success = -1;
	Uint32 messagePending = 0;

	/*Read regs into shadow */
	canRegsShadow.CANME.all = CAN_Ports[port].canRegs->CANME.all;
	canRegsShadow.CANMD.all = CAN_Ports[port].canRegs->CANMD.all;
	canRegsShadow.CANTRS.all = CAN_Ports[port].canRegs->CANTRS.all;
	canRegsShadow.CANTRR.all = CAN_Ports[port].canRegs->CANTRR.all;


	/* disable mailbox */
	canRegsShadow.CANME.all &= ~(bitSelect_32<<mbNum);
	CAN_Ports[port].canRegs->CANME.all = canRegsShadow.CANME.all;


	if(IDE == ID_EXT){
		canID |= (bitSelect_32<<31);
	}
	else{
		canID <<= 18;
	}

	CAN_Ports[port].message_Objects[mbNum].mailbox->MSGID.all = canID;
	CAN_Ports[port].message_Objects[mbNum].mailbox->MSGCTRL.bit.DLC = dataLength;

	/* Set direction & Initialise */
	canRegsShadow.CANMD.all |= (bitSelect_32<<mbNum);
	CAN_Ports[port].message_Objects[mbNum].mailboxState = RX_FREE;

	/* Enable mailbox */
	canRegsShadow.CANME.all = CAN_Ports[port].canRegs->CANME.all;
	canRegsShadow.CANME.all |= (bitSelect_32<<mbNum);

	canRegsShadow.CANOPC.all = CAN_Ports[port].canRegs->CANOPC.all;
	canRegsShadow.CANOPC.all |= (bitSelect_32<<mbNum);


	CAN_Ports[port].canRegs->CANMD.all = canRegsShadow.CANMD.all;
	CAN_Ports[port].canRegs->CANOPC.all = canRegsShadow.CANOPC.all;
	CAN_Ports[port].canRegs->CANME.all = canRegsShadow.CANME.all;

	/* Trying to force CANRMP.mbNum to clear */
	do{
		canRegsShadow.CANRMP.all = CAN_Ports[port].canRegs->CANRMP.all;
		messagePending = canRegsShadow.CANRMP.all & (bitSelect_32<<mbNum);

		CAN_Ports[port].canRegs->CANRMP.all = messagePending;
	}while(messagePending != 0);

	success = 0;

	return success;
}

/* Always returns 0  */
int16 configureTxMailbox(char port, char mbNum, char IDE, Uint32 canID, Uint16 dataLength){
	int16 success = -1;

	/*Read regs into shadow */
	canRegsShadow.CANME.all = CAN_Ports[port].canRegs->CANME.all;
	canRegsShadow.CANMD.all = CAN_Ports[port].canRegs->CANMD.all;
	canRegsShadow.CANTRS.all = CAN_Ports[port].canRegs->CANTRS.all;
	canRegsShadow.CANTRR.all = CAN_Ports[port].canRegs->CANTRR.all;

	/* Make sure TRS bit is cleared by setting abort bit TODO: timeout on this while loop*/
	while((canRegsShadow.CANTRS.all & (bitSelect_32<<mbNum)) != 0){
		canRegsShadow.CANTRR.all |= (bitSelect_32<<mbNum);
		CAN_Ports[port].canRegs->CANTRR.all = canRegsShadow.CANTRR.all;
		canRegsShadow.CANTRS.all = CAN_Ports[port].canRegs->CANTRS.all;
	}

	/* disable mailbox */
	canRegsShadow.CANME.all &= ~(bitSelect_32<<mbNum);
	CAN_Ports[port].canRegs->CANME.all = canRegsShadow.CANME.all;


	if(IDE == ID_EXT){
		canID |= (bitSelect_32<<31);
	}
	else{
		canID <<= 18;
	}

	CAN_Ports[port].message_Objects[mbNum].mailbox->MSGID.all = canID;
	CAN_Ports[port].message_Objects[mbNum].mailbox->MSGCTRL.bit.DLC = dataLength;

	/* Set direction & Initialise */
	canRegsShadow.CANMD.all &= ~(bitSelect_32<<mbNum);
	CAN_Ports[port].message_Objects[mbNum].mailboxState = TX_FREE;

	/* Enable mailbox */
	canRegsShadow.CANME.all |= (bitSelect_32<<mbNum);

	CAN_Ports[port].canRegs->CANMD.all = canRegsShadow.CANMD.all;
	CAN_Ports[port].canRegs->CANME.all = canRegsShadow.CANME.all;

	success = 0;

	return success;
}

int16 loadTxMailbox(char port, char mbNum, Uint32 data[]){
	int16 dataLength = 0;

	canRegsShadow.CANMD.all = CAN_Ports[port].canRegs->CANMD.all;

	CAN_Ports[port].message_Objects[mbNum].mailbox->MDL.all = 0x00000000;
	CAN_Ports[port].message_Objects[mbNum].mailbox->MDH.all = 0x00000000;

	if((canRegsShadow.CANMD.all & (bitSelect_32<<mbNum)) == 0){
		dataLength = CAN_Ports[port].message_Objects[mbNum].mailbox->MSGCTRL.bit.DLC;
		CAN_Ports[port].message_Objects[mbNum].mailbox->MDL.all = data[0];
		CAN_Ports[port].message_Objects[mbNum].mailbox->MDH.all = data[1];
	}
	else{
		dataLength = -1;	/* mailbox not set to receive */
	}

	return dataLength;
}

int16 commitSendMailbox(char port, char mbNum){
	int16 success = -1;

	canRegsShadow.CANTRS.all = 0x00000000;
	canRegsShadow.CANTRS.all |= (bitSelect_32<<mbNum);
	CAN_Ports[port].canRegs->CANTRS.all = canRegsShadow.CANTRS.all;

	CAN_Ports[port].message_Objects[mbNum].mailboxState = TX_PENDING;

	success = 0;

	return success;
}

void disableMailbox(char port, char mbNum){
	canRegsShadow.CANME.all = CAN_Ports[port].canRegs->CANME.all;
	/* disable mailbox */
	canRegsShadow.CANME.all &= ~(bitSelect_32<<mbNum);
	CAN_Ports[port].canRegs->CANME.all = canRegsShadow.CANME.all;
	CAN_Ports[port].message_Objects[mbNum].mailboxState = DISABLED;
}


int16 readRxMailbox(char port, char mbNum, Uint32 data[]){
	int16 dataLength = 0;
	Uint32 messagePending = 0;

	canRegsShadow.CANMD.all = CAN_Ports[port].canRegs->CANMD.all;

	if((canRegsShadow.CANMD.all & (bitSelect_32<<mbNum)) != 0){

		dataLength = CAN_Ports[port].message_Objects[mbNum].mailbox->MSGCTRL.bit.DLC;

		data[0] = CAN_Ports[port].message_Objects[mbNum].mailbox->MDL.all;
		data[1] = CAN_Ports[port].message_Objects[mbNum].mailbox->MDH.all;

		CAN_Ports[port].message_Objects[mbNum].mailboxState = RX_FREE;
	}
	else{
		dataLength = -1;	/* mailbox not set to receive */
	}

	/* Trying to force CANRMP.mbNum to clear */
	do{
		canRegsShadow.CANRMP.all = CAN_Ports[port].canRegs->CANRMP.all;
		messagePending = canRegsShadow.CANRMP.all & (bitSelect_32<<mbNum);

		CAN_Ports[port].canRegs->CANRMP.all = messagePending;
	}while(messagePending != 0);

	return dataLength;
}

/* Updates the mailbox state machine (call at least twice between Tx and Rx tasks) */
void updateMailboxes(char port){

	Uint16 i;
	Uint32 CANRMPclearMask = 0x00000000, CANTAclearMask = 0x00000000;

	canRegsShadow.CANTA.all 	= CAN_Ports[port].canRegs->CANTA.all;
	canRegsShadow.CANTRS.all 	= CAN_Ports[port].canRegs->CANTRS.all;
	canRegsShadow.CANRMP.all 	= CAN_Ports[port].canRegs->CANRMP.all;

	for(i = 0; i < 32; i++)
	{
		switch(CAN_Ports[port].message_Objects[i].mailboxState)
		{

		/** Tx States **/
		case TX_FREE:
			/* mailbox loaded by separate task */
			break;

		case TX_PENDING:
			if((canRegsShadow.CANTRS.all & (bitSelect_32<<i)) == 0)
			{
				if(((canRegsShadow.CANTA.all & (bitSelect_32<<i)) >> i) == 1)
				{
					CANTAclearMask |= (bitSelect_32<<i);
					CAN_Ports[port].message_Objects[i].mailboxState = TX_SENT;
				}
				else
				{
					CAN_Ports[port].message_Objects[i].mailboxState = TX_ERR;
				}
			}

			break;

		case TX_SENT:
			if((canRegsShadow.CANTA.all & (bitSelect_32<<i)) == 0)
			{
				CAN_Ports[port].message_Objects[i].mailboxState = TX_FREE;
			}
			break;

		case TX_ERR:
			CAN_Ports[port].message_Objects[i].mailboxState = TX_FREE;
			/* Transmission aborted */
			/* TODO: What to do here? */
			break;

		/** Rx States **/
		case RX_FREE:
			if((canRegsShadow.CANRMP.all & (bitSelect_32<<i)) != 0)
			{
				CANRMPclearMask |= (bitSelect_32<<i);
				CAN_Ports[port].message_Objects[i].mailboxState = RX_ARRIVAL;
			}
			break;

		case RX_ARRIVAL:
			if((canRegsShadow.CANRMP.all & (bitSelect_32<<i)) == 0)
			{
				CAN_Ports[port].message_Objects[i].mailboxState = RX_PENDING;
			}
			break;

		case RX_PENDING:
			/* mailbox read by separate task */
			break;


		case DISABLED:
		default:
			break;
		}
	}

	CAN_Ports[port].canRegs->CANTA.all = CANTAclearMask;
	CAN_Ports[port].canRegs->CANRMP.all = CANRMPclearMask;
}

messageObjectStates_t checkMailboxState(char port, char mbNum){
	return CAN_Ports[port].message_Objects[mbNum].mailboxState;
}

Uint32 getMailboxID(char port, char mbNum){
	return CAN_Ports[port].message_Objects[mbNum].mailbox->MSGID.all;
}



//---------------------------------------------------------------------------
// InitCanx:
//---------------------------------------------------------------------------
// These function initializes the eCAN module to a known state.
//

void InitCanA(Uint16 baud){		// Initialize eCAN-A module

/* Create a shadow register structure for the CAN control registers. This is
 needed, since only 32-bit access is allowed to these registers. 16-bit access
 to these registers could potentially corrupt the register contents or return
 false data. This is especially true while writing to/reading from a bit
 (or group of bits) among bits 16 - 31 */

struct ECAN_REGS ECanaShadow;

	EALLOW;		// EALLOW enables access to protected bits

/* Configure eCAN RX and TX pins for CAN operation using eCAN regs*/

    ECanaShadow.CANTIOC.all = ECanaRegs.CANTIOC.all;
    ECanaShadow.CANTIOC.bit.TXFUNC = 1;
    ECanaRegs.CANTIOC.all = ECanaShadow.CANTIOC.all;

    ECanaShadow.CANRIOC.all = ECanaRegs.CANRIOC.all;
    ECanaShadow.CANRIOC.bit.RXFUNC = 1;
    ECanaRegs.CANRIOC.all = ECanaShadow.CANRIOC.all;

/* Configure eCAN for HECC mode - (reqd to access mailboxes 16 thru 31) */
									// HECC mode also enables time-stamping feature

	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.SCB = 1;
	ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

/* Initialize all bits of 'Master Control Field' to zero */
// Some bits of MSGCTRL register come up in an unknown state. For proper operation,
// all bits (including reserved bits) of MSGCTRL must be initialized to zero

    ECanaMboxes.MBOX0.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX1.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX2.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX3.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX4.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX5.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX6.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX7.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX8.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX9.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX10.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX11.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX12.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX13.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX14.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX15.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX16.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX17.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX18.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX19.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX20.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX21.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX22.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX23.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX24.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX25.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX26.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX27.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX28.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX29.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX30.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX31.MSGCTRL.all = 0x00000000;

// TAn, RMPn, GIFn bits are all zero upon reset and are cleared again
//	as a matter of precaution.

	ECanaRegs.CANTA.all	= 0xFFFFFFFF;	/* Clear all TAn bits */

	ECanaRegs.CANRMP.all = 0xFFFFFFFF;	/* Clear all RMPn bits */

	ECanaRegs.CANGIF0.all = 0xFFFFFFFF;	/* Clear all interrupt flag bits */
	ECanaRegs.CANGIF1.all = 0xFFFFFFFF;


/* Configure bit timing parameters for eCANA*/
	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.CCR = 1 ;            // Set CCR = 1
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    ECanaShadow.CANES.all = ECanaRegs.CANES.all;

    do{

	    ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    }while(ECanaShadow.CANES.bit.CCE != 1 );  		// Wait for CCE bit to be set..

    ECanaShadow.CANBTC.all = 0;

    /* The following block for all 150 MHz SYSCLKOUT (75 MHz CAN clock) - default. Bit rate = CANA_BAUD_kBPS
			   See Note at End of File */
	ECanaShadow.CANBTC.bit.BRPREG = (5000/baud)-1;
	ECanaShadow.CANBTC.bit.TSEG2REG = 2;
	ECanaShadow.CANBTC.bit.TSEG1REG = 10;

    ECanaShadow.CANBTC.bit.SAM = 1;
    ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;

    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.CCR = 0 ;            // Set CCR = 0
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    ECanaShadow.CANES.all = ECanaRegs.CANES.all;

    do{

       ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 0 ); 		// Wait for CCE bit to be  cleared..

/* Disable all Mailboxes  */
 	ECanaRegs.CANME.all = 0;		// Required before writing the MSGIDs

    EDIS;
}


void InitCanB(Uint16 baud){		// Initialize eCAN-B module

/* Create a shadow register structure for the CAN control registers. This is
 needed, since only 32-bit access is allowed to these registers. 16-bit access
 to these registers could potentially corrupt the register contents or return
 false data. This is especially true while writing to/reading from a bit
 (or group of bits) among bits 16 - 31 */

struct ECAN_REGS ECanbShadow;

   EALLOW;		// EALLOW enables access to protected bits

/* Configure eCAN RX and TX pins for CAN operation using eCAN regs*/

    ECanbShadow.CANTIOC.all = ECanbRegs.CANTIOC.all;
    ECanbShadow.CANTIOC.bit.TXFUNC = 1;
    ECanbRegs.CANTIOC.all = ECanbShadow.CANTIOC.all;

    ECanbShadow.CANRIOC.all = ECanbRegs.CANRIOC.all;
    ECanbShadow.CANRIOC.bit.RXFUNC = 1;
    ECanbRegs.CANRIOC.all = ECanbShadow.CANRIOC.all;

/* Configure eCAN for HECC mode - (reqd to access mailboxes 16 thru 31) */

	ECanbShadow.CANMC.all = ECanbRegs.CANMC.all;
	ECanbShadow.CANMC.bit.SCB = 1;
	ECanbRegs.CANMC.all = ECanbShadow.CANMC.all;

/* Initialize all bits of 'Master Control Field' to zero */
// Some bits of MSGCTRL register come up in an unknown state. For proper operation,
// all bits (including reserved bits) of MSGCTRL must be initialized to zero

    ECanbMboxes.MBOX0.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX1.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX2.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX3.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX4.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX5.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX6.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX7.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX8.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX9.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX10.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX11.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX12.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX13.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX14.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX15.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX16.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX17.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX18.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX19.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX20.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX21.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX22.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX23.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX24.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX25.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX26.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX27.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX28.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX29.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX30.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX31.MSGCTRL.all = 0x00000000;

// TAn, RMPn, GIFn bits are all zero upon reset and are cleared again
//	as a matter of precaution.

	ECanbRegs.CANTA.all	= 0xFFFFFFFF;	/* Clear all TAn bits */

	ECanbRegs.CANRMP.all = 0xFFFFFFFF;	/* Clear all RMPn bits */

	ECanbRegs.CANGIF0.all = 0xFFFFFFFF;	/* Clear all interrupt flag bits */
	ECanbRegs.CANGIF1.all = 0xFFFFFFFF;


/* Configure bit timing parameters for eCANB*/

	ECanbShadow.CANMC.all = ECanbRegs.CANMC.all;
	ECanbShadow.CANMC.bit.CCR = 1 ;            // Set CCR = 1
    ECanbRegs.CANMC.all = ECanbShadow.CANMC.all;

    ECanbShadow.CANES.all = ECanbRegs.CANES.all;

    do{

	    ECanbShadow.CANES.all = ECanbRegs.CANES.all;
	} while(ECanbShadow.CANES.bit.CCE != 1 ); 		// Wait for CCE bit to be  cleared..


    ECanbShadow.CANBTC.all = 0;

	/* The following block for all 150 MHz SYSCLKOUT (75 MHz CAN clock) - default. Bit rate = 1 Mbps
	   See Note at end of file */
	ECanbShadow.CANBTC.bit.BRPREG = (5000/baud)-1;
	ECanbShadow.CANBTC.bit.TSEG2REG = 2;
	ECanbShadow.CANBTC.bit.TSEG1REG = 10;


    ECanbShadow.CANBTC.bit.SAM = 1;
    ECanbRegs.CANBTC.all = ECanbShadow.CANBTC.all;

    ECanbShadow.CANMC.all = ECanbRegs.CANMC.all;
	ECanbShadow.CANMC.bit.CCR = 0 ;            // Set CCR = 0
    ECanbRegs.CANMC.all = ECanbShadow.CANMC.all;

    ECanbShadow.CANES.all = ECanbRegs.CANES.all;

    do{

        ECanbShadow.CANES.all = ECanbRegs.CANES.all;
    } while(ECanbShadow.CANES.bit.CCE != 0 ); 		// Wait for CCE bit to be  cleared..


/* Disable all Mailboxes  */
 	ECanbRegs.CANME.all = 0;		// Required before writing the MSGIDs

    EDIS;
}
