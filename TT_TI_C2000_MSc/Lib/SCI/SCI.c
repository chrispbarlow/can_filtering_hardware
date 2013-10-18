//#######################################################################################
//
// FILE:    SCI.c
//
// TITLE:   Serial Control Interface Library code. .
//
//
//#######################################################################################
// Based on: 	$TI Release: 2833x/2823x Header Files and Peripheral Examples V133 $
// 				$Release Date: June 8, 2012 $
//#######################################################################################


#include "SCI.h"


// Test 1,SCIA  DLB, 8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
void scia_init()
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

 	SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
	SciaRegs.SCICTL2.all =0x0003;
	SciaRegs.SCICTL2.bit.TXINTENA =0;
	SciaRegs.SCICTL2.bit.RXBKINTENA =0;
	SciaRegs.SCIFFRX.bit.RXFFIENA = 1;	//Use receive FIFO

	#if (CPU_FRQ_150MHZ)
	      SciaRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 37.5MHz.
	      SciaRegs.SCILBAUD    =0x00E7;
	#endif
	#if (CPU_FRQ_100MHZ)
      SciaRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 20MHz.
      SciaRegs.SCILBAUD    =0x0044;
	#endif
	SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

// Transmit a character from the SCI
void scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF=a;

}

void scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}

// Transmit a character from the SCI
Uint16 scia_xmitTT(int a)
{
	Uint16 ret = 0;

    if (SciaRegs.SCIFFTX.bit.TXFFST == 0) {
    	SciaRegs.SCITXBUF=a;
    	ret = 1;
    }

    return ret;
}

Uint16 scia_msgTT(char * msg)
{
    static Uint16 i = 0;
    Uint16 ret = 0;

    ret = scia_xmitTT(msg[i]);

    if(msg[i] != '\0'){
        i++;
    }
    else{
    	i = 0;
    	ret = 2;
    }

    return ret;
}

// Initalize the SCI FIFO
void scia_fifo_init()
{
    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.all=0x204f;
    SciaRegs.SCIFFCT.all=0x0;

}






//===========================================================================
// No more.
//===========================================================================

