/*
 * SCI.h
 *
 *  Created on: 25 Jun 2013
 *      Author: chris.barlow
 */

#ifndef SCI_H_
#define SCI_H_
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <DSP2833x_Sci.h>

void sciTest(void);
// Prototype statements for functions found within this file.
void scia_init(void);
void scia_fifo_init(void);
void scia_xmit(int a);
void scia_msg(char *msg);
Uint16 scia_xmitTT(int a);
Uint16 scia_msgTT(char *msg);

#endif /* SCI_H_ */
