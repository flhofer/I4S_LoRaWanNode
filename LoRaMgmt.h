/*
 * LoRaMgmt.h
 *
 *  Created on: Nov 3, 2020
 *      Author: Florian Hofer
 */

#ifndef LORAMGMT_H_
#define LORAMGMT_H_

#include <Stream.h>

void LoRaMgmtSetup();

int LoRaMgmtSendConf();

int LoRaMgmtPoll();

unsigned long LoRaMgmtGetTime();


#endif /* LORAMGMT_H_ */
