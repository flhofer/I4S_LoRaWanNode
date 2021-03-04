/*
 * LoRaMgmt.h
 *
 *  Created on: Nov 3, 2020
 *      Author: Florian Hofer
 */

#ifndef LORAMGMT_H_
#define LORAMGMT_H_

#include <stdint.h>
#include "main.h"

#define LORACHNMAX 8

void LoRaMgmtSetup();
void LoRaSetGblParam(bool confirm, int datalen);

int LoRaSetChannels(uint16_t chnMsk);

int LoRaMgmtSend();
int LoRaMgmtPoll();
int LoRaMgmtUpdt();
int LoRaMgmtRcnf();
int LoRaMgmTxPwr(uint8_t txPwr);

int LoRaMgmtGetResults(sLoRaResutls_t * res);



#endif /* LORAMGMT_H_ */
