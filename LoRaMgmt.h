/*
 * LoRaMgmt.h
 *
 *  Created on: Nov 3, 2020
 *      Author: Florian Hofer
 */

#ifndef LORAMGMT_H_
#define LORAMGMT_H_

#include <stdint.h>

#define LORACHNMAX 8

typedef struct sLoRaResutls {
	uint32_t timeTx;
	uint32_t timeRx;
	uint32_t timeToRx;
	uint32_t txFrq;			// current used frequency
	uint8_t lastCR;			// Coding rate 4/x
	uint8_t txBW;			// current bandwidth in kHz
	uint8_t txDR;			// Tx data rate
	int8_t txPwr;			// Tx power index used
	int8_t rxRssi;			// last rx RSSI, default -128
	uint8_t rxSnr;			// last rx SNR, default -128
} sLoRaResutls_t;


void LoRaMgmtSetup();
void LoRaSetGblParam(bool confirm, int datalen);

int LoRaSetChannels(uint16_t chnMsk);

int LoRaMgmtSend();
int LoRaMgmtPoll();
int LoRaMgmtUpdt();
int LoRaMgmtRcnf();

int LoRaMgmtGetResults(sLoRaResutls_t * res);



#endif /* LORAMGMT_H_ */
