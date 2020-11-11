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

//TODO: change to STDINT formats
typedef struct sLoRaResutls {
	uint32_t timeTx;
	uint32_t timeRx;
	uint32_t timeToRx;
	uint32_t txFrq;			// current used frequency
	uint8_t lastCR;			// Coding rate 4/x
	uint8_t txBW;			// current bandwidth in kHz
	uint8_t txSF;			// Tx spread factor, SF7 to 12
	int8_t txPwr;			// Tx power used in dBm
	int8_t rxRssi;			// last rx RSSI, default -128
	uint8_t rxSnr;			// last rx SNR, default -128
} sLoRaResutls_t;


void LoRaMgmtSetup();
void LoRaSetGblParam(bool confirm, int datalen);

int LoRaSetChannels(uint16_t chnMsk);

int LoRaMgmtSend();
int LoRaMgmtPoll();

sLoRaResutls_t * LoRaMgmtGetResults();



#endif /* LORAMGMT_H_ */
