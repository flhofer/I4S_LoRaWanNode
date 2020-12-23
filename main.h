/*
 * main.h
 *
 *  Created on: May 25, 2020
 *      Author: Florian Hofer
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#include "Arduino.h" // Arduino Framework CORE -- I/O & Serial ports

// Serial connection definition
#define debugSerial Serial		// USB Serial
#define loraSerial Serial1		// Hardware serial

// EUI TTN Uno, ATmega32u4 0004A30B0021DD46
// DevAddr, NwkSKey, AppSKey and the frequency plan
#define LORA_DEVADDR	"01234567"
#define LORA_NWSKEY		"01234567890abcdef01234567890abcd"
#define LORA_APSKEY		"01234567890abcdef01234567890abcd"

// global types
typedef struct sLoRaResutls {
	uint32_t timeTx;
	uint32_t timeRx;		// TODO: one of the two may be removed
	uint32_t timeToRx;
	uint32_t txFrq;			// current used frequency
	uint16_t chnMsk;		// Concluding channel mask
	uint8_t lastCR;			// Coding rate 4/x
	uint8_t txDR;			// Tx data rate
	int8_t txPwr;			// Tx power index used TODO: check if we store index or power dBm
	int8_t rxRssi;			// last rx RSSI, default -128
	uint8_t rxSnr;			// last rx SNR, default -128
} sLoRaResutls_t;

//global variable declarations
extern int debug;				// Global flag Debug console attached (PC)

//Do not add code below this line
#endif /* _MAIN_H_ */
