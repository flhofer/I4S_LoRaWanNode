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

// Select frequency plan between TTN_FP_EU868 or TTN_FP_US915
#define CM_OTAA			1		// LORAWAN use OTAA join instead of ABP
#define CM_DTYCL		2		// LORAWAN disable duty cycle
#define CM_RJN	 		4		// LORAWAN rejoin if failed
#define CM_UCNF			8		// LORAWAN use unconfirmed messages
#define CM_NPBLK		16		// LORAWAN use not public network
#define CM_NRST			32		// LORAWAN do not reset after successful send

#define KEYSIZE			32		// 32

/**
  * LoRa(Wan) Configuration
  */
typedef struct
{
	uint8_t	mode  = 0;			// test mode = 0 off, 1 LoRa, 2 LoRaWan, 3 LoRaWan + Remote, 4 LoRaWan Force Join
	uint8_t confMsk;			// Configuration mask bits

	// Common all Modes
	uint8_t txPowerTst = 0;		// txPower setting for the low power test
	uint8_t dataLen = 1;		// data length to send over LoRa for a test
	uint8_t repeatSend = 5;		// number of send repeats
	uint16_t rxWindow1 = 1000; // pause duration in ms between tx and rx, default 1 sec
	uint16_t rxWindow2 = 2000; // pause duration in ms between tx and rx2 default 2 sec
	union { // 16Bit
		uint16_t frequency;		// LoRa / FSK frequency in 100KHz steps
		uint16_t chnMsk;		// ChannelMask for LoRaWan EU868 (1-16)
	};
	union { // 8Bit
		uint8_t dataRate = 255;	// data rate starting value for LoRaWan, 255 = leave unchanged
		uint8_t spreadFactor;	// spread factor for Chirp signals FSK/LORA
	};

	// LoRaWan (OTAA vs ABP) or LoRa/FSK settings
	union { // 32Bit
		uint8_t bandWidth;		// Bandwidth setting for LoRa & FSK TODO: create steps
		char * devEui = NULL;   // Device EUI OTAA
		char * devAddr;         // Device Address ABP
	};
	union { // 32Bit
		uint8_t	codeRate;		// Code rate for LoRa & FSK, in 4/
		char * appEui = NULL;   // App EUI OTAA
		char * nwkSKey;         // Nw Session key ABP
	};
	union { // 32Bit
		char * appKey = NULL;   // App KEY OTAA
		char * appSKey;         // App Session key ABP
	};

	// function pointers / callback for runtime. < 0 = error, 0 = busy, 1 = done, 2 = stop
	int (*prep)() = NULL;
	int (*start)() = NULL;
	int (*run)() = NULL;

} sLoRaConfiguration_t;

//add your function definitions for the project LoRaWanTest here
// global types
typedef struct {
	uint8_t txCount;		// transmission counter
	uint32_t testTime;		// total test time for this run
	uint32_t timeTx;		// time for TX
	uint32_t timeRx;		// time for RX
	uint32_t timeToRx;		// total time until response
	uint32_t txFrq;			// current used frequency
	uint16_t chnMsk;		// Concluding channel mask
	uint8_t  lastCR;		// Coding rate 4/x
	uint8_t  txDR;			// Tx data rate
	int8_t   txPwr;			// Tx power index used
	uint8_t   rxRssi;		// last rx RSSI, abs, default (-)255
	int8_t   rxSnr;			// last rx SNR, default -128
} sLoRaResutls_t;

void LoRaMgmtMain();

int LoRaMgmtSetup(const sLoRaConfiguration_t * conf, sLoRaResutls_t * const result);
int LoRaMgmtJoin();

int LoRaMgmtSend();
int LoRaMgmtSendDumb();
int LoRaMgmtPoll();
int LoRaMgmtRemote();

int LoRaMgmtGetResults(sLoRaResutls_t ** const res);

const char* LoRaMgmtGetEUI(const sLoRaConfiguration_t * newConf);
int LoRaMgmtUpdt();
int LoRaMgmtRcnf();

#endif /* LORAMGMT_H_ */
