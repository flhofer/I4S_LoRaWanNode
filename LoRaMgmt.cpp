/*
 * LoRaMgmt.cpp
 *
 *  Created on: Nov 3, 2020
 *      Author: Florian Hofer
 */

#include "LoRaMgmt.h"
#include "main.h"				// Global includes/definitions, i.e. address, key, debug mode
#include "timer.h"				// Custom Timer, stop-watch routines
#include "TheThingsNetwork.h"	// LoRaWan library by TTN
#include <stdlib.h>				// AVR standard library

// DevAddr, NwkSKey, AppSKey and the frequency plan
static const char *devAddr = LORA_DEVADDR;
static const char *nwkSKey = LORA_NWSKEY;
static const char *appSKey = LORA_APSKEY;

// Select frequency plan between TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_EU868
#define MAXLORALEN	51			// maximum payload length 0-51 for DR0-2, 115 for DR3, 242 otherwise

// Modem constructor
static TheThingsNetwork ttn(loraSerial, debugSerial,
					freqPlan, TTN_DEFAULT_SF, TTN_DEFAULT_FSB);

static bool conf = false;			// use confirmed messages
static int dataLen = 1; 			// TX data length for tests
static unsigned long rnd_contex;	// pseudo-random generator context (for reentrant)
static unsigned long rxWindow1 = 1000; // pause duration in ms between tx and rx TODO: get parameter
static unsigned long rxWindow2 = 2000; // pause duration in ms between tx and rx2 TODO: get parameter
static unsigned long wdt;			// watch-dog timeout timer value, 15000 default
static unsigned long pollTstamp;	// last poll time-stamp

static uint32_t timeTx;				// Last TX
static uint32_t timeRx;				// Last RX
static uint32_t timeToRx;			// Last Total time

static byte genbuf[MAXLORALEN];			// buffer for generated message

/********************** HELPERS ************************/

/*
 * generatePayload: fills a buffer with dataLen random bytes
 *
 * Arguments: - Byte vector for payload
 *
 * Return:	  - next open position (end of buffer)
 */
static byte *
generatePayload(byte *payload){

	// TODO: unprotected memory
	for (int i=0; i < dataLen; i++, payload++)
		*payload=(byte)(random_r(&rnd_contex) % 255);

	return payload;
}

/*************** CALLBACK FUNCTIONS ********************/

/*
 * onMessage: Callback function for LoraRx
 * Arguments: - Byte vector with payload
 * 			  - vector length
 * 			  - LoRaWan Port
 * Return:	  -
 */
static void onMessage(const uint8_t *payload, size_t size, port_t port){
	if (!debug)
		return;

	debugSerial.print("Port: ");
	debugSerial.print(port);
	debugSerial.print(" Size: ");
	debugSerial.print(size);
	debugSerial.print(" time: ");
	debugSerial.print(0); // TODO: get time
	debugSerial.print(" string: ");
	char * s = (char *) malloc (size*2);
	if (s)
		for (unsigned int i = 0; i< size; i++){
			{
			    if (i > 0) printf(":");
			    sprintf(s+(i*2),"%02X", payload[i]);
			}
		}
	debugSerial.println(s);
	free (s);

// TODO: improve, use unformatted printing!
//    const char * hex = "0123456789ABCDEF";
//    char * pout = str;
//    int i = 0;
//    for(; i < sizeof(buf)-1; ++i){
//        *pout++ = hex[(*pin>>4)&0xF];
//        *pout++ = hex[(*pin++)&0xF];
//        *pout++ = ':';
//    }
//    *pout++ = hex[(*pin>>4)&0xF];
//    *pout++ = hex[(*pin)&0xF];
//    *pout = 0;
}

/*
 * onBeforeTx: Callback function for before LoRa TX
 * Arguments: -
 *
 * Return:	  -
 */
static void onBeforeTx(){
	startTimer();
	timeTx = 0;
	timeRx = 0;
	timeToRx = 0;
}

/*
 * onAfterTx: Callback function for after LoRa TX
 * Arguments: -
 *
 * Return:	  -
 */
static void onAfterTx(){
	timeTx = getTimer();
}

/*
 * onAfterTx: Callback function for after LoRa RX
 * Arguments: -
 *
 * Return:	  -
 */
static void onAfterRx(){
	timeToRx = getTimer();
	timeRx = timeToRx - timeTx - rxWindow1;
	if (timeRx > 1000)
		timeRx -= rxWindow2;
}

/*************** TEST SEND FUNCTIONS ********************/

static int
evaluateResponse(int ret){
	switch (ret) {

	  // TX only
	case TTN_SUCCESSFUL_TRANSMISSION:
	  // ACK receive ok
	case TTN_SUCCESSFUL_RECEIVE:
	  return 0;

	case TTN_UNSUCESSFUL_RECEIVE:
		int tn;
		tn = ttn.getStatus();
		debugSerial.print("Status-Modem ");
		debugSerial.println(tn);
		if (tn == TTN_MDM_IDLE)
			return 0; // Listening but Nothing left to send?
		else
			return 1; // Maybe still outstanding response

	default:
	case TTN_ERROR_UNEXPECTED_RESPONSE:
	  debugSerial.println("Unable to send payload!\n");
	  return -999;

	case TTN_ERROR_SEND_COMMAND_FAILED:
	  return (int)ttn.getLastError(); // transform error code to int -> forward to main
	}
}

/*
 * LoRaMgmtSendConf: send a confirmed message. If no response arrives
 * 		within timeout, return 1 (busy)
 *
 * Arguments: -
 *
 * Return:	  status of polling, 0 ok, -1 error, 1 busy
 */
int LoRaMgmtSend(){

	// TODO: get rid of local buffer genbuf
	byte payload[dataLen];
	(void)memcpy(payload, genbuf, dataLen);

	// Send it off
	ttn_response_t ret = ttn.sendBytes(payload, sizeof(payload), 1, conf);
	pollTstamp = millis();
	return evaluateResponse(ret);
}

/*
 * LoRaMgmtPoll: poll function for confirmed and delayed TX
 *
 * Arguments: -
 *
 * Return:	  status of polling, 0 ok, -1 error, 1 busy
 */
int LoRaMgmtPoll(){

	// compute time, wait for window length at least seconds, aware of unsigned
	unsigned long wait = (rxWindow1 + rxWindow2) - min(rxWindow1 + rxWindow2, (millis() - pollTstamp));
	delay(wait);

	// set modem to true to read only modem.
	ttn_response_t ret = ttn.poll(1, conf, conf);

	pollTstamp = millis();

	return evaluateResponse(ret);
}

/*************** MANAGEMENT FUNCTIONS ********************/

/*
 * LoRaGetChannels:
 *
 * Arguments: - pointer to channel enable bit mask to fill, 0 off, 1 on
 *
 * Return:	  - return 0 if OK, -1 if error
 */
static int
LoRaGetChannels(uint16_t * chnMsk){

  *chnMsk = 0;

  for (int i=0; i<LORACHNMAX; i++)
	  *chnMsk |= ttn.getChannelStatus((uint8_t)i) << i;

  return (0 == *chnMsk) * -1; // error if mask is empty!
}

/*
 * LoRaMgmtGetResults: getter for last experiment results
 *
 * Arguments: - pointer to Structure for the result data
 *
 * Return:	  - 0 if ok, <0 error
 */
int
LoRaMgmtGetResults(sLoRaResutls_t * res){
	res->timeTx = timeTx;
	res->timeRx = timeRx;
	res->timeToRx = timeToRx;
	res->txFrq = ttn.getFrequency();
	(void)LoRaGetChannels(&res->chnMsk);
	res->lastCR = ttn.getCR();
	res->txDR = ttn.getDR();
	res->txPwr = ttn.getPower();
	res->rxRssi = ttn.getRSSI();
	res->rxSnr = ttn.getSNR();
	return 0;
}

/*
 * LoRaMgmtSetup: setup LoRaWan communication with modem
 *
 * Arguments: -
 *
 * Return:	  -
 */
void LoRaMgmtSetup(){

	installTimer(); // setup timer1 registers

	// Initialize Serial1
	loraSerial.begin(57600);

	debugSerial.println("-- PERSONALIZE");
	ttn.personalize(devAddr, nwkSKey, appSKey);

//	ttn.setClass(CLASS_C);
	ttn.onMessage(&onMessage);
	ttn.onBeforeTx(&onBeforeTx);
	ttn.onAfterTx(&onAfterTx);
	ttn.onAfterRx(&onAfterRx);

	debugSerial.println("-- STATUS");
	ttn.showStatus();

	wdt = ttn.getWatchDogTimer();

	debugSerial.print("Watchdog timer set to [ms] ");
	debugSerial.println(wdt);

	// set to LorIoT standard RX, DR = 0
	ttn.setRx2Channel(869525000, 0);
}

/*
 * LoRaSetGblParam: set generic parameters, re-init random seed
 *
 * Arguments: - confirmed send yes/no
 * 			  - simulated payload length
 *
 * Return:	  -
 */
void LoRaSetGblParam(bool confirm, int datalen){
	conf = confirm;
	// set boundaries for len value
	dataLen = max(min(datalen, MAXLORALEN), 1);

	// initialize random seed with datalen as value
	// keep consistency among tests, but differs with diff len
	srandom(dataLen);
	// Prepare PayLoad of x bytes
	(void)generatePayload(genbuf);
}

/*
 * LoRaSetChannels:
 *
 * Arguments: - channel enable bit mask, 0 off, 1 on
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int LoRaSetChannels(uint16_t chnMsk, uint8_t drMin, uint8_t drMax){

  bool retVal = true;
  long frq = 0;

  for (int i=0; i<LORACHNMAX; i++, chnMsk >>=1){
	  // default only chnannels 1-8 are set
	  if (i >= 8)
		  frq = 864100000 + 200000 * (i - 8);

	  retVal &= ttn.setChannel((uint8_t)i, frq, drMin, drMax);
	  retVal &= ttn.setChannelStatus((uint8_t)i, (bool)chnMsk & 0x01);
  }

  return !retVal * -1;
}

/*
 * LoRaMgmtUpdt: update LoRa message buffer
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int LoRaMgmtUpdt(){
	// Prepare PayLoad of x bytes
	(void)generatePayload(genbuf);

	return 0;
}

/*
 * LoRaMgmtRcnf: reset modem and reconf
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int LoRaMgmtRcnf(){
	if (conf)
		ttn.reset(1); // reset with adr on

	return 0;
}

/*
 * LoRaMgmTxPwr: set power index on modem
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int LoRaMgmTxPwr(uint8_t txPwr){

	ttn.setPowerIndex(txPwr);

	return 0;
}


