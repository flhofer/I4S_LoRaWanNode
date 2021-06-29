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

#define freqPlan TTN_FP_EU868
#define POLL_NO		5			// How many times to poll
#define MAXLORALEN	242			// maximum payload length 0-51 for DR0-2, 115 for DR3, 242 otherwise
#define LORACHNMAX	16
#define LORABUSY	-4			// error code for busy channel
#define RESFREEDEL	40000		// ~resource freeing delay ETSI requirement air-time reduction
#define MACHDRFTR	13			// Length in bytes of MACHDR + FHDR + FPORT + MIC

// Modem constructor
static TheThingsNetwork ttn(loraSerial, debugSerial,
					freqPlan, TTN_DEFAULT_SF, TTN_DEFAULT_FSB);

static uint8_t actBands = 2;	// active channels
static int	pollcnt;			// un-conf poll retries

static unsigned long rnd_contex;	// pseudo-random generator context (for reentrant)
static byte genbuf[MAXLORALEN];		// buffer for generated message

static uint32_t startSleepTS;	// relative MC time of Sleep begin
static uint32_t timerMillisTS;	// relative MC time for timers
static uint32_t startTestTS;	// relative MC time for test start
static uint32_t sleepMillis;	// Time to remain in sleep
static uint32_t rxWindow1 = 1000; // pause duration in ms between tx and rx TODO: get parameter
static uint32_t rxWindow2 = 1000; // pause duration in ms between rx1 and rx2 TODO: get parameter

static const sLoRaConfiguration_t * conf;	// Pointer to configuration entry
static sLoRaResutls_t * trn;				// Pointer to actual entry
static enum {	iIdle,
				iSend,
				iPoll,
				iRetry,
				iBusy,
				iChnWait,
				iSleep,
			} internalState;

static unsigned long wdt;			// watch-dog timeout timer value, 15000 default
static unsigned long pollTstamp;	// last poll time-stamp

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

	for (int i=0; i < conf->dataLen; i++, payload++)
		*payload=(byte)(random_r(&rnd_contex) % 255);

	return payload;
}

/*************** CALLBACK FUNCTIONS ********************/

static void onMessage(const uint8_t *payload, size_t size, port_t port) __attribute__((unused));

/*
 * onMessage: Callback function for LoraRx
 * Arguments: - Byte vector with payload
 * 			  - vector length
 * 			  - LoRaWan Port
 * Return:	  -
 */
static void
onMessage(const uint8_t *payload, size_t size, port_t port){
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
static void
onBeforeTx(){
	timerMillisTS = millis();
	trn->timeTx = 0;
	trn->timeRx = 0;
	trn->timeToRx = 0;
}

/*
 * onAfterTx: Callback function for after LoRa TX
 * Arguments: -
 *
 * Return:	  -
 */
static void
onAfterTx(){
	trn->timeTx = getTimer();
}

/*
 * onAfterTx: Callback function for after LoRa RX
 * Arguments: -
 *
 * Return:	  -
 */
static void onAfterRx(){
	trn->timeToRx = getTimer();
	trn->timeRx = trn->timeToRx - trn->timeTx - rxWindow1;
	if (trn->timeRx > rxWindow2)
		trn->timeRx -= rxWindow2;
}

/*
 * computeAirTime:
 *
 * Arguments: - payload length
 * 			  - data rate (7-12)
 *
 * Return:	  - expected airTime in ms
 */
static uint32_t
computeAirTime(uint8_t dataLen, uint8_t dataRate){

	static const uint32_t dataRates[] =  {250, 440, 980, 1760, 3125, 5470};

	dataLen+=MACHDRFTR;

	return dataLen * 1000 / dataRates[Max(12-dataRate, 0)];
}

/*
 * setTxPwr: set power index on modem
 *
 * Arguments: - used mode, 0-..4
 * 			  - txPwr 0-5..
 *
 * Return:	  - return 0 if OK, -1 if error
 */
static int
setTxPwr(uint8_t mode, uint8_t txPwr){

	return ttn.setPowerIndex(txPwr)? 0 : -1;
}

/*
 * getChannels:
 *
 * Arguments: - pointer to channel enable bit mask to fill, 0 off, 1 on
 *
 * Return:	  - return 0 if OK, -1 if error
 */
static int
getChannels(uint16_t * chnMsk){

  *chnMsk = 0;

  for (int i=0; i<LORACHNMAX; i++)
	  *chnMsk |= (uint16_t)ttn.getChannelStatus((uint8_t)i) << i;

  return (0 == *chnMsk) * -1; // error if mask is empty!
}

/*
 * setChannelsCnf:
 *
 * Arguments: - channel enable bit mask, 0 off, 1 on
 *
 * Return:	  - return 0 if OK, -1 if error
 */
static int
setChannelsCnf(uint8_t drMin, uint8_t drMax){

  bool retVal = true;
  long frq = 0;

  for (int i=0; i<LORACHNMAX; i++){
	  // default TTN only channels 1-8 are set
	  if (i >= 8)
		  frq = 864100000 + 200000 * (i - 8);

	  retVal &= ttn.setChannel((uint8_t)i, frq, drMin, drMax);
	  retVal &= ttn.setChannelDCycle((uint8_t)i, 100.0);
  }

  return !retVal * -1;
}

/*
 * setChannels: set the Channel-Mask and the wanted data rate
 *
 * Arguments: - pointer to channel enable bit mask to use, 0 off, 1 on
 * 			  - dataRate for test start, (disables ADR)
 *
 * Return:	  - return 0 if OK, -1 if error
 */
static int
setChannels(uint16_t chnMsk, uint8_t dataRate) {

	bool retVal = true;

	for (int i=0; i<LORACHNMAX; i++, chnMsk >>=1){
	  retVal &= ttn.setChannelStatus((uint8_t)i, (bool)chnMsk & 0x01);
	}

	if (dataRate == 255){
		retVal &= ttn.setDR(5);
		retVal &= ttn.reset(true);
	}
	else {
		retVal &= ttn.reset(false);
		retVal &= ttn.setDR(dataRate);
	}
	retVal &=  !setChannelsCnf(0 , 5);

	return !retVal * -1;
}

/*
 * loRaJoin: Join a LoRaWan network
 *
 * Arguments: - pointer to test configuration to use
 *
 * Return:	  returns 0 if successful, else -1
 */
static int
loRaJoin(const sLoRaConfiguration_t * newConf){ // TODO: this resets ADR -> set here !
	if (newConf->confMsk & CM_OTAA)
		return !ttn.join(newConf->appEui, newConf->appKey) * -1;
	else
		return !ttn.personalize(newConf->devAddr, newConf->nwkSKey, newConf->appSKey) * -1;
}

/*
 * setupLoRaWan: setup LoRaWan communication with modem
 *
 * Arguments: - pointer to test configuration to use
 *
 * Return:	  returns 0 if successful, else -1
 */
static int
setupLoRaWan(const sLoRaConfiguration_t * newConf){

	if (!modem.begin(freqPlan)) {
		debugSerial.println("Failed to start module");
		return -1;
	};

	int ret = 0;
	ret |= !modem.dutyCycle(newConf->confMsk & CM_DTYCL); // switch off the duty cycle
	ret |= !modem.setADR(false);	// disable ADR by default

	modem.publicNetwork(!(newConf->confMsk & CM_NPBLK));

	if (!(newConf->confMsk & CM_RJN) && loRaJoin(newConf))
	{
		// Something went wrong; are you indoor? Move near a window and retry
		debugSerial.println("Network join failed");
		return -1;
	}

	// Set poll interval to 1 sec.
	modem.minPollInterval(1); // for testing only

	if (!(newConf->confMsk & CM_OTAA)){
		// set to LorIoT standard RX, DR
		ret |= !modem.setRX2Freq(869525000);
		ret |= !modem.setRX2DR(0);
	}

	return ret *-1;
}

/*
 * setupDumb: setup LoRa communication with modem
 *
 * Arguments: - pointer to test configuration to use
 *
 * Return:	  - return 0 if OK, -1 if error
 */
static int
setupDumb(const sLoRaConfiguration_t * newConf){

	modem.dumb();

	// Configure LoRa module to transmit and receive at 915MHz (915*10^6)
	// Replace 915E6 with the frequency you need (eg. 433E6 for 433MHz)
	if (!LoRa.begin((long)newConf->frequency * 100000)) {
		debugSerial.println("Starting LoRa failed!");
		return -1;
	}

	LoRa.setSpreadingFactor(newConf->spreadFactor);
	LoRa.setSignalBandwidth(newConf->bandWidth*1000);
	LoRa.setCodingRate4(newConf->codeRate);

	return 0;
}

/*
 * setupPacket: setup LoRa packet parameters communication with modem
 *
 * Arguments: - pointer to test configuration to use
 *
 * Return:	  - return 0 if OK, -1 if error
 */
static int
setupPacket(const sLoRaConfiguration_t * newConf){
	int ret = 0;
	ret |= !modem.setTxConfirmed(!(newConf->confMsk & CM_UCNF));
	ret |= !modem.setPort(2 + ((newConf->confMsk & CM_UCNF) >> 3));
	return ret * -1;

}

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

/*************** TEST SEND FUNCTIONS ********************/

/*
 * LoRaMgmtSendConf: send a confirmed message. If no response arrives
 * 		within timeout, return 1 (busy)
 *
 * Arguments: -
 *
 * Return:	  status of sending, 0 ok, -1 error, 1 busy
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

	// set to LorIoT standard RX, DR = 0, not default
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
 * LoRaMgmtSetup: setup LoRaWan communication with modem
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int LoRaMgmtSetupDumb(long FRQ){

}


//
//if ((ret = LoRaMgmtSend()) && ret != 1){
//	if (-9 == ret) // no chn -> pause for free-delay / active channels
//		delay(RESFREEDEL/actChan);
//	else
//		delay(100); // simple retry timer 100ms, e.g. busy
//	break;
//}
//
//// sent but no response from confirmed, or not confirmed msg, goto poll
//if (ret == 1 || !confirmed){
//	tstate = rRun;
//	printPrgMem(PRTSTTTBL, PRTSTTPOLL);
//
//}
//else {
//	tstate = rStop;
//	printPrgMem(PRTSTTTBL, PRTSTTSTOP);
//	break;
//}
//// fall-through
//// @suppress("No break at end of case")

//if ((ret = LoRaMgmtPoll()) && (confirmed || (pollcnt < UNCF_POLL))){
//	if (-9 == ret) // no chn -> pause for free-delay / active channels
//		delay(RESFREEDEL/actChan);
//	else if (1 == ret)
//		pollcnt++;
//	else
//		delay(100); // simple retry timer 100ms, e.g. busy
//	break;
//}
//
//// Unconf polling ended and still no response, or confirmed and error message (end of retries)
//if ((failed = (0 != ret)))
//	printPrgMem(PRTSTTTBL, PRTSTTPOLLERR);
//
//tstate = rStop;
//printPrgMem(PRTSTTTBL, PRTSTTSTOP);
//// fall-through
//// @suppress("No break at end of case")

//retries++;
//pollcnt=0;
//
//// unsuccessful and retries left?
//if (failed && (TST_RETRY > retries)){
//	tstate = rStart;
//	printPrgMem(PRTSTTTBL, PRTSTTRETRY);
//	delay(RESFREEDEL/actChan); // delay for modem resource free
//	(void)LoRaMgmtUpdt();
//	break;
//}
//
//tstate = rEvaluate;
//printPrgMem(PRTSTTTBL, PRTSTTEVALUATE);
//// fall-through
//// @suppress("No break at end of case")

/*
 * LoRaMgmtGetResults: getter for last experiment results
 *
 * Arguments: - result structure pointer
 *
 * Return:	  - 0 if OK, < 0 = error, 0 = busy, 1 = done, 2 = stop
 */
int
LoRaMgmtGetResults(sLoRaResutls_t ** const res){
	if (!trn)
		return -1;
	int ret = 0;
	trn->testTime = millis() - startTestTS;
	if (conf->mode == 1){
		trn->txFrq = conf->frequency*100000;
		trn->lastCR = conf->codeRate;
		trn->txDR = conf->spreadFactor;
		trn->txPwr = conf->txPowerTst;
	}
	else{
		trn->txFrq = ttn.getFrequency();
		ret |= LoRaGetChannels(&res->chnMsk);
		trn->lastCR = ttn.getCR();
		trn->txDR = ttn.getDR();
		trn->txPwr = ttn.getPower();
		trn->rxRssi = ttn.getRSSI();
		trn->rxSnr = ttn.getSNR();
	}
	*res = trn++;// shift to next slot
	return (ret == 0) ? 1 : -1;
}

/*
 * LoRaMgmtJoin: Join a LoRaWan network
 *
 * Arguments: -
 *
 * Return:	  - returns < 0 = error, 0 = busy, 1 = done, 2 = stop
 */
int
LoRaMgmtJoin(){
	(void)LoRaMgmtSetup(conf, trn);
	(void)loRaJoin(conf);
	return 0;
}

/*
 * LoRaMgmtUpdt: Update LoRa message buffer
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int
LoRaMgmtUpdt(){
	if (internalState == iIdle){
		// Prepare PayLoad of x bytes
		(void)generatePayload(genbuf);

		pollcnt = 0;

		return 1;
	}

	return 0;
}

/*
 * LoRaMgmtRcnf: reset modem and reconfiguration
 *
 * Arguments: -
 *
 * Return:	  - returns < 0 = error, 0 = busy, 1 = done, 2 = stop
 */
int
LoRaMgmtRcnf(){
	if (internalState == iIdle){
//		internalState = iPoll; // wait for a poll timer before continuing to next step
//		if (conf)
//			ttn.reset(1); // reset with adr on

		return !LoRaMgmtSetup(conf, trn)? 1 :  -1;
	}
	return 0;
}

/*
 * LoRaMgmtGetEUI: get EUI of the micro-controller
 *
 * Arguments: -
 *
 * Return:	  - C string EUI
 */
const char*
LoRaMgmtGetEUI(){
	ttn.getHardwareEui(conf->devEui, 17);
	return conf->devEui;
}

/*************** MAIN CALL FUNCTIONS ********************/

/*
 * LoRaMgmtMain: state machine for the LoRa Control
 *
 * Arguments: -
 *
 * Return:	  -
 */
void
LoRaMgmtMain (){
	switch (internalState){

	case iIdle:
		break;
	case iSend:
	case iRetry:
		startSleepTS = millis();
		sleepMillis = 100;	// Sleep timer after send, minimum wait
		internalState = iSleep;
		break;
	case iPoll:
		startSleepTS = millis();
		trn->txDR = ttn.getDR();
		sleepMillis = rxWindow1 + rxWindow2 + computeAirTime(conf->dataLen, trn->txDR) + 1000; // e.g. ACK lost, = 2+-1s (random)
		internalState = iSleep;
		break;
	case iBusy:	// Duty cycle = 1% chn [1-3], 0.1% chn [4-8]  pause = T/dc - T
		startSleepTS = millis();
		trn->txDR = ttn.getDR();
		sleepMillis = rxWindow1 + rxWindow2 + computeAirTime(conf->dataLen, trn->txDR);
		internalState = iSleep;
		break;
	case iChnWait:
		startSleepTS = millis();
		trn->txDR = ttn.getDR();
		{
			uint32_t timeAir = computeAirTime(conf->dataLen, trn->txDR);
			sleepMillis =  timeAir * 100 - timeAir; // This is for Channel 1-3, others * 1000
		}
		internalState = iSleep;
		break;
	case iSleep:
		if (millis() - startSleepTS > sleepMillis)
			internalState = iIdle;
	}
}
