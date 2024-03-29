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
#define POLL_NO		0			// How many times to poll for uncnf
#define MAXLORALEN	242			// maximum payload length 0-51 for DR0-2, 115 for DR3, 242 otherwise
#define LORACHNMAX	16
#define LORABUSY	-4			// error code for busy channel
#define RESFREEDEL	40000		// ~resource freeing delay ETSI requirement air-time reduction
#define MACHDRFTR	13			// Length in bytes of MACHDR + FHDR + FPORT + MIC

#define MIN(a, b)	(a < b) ? a : b

// Modem constructor
static TheThingsNetwork ttn(loraSerial, debugSerial,
					freqPlan, TTN_DEFAULT_SF, TTN_DEFAULT_FSB);

static int	pollcnt;			// un-conf poll retries

static uint32_t startSleepTS;	// relative MC time of Sleep begin
static uint32_t startTestTS;	// relative MC time for test start
static uint32_t sleepMillis;	// Time to remain in sleep

static const sLoRaConfiguration_t * conf;	// Pointer to configuration entry
static sLoRaResutls_t * trn;				// Pointer to actual entry
static enum {	iIdle,
				iSend,
				iPoll,
				iRetry,
				iBusy,
				iChnWait,
				iSleep,
				iRndWait,
			} internalState;

/********************** HELPERS ************************/
static uint8_t
countSetBits(int n)
{
	uint8_t count = 0;
	while (n) {
		n &= (n - 1);
		count++;
	}
	return count;
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
 * onMessageRemote: Callback function for LoraRx on LoRaRemote incoming Message
 * Arguments: - Byte vector with payload
 * 			  - vector length
 * 			  - LoRaWan Port
 * Return:	  -
 */
static void
onMessageRemote(const uint8_t *payload, size_t size, port_t port){

	if (size == 1){ // one letter
		switch(*payload){
		case 'R':
			pollcnt = 1;
			return;
		case 'S':
			pollcnt = 2;
			return;
		}
	}

	debugSerial.print("Invalid message, ");
	onMessage(payload, size, port);
}

/*
 * onBeforeTx: Callback function for before LoRa TX
 * Arguments: -
 *
 * Return:	  -
 */
static void
onBeforeTx(){
	trn->timeTx = 0;
	trn->timeRx = 0;
	trn->timeToRx = 0;
	startTimer();
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
	trn->timeRx = trn->timeToRx - trn->timeTx - ((uint32_t)conf->rxWindow1)*1000;
}

/*
 * computeAirTime:
 *
 * Arguments: - payload length
 * 			  - data rate (0-5)
 *
 * Return:	  - expected airTime in ms
 */
static uint32_t
computeAirTime(uint8_t dataLen, uint8_t dataRate){

	static const uint32_t dataRates[] =  {250, 440, 980, 1760, 3125, 5470};

	dataLen+=MACHDRFTR;

	return (uint32_t)dataLen * 8000l / dataRates[MIN(dataRate, 5)];
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
setTxPwr( __attribute__((unused)) uint8_t mode, uint8_t txPwr){

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
setChannelsCnf(const sLoRaConfiguration_t * const newConf, uint8_t drMin, uint8_t drMax){

  bool retVal = true;
  long frq = 0;
  uint16_t chnMsk = newConf->chnMsk;

  for (int i=0; i<LORACHNMAX; i++, chnMsk >>=1){
	  // default TTN only channels 1-8 are set
	  if (i >= 8)
		  frq = 864100000 + 200000 * (i - 8);

	  retVal &= ttn.setChannel((uint8_t)i, frq, drMin, drMax);

	  if (!(chnMsk & 0x01))
		retVal &= ttn.setChannelDCycle((uint8_t)i, 0);			// off
	  else if (newConf->confMsk & CM_DTYCL)
		retVal &= ttn.setChannelDCycle((uint8_t)i, 100.0);		// no DCycle (testing)
	  else{
		  // ETSI compliant duty cycle configuration
		  if (i < 3)
			  // Base Channels 1-3 common 1% band G1
			  retVal &= ttn.setChannelDCycle((uint8_t)i, 1.0/(float)countSetBits(newConf->chnMsk & 0x07));
		  else{
			  uint8_t bits;
			  if (i<8)
				  // Channels 4-8 common 0.1 % band G2
				  bits = countSetBits(newConf->chnMsk & 0xF8);
			  else
				  // Channels 9-16 common 0.1 % band G
				  bits = countSetBits(newConf->chnMsk & 0xFF00);

			  retVal &= ttn.setChannelDCycle((uint8_t)i, 0.1/(float)bits);
		  }
	  }
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
	  retVal &= ttn.setChannelStatus((uint8_t)i, (bool)(chnMsk & 0x01));
	}

	if (dataRate == 255){
		retVal &= ttn.setDR(5);
		retVal &= ttn.setADR(true);
	}
	else {
		retVal &= ttn.setADR(false);
		retVal &= ttn.setDR(dataRate);
	}

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
loRaJoin(const sLoRaConfiguration_t * const newConf){ // TODO: this resets ADR -> set here !
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
setupLoRaWan(const sLoRaConfiguration_t * const newConf){

	installTimer(); // setup timer1 registers

	int ret = 0;
	uint32_t fcu = 0;		// message up-counter
	uint32_t fcd = 0;		// message down-counter

	// update counters for next cycle
	fcu = ttn.getFCU();
	fcd = ttn.getFCD();

	// Initialize Serial1
	loraSerial.begin(57600);

	ret |= !ttn.setADR(false);

	if (!(newConf->confMsk & CM_RJN) && loRaJoin(newConf))
	{
		// Something went wrong; are you indoor? Move near a window and retry
		debugSerial.println("Network join failed");
		return -1;
	}

//	ttn.showStatus();
//	ttn.setClass(CLASS_C);

//	wdt = ttn.getWatchDogTimer();
//
//	debugSerial.print("Watchdog timer set to [ms] ");
//	debugSerial.println(wdt);

	if (!(newConf->confMsk & CM_OTAA)){
		// set to LorIoT standard RX, DR
		ret |= !ttn.setRx2Channel(869525000, 0);
		ret |= !ttn.setRX1Delay(newConf->rxWindow1);
	}	// set to LorIoT standard RX, DR = 0, not default

	ret |= !ttn.setFCU(fcu);
	ret |= !ttn.setFCD(fcd);

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
setupDumb( __attribute__((unused)) const sLoRaConfiguration_t * newConf){

	return 0;
}

/*
* evaluateResponse: evaluate modem response and inquiry more detail (if necessary)
*
* Arguments: - return value from modem send/poll
*
* Return:	  - 0 if done?, 1 if ok send, 2 if ok tx/rx, neg if error/busy
*/
static int
evaluateResponse(int ret){
	switch (ret) {

	  // TX only
	case TTN_SUCCESSFUL_TRANSMISSION:
	  // ACK receive ok
	case TTN_SUCCESSFUL_RECEIVE:
	  return ret;

	case TTN_UNSUCCESSFUL_RECEIVE:
		if (TTN_MDM_IDLE == ttn.getStatus())
			return TTN_MDM_IDLE; // All done, modem idle, however no ACK
		else
			return TTN_ERR_BUSY; // Maybe still outstanding response

	default:
	case TTN_ERROR_UNEXPECTED_RESPONSE:
	  return -999;

	case TTN_ERROR_SEND_COMMAND_FAILED: // -1 = busy... -12 = err
	  return (int)ttn.getLastError();
	}
}

/*************** TEST SEND FUNCTIONS ********************/

/*
 * LoRaMgmtSendDumb: send a message with the defined mode (dumb.. stub for now)
 *
 * Arguments: -
 *
 * Return:	  status of sending, < 0 = error, 0 = busy, 1 = done, 2 = stop
 */
int LoRaMgmtSendDumb(){
	return 0;
}

/*
 * LoRaMgmtSend: send a message with the defined mode
 *
 * Arguments: -
 *
 * Return:	  status of sending, < 0 = error, 0 = busy, 1 = done, 2 = stop
 */
int LoRaMgmtSend(){

	if (internalState == iIdle){
		internalState = iSend;

		int ret;
		{ // Send it off
			port_t port = 2 + ((conf->confMsk & CM_UCNF) >> 3);

			// initialize random seed with dataLen as value
			// keep consistency among tests, but differs with diff len
			unsigned long rnd_contex = conf->dataLen;	// pseudo-random generator context (for reentrant)
			byte genbuf[MAXLORALEN];					// buffer for generated message

			for (int i=0; i < conf->dataLen; i++)
				genbuf[i]=(byte)(random_r(&rnd_contex) % 255);

			ttn_response_t rsp = ttn.sendBytes(genbuf, conf->dataLen,
					port, !(conf->confMsk & CM_UCNF));
			ret = evaluateResponse(rsp);
		}

		if (ret < 0){
			if (TTN_ERR_BUSY == ret ){ // no channel available -> pause for free-delay / active channels
				internalState = iBusy;
				return !(conf->confMsk & CM_UCNF); // Cnf -> goto poll, Uncnf stay here
			}
			else if (TTN_ERR_NFRCHN == ret){ // MKR does not have it
				internalState = iChnWait;
				return 0;
			}
			return ret;
		}

		pollcnt = 0;
		internalState = iBusy;
		if (conf->repeatSend == 0)
			return 0; // If set to infinite, repeat send command until end

		if ((POLL_NO == 0 && (conf->confMsk & CM_UCNF))
				|| ((ret == TTN_SUCCESSFUL_RECEIVE || ret == TTN_SUCCESSFUL_TRANSMISSION)
						&& !(conf->confMsk & CM_UCNF))){
			// message ACK received or confirmed finished
			trn->txCount++;
			internalState = iIdle;
			return 2;
		}
		return 1;
	}
	return 0;	// else busy
}

/*
 * LoRaMgmtPoll: poll function for confirmed and delayed TX, check Receive
 *
 * Arguments: -
 *
 * Return:	  status of polling, < 0 = error, 0 = busy, 1 = done, 2 = stop
 */
int
LoRaMgmtPoll(){
	if (internalState == iIdle){

		if (!(conf->confMsk & CM_UCNF)){
			internalState = iRetry;

			// set modem to true to read only modem.
			ttn_response_t stat = ttn.poll(2, true, true);		// port 2 conf
			int ret = evaluateResponse(stat);

			if (ret == TTN_ERR_BUSY) // Not yet finished with retries
				return 0;

			internalState = iIdle;
			if (TTN_SUCCESSFUL_RECEIVE == ret
					|| TTN_SUCCESSFUL_TRANSMISSION == ret) {
				trn->txCount++;
				return 2; // Stop once received
			}
			return -1;
		}
		else{
			internalState = iPoll;

			// set modem to true to read only modem.
			ttn_response_t stat = ttn.poll(3, false, false);	// port 3 unconf
			int ret = evaluateResponse(stat);
			if (0 > ret){
				if (pollcnt < POLL_NO-1){
					if (TTN_ERR_BUSY == ret ) //
						internalState = iBusy;
					else if (TTN_ERR_NFRCHN == ret)   // no channel available -> pause for duty cycle-delay prop
						internalState = iChnWait;
					else
						internalState = iRetry;
					return 0;	// return 0 until count
				}
				pollcnt++;
				return -1;
			}

			pollcnt++;
			trn->txCount++;

			// read receive buffer
			if (TTN_SUCCESSFUL_RECEIVE == ret){
				// message received
				internalState = iIdle;
				return 1;
			}
			else {
				// No message received
				if (pollcnt < POLL_NO)
					return 0;
				return -1;
			}
		}
	}
	return 0;
}

/*
 * LoRaMgmtRemote: poll modem for go/stop commands
 *
 * Arguments: -
 *
 * Return:	  status of polling, < 0 = error, 0 = busy, 1 = done, 2 = stop
 */
int
LoRaMgmtRemote(){
	if (internalState == iIdle){
		internalState = iPoll;

		ttn_response_t stat =  ttn.poll(1, false,false);
		int ret = evaluateResponse(stat);

		if (ret < 0 && ret != TTN_ERR_BUSY && ret != TTN_ERR_NFRCHN)
			return ret;

		if (ret != TTN_SUCCESSFUL_RECEIVE){
			// No down-link message received at this time.
			return 0;
		}

		return pollcnt; // stores read result
	}
	return 0;
}


/*************** MANAGEMENT FUNCTIONS ********************/

/*
 * LoRaMgmtSetup: Setup LoRaWan communication with Modem
 *
 * Arguments: - result structure pointer to list of results
 *
 * Return:	  - returns 0 if successful, else -1
 */
int
LoRaMgmtSetup(const sLoRaConfiguration_t * newConf,
		sLoRaResutls_t * const result){

	int ret = 0;
	switch (newConf->mode){
	default:
	case 0: ;
			break;
	case 1: ret = setupDumb(newConf);
			break;
	case 2 ... 4:
			ret = setupLoRaWan(newConf);
			ret |= setChannelsCnf(newConf, 0, 5);
			ret |= setChannels(newConf->chnMsk, newConf->dataRate);
			if (newConf->repeatSend == 0)
				internalState = iRndWait;
	}
	ret |= setTxPwr(newConf->mode, newConf->txPowerTst);
	trn = result;

	pollcnt = 0;
	trn->txCount = 0;

	if (ret == 0){
		if (newConf->mode == 3)
			ttn.onMessage(&onMessageRemote);
		else
			ttn.onMessage(&onMessage);
		ttn.onBeforeTx(&onBeforeTx);
		ttn.onAfterTx(&onAfterTx);
		ttn.onAfterRx(&onAfterRx);

		conf = newConf;
	}
	startTestTS = millis();
	return ret;
}

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
		ret |= getChannels(&trn->chnMsk);
		trn->lastCR = ttn.getCR();
		trn->txDR = ttn.getDR();
		trn->txPwr = ttn.getPower();
		trn->rxRssi = (uint8_t)abs(ttn.getRSSI());
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
//		(void)generatePayload(genbuf);

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
		if (!(conf->confMsk & CM_NRST))
			return !LoRaMgmtSetup(conf, trn)? 1 :  -1;
		return 1;
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
LoRaMgmtGetEUI(const sLoRaConfiguration_t * newConf){
	// Initialize Serial1
	loraSerial.begin(57600);

	ttn.getHardwareEui(newConf->devEui, KEYSIZE+1);
	return newConf->devEui;
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
		sleepMillis =  (uint32_t)conf->rxWindow2 + computeAirTime(conf->dataLen, trn->txDR) + 1000; // e.g. ACK lost, = 2+-1s (random)
		internalState = iSleep;
		break;
	case iBusy:	// Duty cycle = 1% chn [1-3], 0.1% chn [4-8]  pause = T/dc - T
		startSleepTS = millis();
		sleepMillis = (uint32_t)conf->rxWindow1; // Min send time
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
	case iRndWait:
		startSleepTS = millis();
		{
			unsigned long rnd_contex = startSleepTS;
			sleepMillis = rand_r(&rnd_contex) % conf->rxWindow2;
		}
		internalState = iSleep;
		break;
	case iSleep:
		if (millis() - startSleepTS > sleepMillis)
			internalState = iIdle;
	}
}
