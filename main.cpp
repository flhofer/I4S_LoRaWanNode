/*
 * main.cpp
 *
 *  Created on: May 25, 2020
 *      Author: Florian Hofer
 */

#include "main.h"

#include <util/atomic.h>		// Protected block routines
#include <avr/eeprom.h>			// EEPROM functions

#include "LoRaMgmt.h"			// LoRaWan modem management

#define UNCF_POLL	5			// How many times to poll
#define UNCF_RETRY	5			// Hoe many times retry to send unconfirmed message
#define RESFREEDEL	30000		// ~resource freeing delay ETSI requirement air-time reduction

/* EEPROM address */

 uint8_t EEMEM ee_bootCnt;	// reboot counter

/* Locals 		*/

static uint8_t actChan = 16;		// active channels

/* 	Globals		*/

int debug = 1;

// Test data structure
typedef struct _testParam{
	unsigned long *results;	// Data points TODO: change to LL
	size_t resultsSize;		// size of data points
	int counter;			// How many times to repeat
	int syncCode;			// DO sync code with jamming devices
	int (*init)(void);		// Hardware and RF preparation code
	int (*start)(void);		// Start of test
	int (*run)(void);		// dwelling status, i.e., wait for resutls
	int (*stop)(void);		// closing state
	int (*evaluate)(void);	// Computation of result
	int (*reset)(void);		// Reset steps for a new run
} testParam_t;

/*************** TEST FUNCTION CALL ********************/

static int
TT_InitMono(){

	// Setup channels to MonoChannel
	debugSerial.println("Init - channel config");

	actChan = 1;
	return LoRaSetChannels(0x01); // Enable channel 1 only;

}

static int
Aeval(){

	return 0;
}


/*************** TEST CONFIGURATIONS ********************/

// Test definition
static testParam_t testA1 = {
	NULL, 0, 2, 0,
	&TT_InitMono,
	&LoRaMgmtSend,
	&LoRaMgmtPoll,
	NULL,
	&Aeval,
	NULL,
};

static testParam_t testA2 = {
	NULL, 0, 0, 0,
	&TT_InitMono,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

// Test group definition
static testParam_t * testGrpA[] = {
		&testA1,
		&testA2,
		NULL // Terminator for automatic sizing
};

static testParam_t * testGrpB[] = {
		NULL // Terminator for automatic sizing
};

static testParam_t * testGrpC[] = {
		NULL // Terminator for automatic sizing
};

// All tests grouped
static testParam_t **testConfig[] = { // array of testParam_t**
		testGrpA, // array of testParam_t* (by reference), pointer to first testParam_t* in array
		testGrpB,
		testGrpC,
		NULL // Terminator for automatic sizing
};

/*************** TEST MANAGEMENT FUNCTIONS*****************/

// Enumeration for test status
enum testRun { 	rError = -1,
				rInit = 0,
				rStart,
				rRun,
				rStop,
				rEvaluate,
				rReset,
				rEnd = 10
			};

static enum testRun tstate = rInit;
static bool confirmed = false;	// TODO: implement menu and switch
static int dataLen = 1; 	// TODO: implement menu and switch
static int retries; 		// un-conf send retries
static int pollcnt;			// un-conf poll retries

/*
 * writeSyncState: set sync for companion devices
 *
 * Arguments: - code to send to the other participants in test
 *
 * Return:	  -
 */
static void
writeSyncState(int syncCode){

	;
}


/*
 * runTest: test runner
 *
 * Arguments: - pointer to test structure of actual test
 *
 * Return:	  - test run enumeration status
 */
static enum testRun
runTest(testParam_t * testNow){

	if (!testNow){
		debugSerial.println("WARN: Invalid test configuration");
		return rError;
	}

	int ret = 0;
	switch(tstate){

	case rInit:

		if (testNow->init)
			if ((ret = testNow->init()))
				break;

		writeSyncState(testNow->syncCode);

		// Set global test parameters
		LoRaSetGblParam(confirmed, dataLen);
		retries=0;
		pollcnt=0;

		tstate = rStart;
		// no break
		// fall-through

	case rStart:

		if (testNow->start)
			if ((ret = testNow->start()) && ret!=1){ // next if modem is busy
				if (ret == -2)
					delay(RESFREEDEL/actChan); // test if busy= sent anyway
				break;
			}

		tstate = rRun;
		// no break
		// fall-through

	case rRun:

		if (testNow->run)
			if ((ret = testNow->run()) && (pollcnt < UNCF_POLL || confirmed)){
				pollcnt++; // TODO: differentiate no response with not possible to tx/rx.
				break;
			}

		// Unconf polling ended and still busy? -> no response during poll retries
		if (ret == 1 && !confirmed){
			debugSerial.println("Poll - No response from server.");
			retries++;
			pollcnt=0;
			if (UNCF_RETRY > retries){
				tstate = rStart;
				delay(RESFREEDEL/actChan); // delay for modem resource free
				break;
			}

		}

		tstate = rStop;
		// no break
		// fall-through

	case rStop:
		if (testNow->stop)
			if ((ret = testNow->stop()))
				break;

		tstate = rEvaluate;
		// no break
		// fall-through

	case rEvaluate:
		if (testNow->evaluate)
			if ((ret = testNow->evaluate()))
				break;

		debugSerial.println("Evaluate - add measurement");

//		if (!realloc(testNow->results, (testNow->resultsSize+1) * sizeof(unsigned long) )){
//			debugSerial.println("re-alloc error!"); // TODO: avoid dynamic memory allocation
//			return rError;
//		}
//
//		testNow->results[testNow->resultsSize] = LoRaMgmtGetTime();
//		testNow->resultsSize++;

		sLoRaResutls_t * res;
		res =  LoRaMgmtGetResults();
		debugSerial.print("CR 4/");
		debugSerial.print(res->lastCR);
		debugSerial.print(" SF ");
		debugSerial.print(res->txSF);
		debugSerial.print(" BW ");
		debugSerial.print(res->txBW);
		debugSerial.println("kHz ");
		debugSerial.print("Freq [hz] ");
		debugSerial.print(res->txFrq);
		debugSerial.print(" rxBW ");
		debugSerial.print(res->rxBw);
		debugSerial.print(" pwr [dBm] ");
		debugSerial.print(res->txPwr);
		debugSerial.print(" rssi ");
		debugSerial.print(res->rxRssi);
		debugSerial.print(" snr ");
		debugSerial.println(res->rxSnr);

		// Test repeats?
		if (--testNow->counter <= 0){
			tstate = rEnd;
			break;
		}

		tstate = rReset;
		// no break
		// fall-through

	case rReset:
		if (testNow->reset)
			if ((ret = testNow->reset()))
				break;

		//delay(RESFREEDEL/actChan); // delay for modem resource free
		tstate = rInit;
		// no break

	default:
	case rEnd:
		;
	}

	if (ret == -1 ){
		debugSerial.println("Error during state execution");
		tstate = rEnd;
	}

	return tstate;
}

// TODO: hide test state from globals
static testParam_t ** tno = NULL,
					*** tgrp = NULL;

static unsigned long startTs = 0; // loop timer

/*
 * selectTest: test organization and selection
 *
 * Arguments: -
 *
 * Return:	  -
 */
static void
selectTest(){

	// end of tests groups?
	if (!*tgrp){
		debugSerial.println("End of test groups");

		while(1); // END PROGRAM HERE
		return;
	}

	enum testRun res = runTest(*tno);

	if (res == rError)
		debugSerial.println("ERROR: test malfunction");

	// test ended
	if ((res == rEnd || res == rError) && *tno){
		debugSerial.println("Skip to next test ");
		tno++;
		tstate = rInit; // TODO: fix internal

		debugSerial.println("-- LOOP 10 Seconds");

		// compute time to 10 seconds, aware of unsigned
		unsigned long wait = 10000 - min(10000, (millis() - startTs));
		delay(wait);
		startTs = millis();
	}

	// end of tests of test group?
	if (!*tno){
		if (*tgrp){
			tgrp++; // next test group
			tno = *tgrp;
			debugSerial.println("Skip to next test group ");
			startTs = millis();
		}
	}


}

/*************** SYSTEM SETUP AND LOOP *****************/

void initVariant(){
	// board dependent settings for cross-compatibility
	;
}

void setup()
{
	// Initialize Serial ports 0
	if (debug) // don't even try if not set
	{
		// setup serial for debug, disable if no connection after 10 seconds
		debugSerial.begin(9600);
		int waitSE = 999;	// wait for 10 seconds, -10 ms for computation
		while (!debugSerial && ((waitSE))) {
		  // delay(10); -- Included in Serial_::operator()
		  waitSE--;
		}
		debug &= waitSE;	// reset debug flag if time is elapsed
	}

	uint8_t val = 0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		val = eeprom_read_byte(&ee_bootCnt);
	}

	debugSerial.print("Reboot counter read from device: ");
	debugSerial.println(val);

//	if (!debug)
//		ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
//			eeprom_update_byte(&ee_bootCnt, ++val);
//		}

	LoRaMgmtSetup();

	tgrp = &testConfig[0]; 	// assign pointer to pointer to TestgroupA
	tno = *tgrp; 			// assign pointer to pointer to test 1

	startTs = millis();		// snapshot starting time
}

void loop()
{
  // call test selector
  selectTest();

  debugSerial.read();
}
