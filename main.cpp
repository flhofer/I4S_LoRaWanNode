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
#define TST_RETRY	5			// How many times retry to send message
#define TST_MXRSLT	40			// What's the max number of test results we allow?
#define RESFREEDEL	30000		// ~resource freeing delay ETSI requirement air-time reduction

/* EEPROM address */

 uint8_t EEMEM ee_bootCnt;	// reboot counter

/* Locals 		*/

static uint8_t actChan = 16;		// active channels
static sLoRaResutls_t testResults[TST_MXRSLT];	// Storage for test results
static sLoRaResutls_t * testResultNow;

/* 	Globals		*/

int debug = 1;

// Test data structure
typedef struct _testParam{
	sLoRaResutls_t *results;// Data points
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
	NULL,
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
	PORTB = (PORTB & 0x0F) | (syncCode << 4);
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
		debugSerial.println("Start");
		// fall-through
		// @suppress("No break at end of case")

	case rStart:

		if (testNow->start)
			if ((ret = testNow->start()) && ret != 1){ // next if modem is busy
				if (-9 == ret) // no chn -> pause for free-delay / active channels
					delay(RESFREEDEL/actChan);
				else
					delay(100); // simple retry timer 100ms, e.g. busy
				break;
			}

		debugSerial.print("Continue ");
		// sent but no response from confirmed, or not confirmed msg, goto poll
		if (ret == 1 || !confirmed){
			tstate = rRun;
			debugSerial.println("Poll for answer");
		}
		else {
			tstate = rStop;
			debugSerial.println("Stop test");
			break;
		}
		// fall-through
		// @suppress("No break at end of case")

	case rRun:

		if (testNow->run)
			if ((ret = testNow->run()) && (pollcnt < UNCF_POLL || confirmed)){
				if (-9 == ret) // no chn -> pause for free-delay / active channels
					delay(RESFREEDEL/actChan);
				else if (1 == ret)
					pollcnt++;
				else
					delay(100); // simple retry timer 100ms, e.g. busy
				break;
			}

		// Unconf polling ended and still no response, or confirmed and error message (end of retries)
		if ((!confirmed && 1 == ret) || (confirmed && 0 != ret))
			debugSerial.println("Poll - No response from server.");

		tstate = rStop;
		debugSerial.println("Stop test");
		// fall-through
		// @suppress("No break at end of case")

	case rStop:
		if (testNow->stop)
			if ((ret = testNow->stop()))
				break;

		retries++;
		pollcnt=0;

		if (TST_RETRY > retries){
			tstate = rStart;
			debugSerial.println("Retry");
			delay(RESFREEDEL/actChan); // delay for modem resource free
			break;
		}

		tstate = rEvaluate;
		debugSerial.println("Evaluate");
		// fall-through
		// @suppress("No break at end of case")

	case rEvaluate:
		if (testNow->evaluate)
			if ((ret = testNow->evaluate()))
				break;

		debugSerial.println(" - add measurement");

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
		debugSerial.print(" pwr [dBm] ");
		debugSerial.print(res->txPwr);
		debugSerial.print(" rssi ");
		debugSerial.print(res->rxRssi);
		debugSerial.print(" snr ");
		debugSerial.println(res->rxSnr);

		if (!testNow->results)
			testNow->results = testResultNow; // apply to first

		if (testResultNow < &testResults[TST_MXRSLT]){
			// Copy results to local counters, starts at the end
			(void)memcpy(testResultNow, res, sizeof(sLoRaResutls_t));
			testNow->resultsSize++;
			testResultNow++;
		}
		else
			debugSerial.println("*** STORAGE FULL ***\r\nNo space for further results");

		// Test repeats?
		if (--testNow->counter <= 0){
			tstate = rEnd;
			debugSerial.println("End test");
			break;
		}

		tstate = rReset;
		debugSerial.println("Reset");

		// fall-through
		// @suppress("No break at end of case")

	case rReset:
		if (testNow->reset)
			if ((ret = testNow->reset()))
				break;

		//delay(RESFREEDEL/actChan); // delay for modem resource free
		tstate = rInit;
		debugSerial.println("Restart - Init");
		break;

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
	DDRB |= 0xF0;
	PORTB = 0x00;

	// Led 13 out
	DDRC |= 0x80;
	PORTC &= 0x7F;
}

void setup()
{
	// Initialize Serial ports 0
	if (debug) // don't even try if not set
	{
		// setup serial for debug, disable if no connection after 10 seconds
		debugSerial.begin(9600);
		int waitSE = 999;	// wait for 10 seconds, -10 ms for computation
		while (!debugSerial && waitSE) {
		  //delay(10);// -- Included in Serial_::operator()
		  waitSE--;
		}
		debug = ((waitSE));	// reset debug flag if time is elapsed
	}

	// Blink once PIN13 to show program start
	PORTC |= (0x01 << PINC7);
	delay (500);
	PORTC &= ~(0x01 << PINC7);

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
	testResultNow = &testResults[0]; // Init results pointer

	startTs = millis();		// snapshot starting time
}

void loop()
{
  // call test selector
  selectTest();

  debugSerial.read();
}
