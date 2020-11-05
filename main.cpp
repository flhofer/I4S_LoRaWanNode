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

uint8_t EEMEM ee_bootCnt;	// reboot counter

int debug = 1;

// Test data structure
typedef struct _testParam{
	unsigned long *results;
	size_t resultsSize;
	int counter;
	int (*init)(void);
	int (*prepare)(void);
	int (*run)(void);
	int (*stop)(void);
	int (*evaluate)(struct _testParam * testNow);
	int (*reset)(void);
} testParam_t;

/*************** TEST FUNCTION CALL ********************/

static int
Ainit(){

	// Setup channels to MonoChannel
	debugSerial.println("Init - channel config");
	return 0;
}

static int
Aeval(testParam_t * testNow){
	debugSerial.println("Evaluate - add measurement");

	if (!realloc(testNow->results, (testNow->resultsSize+1) * sizeof(unsigned long) )){
		debugSerial.println("realloc error!");
		return -1;
	}

	testNow->results[testNow->resultsSize] = LoRaMgmtGetTime();
	testNow->resultsSize++;

	return 0;
}


/*************** TEST CONFIGURATIONS ********************/

// Test definition
testParam_t testA1 = {
	NULL, 0, 2,
	&Ainit,
	&LoRaMgmtSend,
	&LoRaMgmtPoll,
	NULL,
	&Aeval,
	NULL,
};

testParam_t testA2 = {
	NULL, 0, 0,
	&Ainit,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

// Test group definition
testParam_t * testGrpA[] = {
		&testA1,
		&testA2,
		NULL // Terminator for automatic sizing
};

testParam_t * testGrpB[] = {
		NULL // Terminator for automatic sizing
};

testParam_t * testGrpC[] = {
		NULL // Terminator for automatic sizing
};

// All tests grouped
testParam_t **testConfig[] = { // array of testParam_t**
		testGrpA, // array of testParam_t* (by reference), pointer to first testParam_t* in array
		testGrpB,
		testGrpC,
		NULL // Terminator for automatic sizing
};

/*************** TEST MANAGEMENT FUNCTIONS*****************/


// Enumeration for test status
enum testRun { 	rError = -1,
				rInit = 0,
				rPrepare,
				rRun,
				rStop,
				rEvaluate,
				rReset,
				rEnd = 10
			};

enum testRun tstate = rInit;

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

		tstate = rPrepare;
		// no break
		// fall-through

	case rPrepare:

		if (testNow->prepare)
			if ((ret = testNow->prepare())
					&& ret != 1) // skip if busy
				break;

		tstate = rRun;
		// no break
		// fall-through

	case rRun:

		if (testNow->run)
			if ((ret = testNow->run()))
				break;

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
			if ((ret = testNow->evaluate(testNow)))
				break;

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

		tstate = rEnd;
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
testParam_t ** tno = NULL,
			*** tgrp = NULL;

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
		delay(10000); // TODO: Class C must scan all the time
	}

	// end of tests of test group?
	if (!*tno){
		if (*tgrp){
			tgrp++; // next test group
			tno = *tgrp;
			debugSerial.println("Skip to next test group ");
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
		debug = !(waitSE);	// reset debug flag if time is elapsed
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
}

void loop()
{
  // call test selector
  selectTest();

  debugSerial.read();
}
