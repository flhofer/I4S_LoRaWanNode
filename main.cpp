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

// TODO: hide test state from globals
int tno = 0,
	tgrp = 0;

int debug = 1;


/*************** TEST FUNCTION CALL ********************/

static int
Ainit(){

	return 0;
}

/*************** TEST CONFIGURATIONS ********************/

// Test data structure
typedef struct _testParam{
	unsigned long *resutls;
	size_t resutlsSize;
	int (*init)(void);
	int (*prepare)(void);
	int (*run)(void);
	int (*stop)(void);
	int (*evaluate)(void);
} testParam_t;

// Test definition
testParam_t testA1 = {
	NULL, 0,
	&Ainit,
	NULL,
	NULL,
	NULL,
	NULL
};

testParam_t testA2 = {
	NULL, 0,
	&Ainit,
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

	switch(tstate){

	case rInit:

		if (testNow->init)
			if (testNow->init())
				break;

		tstate = rPrepare;
		// no break
		// fall-through

	case rPrepare:

		if (testNow->prepare())
			if (testNow->prepare())
				break;

		tstate = rRun;
		// no break
		// fall-through

	case rRun:

		if (testNow->run())
			if (testNow->run())
				break;

		tstate = rStop;
		// no break
		// fall-through

	case rStop:
		if (testNow->stop())
			if (testNow->stop())
				break;

		tstate = rEvaluate;
		// no break
		// fall-through

	case rEvaluate:
		if (testNow->evaluate())
			if (testNow->evaluate())
				break;

		tstate = rEnd;
		// no break

	default:
	case rEnd:
		;
	}

	return tstate;
}

/*
 * selectTest: test organization and selection
 *
 * Arguments: -
 *
 * Return:	  -
 */
static void
selectTest(){

	runTest(NULL);

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

}

void loop()
{
  debugSerial.println("-- LOOP 10 Seconds");

  // call test selector
  selectTest();


  delay(10000); // TODO: Class C must scan all the time
  debugSerial.read();
}
