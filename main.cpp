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

int tno = 0,
	tgrp = 0,
	tstate = 0;

int debug = 1;


/*************** TEST CONFIGURATIONS ********************/

void Ainit(){

}

/*************** TEST CONFIGURATIONS ********************/

typedef struct _testParam{
	unsigned long *resutls;
	size_t resutlsSize;
	void (*init)(void);
	void (*prepare)(void);
	void (*run)(void);
	void (*stop)(void);
	void (*evaluate)(void);
} testParam_t;

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

testParam_t **testConfig[] = { // array of testParam_t**
		testGrpA, // array of testParam_t* (by reference), pointer to first testParam_t* in array
		testGrpB,
		testGrpC,
		NULL // Terminator for automatic sizing
};

/*************** TEST MANAGEMENT FUNCTIONS*****************/

enum testRun { 	rInit,
				rPrepare,
				rRun,
				rStop,
				rEvaluate
			};


void runTest(int maxTest){

	switch(tstate){

	case rInit:

		tstate++;
		break;

	case rPrepare:

		tstate++;
		break;

	case rRun:

		tstate++;
		break;

	case rStop:

		tstate++;
		break;

	case rEvaluate:

		tstate++;
		break;

	}
}

void selectTest(){


}

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
