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
#define TST_MXRSLT	50			// What's the max number of test results we allow?
#define RESFREEDEL	40000		// ~resource freeing delay ETSI requirement air-time reduction

// Allocate initPorts in init section3 of code
void initPorts (void) __attribute__ ((naked)) __attribute__ ((section (".init3")));

/* EEPROM address */

 uint8_t EEMEM ee_confirmed;	// store confirmed mode yes-no
 uint8_t EEMEM ee_txPowerTst; 	// Tx power for the low power test
 uint8_t EEMEM ee_dataLenMin;	// Minimum data length for test
 uint8_t EEMEM ee_dataLenMax;	// Maximum data length for test


/* PROGMEM string lists */

const char prtSttStart[] PROGMEM = "Start test\n";
const char prtSttPoll[] PROGMEM = "Poll for answer\n";
const char prtSttStop[] PROGMEM = "Stop test\n";
const char prtSttRetry[] PROGMEM = "Retry\n";
const char prtSttEvaluate[] PROGMEM = "Evaluate\n";
const char prtSttAddMeas[] PROGMEM = " - add measurement\n";
const char prtSttReset[] PROGMEM = "Reset\n";
const char prtSttRestart[] PROGMEM = "Restart - Init\n";
const char prtSttEnd[] PROGMEM = "End test\n";
const char prtSttPollErr[] PROGMEM = "Poll - No response from server.\n";
const char prtSttDone[] PROGMEM = "done";
const char prtSttErrExec[] PROGMEM = "ERROR: during state execution\n";
const char prtSttErrText[] PROGMEM = "ERROR: test malfunction\n";
const char prtSttWrnConf[] PROGMEM = "WARN: Invalid test configuration\n";

PGM_P const prtSttStr[] PROGMEM = {prtSttStart, prtSttPoll, prtSttStop, prtSttRetry, prtSttEvaluate, prtSttAddMeas, prtSttReset, prtSttRestart, prtSttEnd, prtSttPollErr, prtSttDone, prtSttErrExec, prtSttErrText, prtSttWrnConf};

#define PRTSTTSTART 0
#define PRTSTTPOLL 1
#define PRTSTTSTOP 2
#define PRTSTTRETRY 3
#define PRTSTTEVALUATE 4
#define PRTSTTADDMEAS 5
#define PRTSTTRESET 6
#define PRTSTTRESTART 7
#define PRTSTTEND 8
#define PRTSTTPOLLERR 9
#define PRTSTTDONE 10
#define PRTSTTERREXEC 11
#define PRTSTTERRTEXT 12
#define PRTSTTWRNCONF 13

const char prtTblCR[] PROGMEM = " CR 4/";
const char prtTblDR[] PROGMEM = " DR ";
const char prtTblChnMsk[] PROGMEM = " MSK [EN] ";
const char prtTblFrq[] PROGMEM = " Freq [hz] ";
const char prtTblPwr[] PROGMEM = " pwr [dBm] ";
const char prtTblRssi[] PROGMEM = " rssi ";
const char prtTblSnr[] PROGMEM = " snr ";
const char prtTblTTx[] PROGMEM = " Time TX: ";
const char prtTblTRx[] PROGMEM = " Time RX: ";
const char prtTblTTl[] PROGMEM = " Time Total: ";
const char prtTblTms[] PROGMEM = " ms";

PGM_P const prtTblStr[] PROGMEM = {prtTblCR, prtTblDR, prtTblChnMsk, prtTblFrq, prtTblPwr, prtTblRssi, prtTblSnr, prtTblTTx, prtTblTRx, prtTblTTl, prtTblTms};

#define PRTTBLCR 0
#define PRTTBLDR 1
#define PRTTBLCHMSK 2
#define PRTTBLFRQ 3
#define PRTTBLPWR 4
#define PRTTBLRSSI 5
#define PRTTBLSNR 6
#define PRTTBLTTX 7
#define PRTTBLTRX 8
#define PRTTBLTTL 9
#define PRTTBLTMS 10

#define PRTSTTTBL 0
#define PRTTBLTBL 1

/* Locals 		*/

static sLoRaResutls_t testResults[TST_MXRSLT];	// Storage for test results
static sLoRaResutls_t * trn;					// Pointer to actual entry
static char prntGrp;							// Actual executing group
static int prntTno;								// actual executing testno
static uint8_t actChan = 16;					// active channels
static int testend = 1;							// is test terminated?
static uint8_t dataLen = 1;						// data length to send over LoRa for a test
static uint8_t dataLenMin = 1;					// Min data length to send over LoRa
static uint8_t dataLenMax = 255;				// Max data length to send over LoRa
static uint8_t txPowerTst = 4;					// Max data length to send over LoRa
static bool confirmed = true;					// TODO: implement menu and switch, BUT should it be changed?

static uint8_t testGrp = 1;						// Running variables number
static uint8_t testNo = 1;						// "	"	number

/* 	Globals		*/

int debug = 1;

// Test data structure
typedef struct _testParam{
	uint16_t chnEnabled;	// Channels enabled for this test
	uint8_t txPowerIdx;		// Initial TX power index
	uint8_t drMax;			// data rate Maximum for the test
	uint8_t drMin;			// data rate Minimum for the test
	// RX1 Window
	// RX1 delay
	// RX1 DR offset
	// RX2 Window
	// RX2 delay
} testParam_t;

/*************** TEST CONFIGURATIONS ********************/

// Test definition
static testParam_t testA1 = {
	0x01,	// channels
	0,		// TX
	5,		// DR max
	0		// DR min
};

// Test definition
static testParam_t testB1 = {
	0x01,	// channels
	4,		// TX
	5,		// DR max
	0		// DR min
};

static testParam_t testC1 = {
	0xFF,	// channels
	0,		// TX
	5,		// DR max
	0		// DR min
};

// Test group definition - A all remain the same
static testParam_t * testGrpA[] = {
		&testA1,
		&testC1,
		&testA1,
		&testA1,
		&testA1,
		NULL // Terminator for automatic sizing
};

static testParam_t * testGrpB[] = {
		&testA1,
		&testB1,
		&testA1,
		NULL // Terminator for automatic sizing
};

static testParam_t * testGrpC[] = {
		&testA1,
		&testA1,
		&testC1,
		&testC1,
		&testC1,
		NULL // Terminator for automatic sizing
};

// All tests grouped
static testParam_t **testConfig[] = { // array of testParam_t**
		testGrpA, // array of testParam_t* (by reference), pointer to first testParam_t* in array
		testGrpB,
		testGrpC,
		NULL // Terminator for automatic sizing
};


/*************** MIXED STUFF ********************/

static void
printScaled(uint32_t value, uint32_t Scale = 1000){
	debugSerial.print(value / Scale);
	debugSerial.print(".");
	value %= Scale;
	Scale /=10;
	while (value < Scale){
		debugSerial.print("0");
		Scale /=10;
	}
	if (value != 0)
		debugSerial.print(value);
}

static void
printPrgMem(int tbl, int pos){

	char buf[46];
	switch (tbl){

		default:
		case PRTSTTTBL:
			strcpy_P(buf,(PGM_P)pgm_read_word(&(prtSttStr[pos])));
			break;

		case PRTTBLTBL:
			strcpy_P(buf,(PGM_P)pgm_read_word(&(prtTblStr[pos])));
			break;
	}
	debugSerial.print(buf);

}

static void
printTestResults(){
	// use all local, do not change global
	sLoRaResutls_t * trn = &testResults[0]; // Init results pointer

	// for printing
	char buf[128];

	debugSerial.println("Results");
	for (int i = 1; i<= TST_MXRSLT; i++, trn++){
		sprintf(buf, "%c;%02d;%02d;%07lu;%07lu;0x%02X;%lu;%02u;%02d;%03d;%03d",
				prntGrp, prntTno, i, trn->timeTx, trn->timeRx,
				trn->chnMsk, trn->txFrq, trn->txDR, trn->txPwr,
				trn->rxRssi, trn->rxSnr);
				debugSerial.println(buf);
		}
}

uint8_t countSetBits(int n)
{
	uint8_t count = 0;
    while (n) {
        n &= (n - 1);
        count++;
    }
    return count;
}

void readEEPromSettings () {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		confirmed = eeprom_read_byte(&ee_confirmed);
		dataLenMin = eeprom_read_byte(&ee_dataLenMin);
		dataLenMax = eeprom_read_byte(&ee_dataLenMax);
		txPowerTst = eeprom_read_byte(&ee_txPowerTst);
	}
}

void writeEEPromDefaults() { // for defaults
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		eeprom_update_byte(&ee_confirmed, 1);
		eeprom_update_byte(&ee_dataLenMin, 1);
		eeprom_update_byte(&ee_dataLenMax, 255);
		eeprom_update_byte(&ee_txPowerTst, 4);
	}
}


static testParam_t ** tno = NULL;
static testParam_t *** tgrp = NULL;

static unsigned long startTs = 0; // loop timer

void readInput() {

	char A;
	while (debugSerial.available()){
		A = debugSerial.read();
		switch (A){
		// read parameter, they come together
		case 'g': // read test group
			A = debugSerial.read();
			A = A - 48;
			if (A < 5 && A >= 0)
				testGrp = A;
			break;
		case 't': // read test number to go
			A = debugSerial.read();
			A = A - 32;
			if (A < 10 && A >= 0)
				testNo = A;
			break;
		case 'u': // set to unconfirmed
			confirmed = false;
//			ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
//				eeprom_update_byte(&ee_confirmed, confirmed);
//			}
			break;
		case 'c': // set to confirmed
			confirmed = true;
//			ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
//				eeprom_update_byte(&ee_confirmed, confirmed);
//			}
			break;
		case 'p': // read tx power index
			A = debugSerial.read();
			A = A - 32;
			if (A < 10 && A >= 0){
				txPowerTst = A;
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
					eeprom_update_byte(&ee_txPowerTst, txPowerTst);
				}
			}
			break;

		case 'r': // set to run
			testend = false;

			// run through tests to pick the right test
			while (testGrp && *tgrp){
				tgrp++; // next test group
				testGrp--;
			}
			tno = *tgrp;
			while (testNo && *tno){
				tgrp++; // next test group
				testNo--;
			}
			startTs = millis();
			break;
		// TODO: add data length menu
		}
	}

}

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
static int	retries; 			// un-conf send retries
static int	pollcnt;			// un-conf poll retries

/*
 * runTest: test runner
 *
 * Arguments: - pointer to test structure of actual test
 *
 * Return:	  - test run enumeration status
 */
static enum testRun
runTest(testParam_t * testNow){

	int failed = 0;

	if (!testNow){
		printPrgMem(PRTSTTTBL, PRTSTTWRNCONF);
		return rError;
	}

	// reset status on next test
	if (rEnd == tstate || rError == tstate ){
		tstate = rInit; // we don't call with error/end
		memset(testResults,0, sizeof(testResults));
		trn = &testResults[0];	// Init results pointer
 	}

	int ret = 0;
	switch(tstate){

	case rInit:

		// Set global test parameters
		LoRaSetGblParam(confirmed, dataLen);
		retries=0;
		pollcnt=0;

		// Setup channels as configured
		actChan = countSetBits(testNow->chnEnabled);

		if (LoRaSetChannels(testNow->chnEnabled, testNow->drMin, testNow->drMax)) // set channels
			break; // TODO: error

		if (LoRaMgmtTxPwr(testNow->txPowerIdx)) // set power index;
			break; // TODO: error

		tstate = rStart;
		printPrgMem(PRTSTTTBL, PRTSTTSTART);
		// fall-through
		// @suppress("No break at end of case")

	case rStart:

		if ((ret = LoRaMgmtSend()) && ret != 1){
			if (-9 == ret) // no chn -> pause for free-delay / active channels
				delay(RESFREEDEL/actChan);
			else
				delay(100); // simple retry timer 100ms, e.g. busy
			break;
		}

		// sent but no response from confirmed, or not confirmed msg, goto poll
		if (ret == 1 || !confirmed){
			tstate = rRun;
			printPrgMem(PRTSTTTBL, PRTSTTPOLL);

		}
		else {
			tstate = rStop;
			printPrgMem(PRTSTTTBL, PRTSTTSTOP);
			break;
		}
		// fall-through
		// @suppress("No break at end of case")

	case rRun:

		if ((ret = LoRaMgmtPoll()) && (confirmed || (pollcnt < UNCF_POLL))){
			if (-9 == ret) // no chn -> pause for free-delay / active channels
				delay(RESFREEDEL/actChan);
			else if (1 == ret)
				pollcnt++;
			else
				delay(100); // simple retry timer 100ms, e.g. busy
			break;
		}

		// Unconf polling ended and still no response, or confirmed and error message (end of retries)
		if ((failed = (0 != ret)))
			printPrgMem(PRTSTTTBL, PRTSTTPOLLERR);

		tstate = rStop;
		printPrgMem(PRTSTTTBL, PRTSTTSTOP);
		// fall-through
		// @suppress("No break at end of case")

	case rStop:

		retries++;
		pollcnt=0;

		// unsuccessful and retries left?
		if (failed && (TST_RETRY > retries)){
			tstate = rStart;
			printPrgMem(PRTSTTTBL, PRTSTTRETRY);
			delay(RESFREEDEL/actChan); // delay for modem resource free
			(void)LoRaMgmtUpdt();
			break;
		}

		tstate = rEvaluate;
		printPrgMem(PRTSTTTBL, PRTSTTEVALUATE);
		// fall-through
		// @suppress("No break at end of case")

	case rEvaluate:
		printPrgMem(PRTSTTTBL, PRTSTTADDMEAS);

		(void)LoRaMgmtGetResults(trn); // TODO: implement and use return value
		// pgm_read_word = read char pointer address from PROGMEM pos PRTTBLCR of the string array
		// strcpy_P = copy char[] from PRROGMEM at that address of PRROGMEM to buf
		// *.print = print that char to serial
		printPrgMem(PRTTBLTBL,PRTTBLCR);
		debugSerial.print(trn->lastCR);
		printPrgMem(PRTTBLTBL,PRTTBLDR);
		debugSerial.print(trn->txDR);
		printPrgMem(PRTTBLTBL,PRTTBLCHMSK);
		debugSerial.println(trn->chnMsk);
		printPrgMem(PRTTBLTBL,PRTTBLFRQ);
		debugSerial.print(trn->txFrq);
		printPrgMem(PRTTBLTBL,PRTTBLPWR);
		debugSerial.print(trn->txPwr);
		printPrgMem(PRTTBLTBL,PRTTBLRSSI);
		debugSerial.print(trn->rxRssi);
		printPrgMem(PRTTBLTBL,PRTTBLSNR);
		debugSerial.println(trn->rxSnr);

		printPrgMem(PRTTBLTBL,PRTTBLTTX);
		printScaled(trn->timeTx);
		printPrgMem(PRTTBLTBL,PRTTBLTMS);
		printPrgMem(PRTTBLTBL,PRTTBLTRX);
		printScaled(trn->timeRx);
		printPrgMem(PRTTBLTBL,PRTTBLTMS);
		printPrgMem(PRTTBLTBL,PRTTBLTTL);
		printScaled(trn->timeToRx);
		printPrgMem(PRTTBLTBL,PRTTBLTMS);
		debugSerial.println();

		// End of tests?
		if (trn >= &testResults[TST_MXRSLT-1]){
			tstate = rEnd;
			printPrgMem(PRTSTTTBL, PRTSTTEND);
			printTestResults();
			break;
		}

		trn++;
		tstate = rReset;
		printPrgMem(PRTSTTTBL, PRTSTTRESET);

		// fall-through
		// @suppress("No break at end of case")

	case rReset:
		delay(RESFREEDEL/actChan); // delay for modem resource free
		tstate = rInit;
		printPrgMem(PRTSTTTBL, PRTSTTRESTART);
		break;

	default:
	case rEnd:
		if (!testend)
			printPrgMem(PRTSTTTBL, PRTSTTDONE);
		testend = 1;
	}

	if (-1 == ret && (rStart != tstate) && (rRun != tstate) ){
		printPrgMem(PRTSTTTBL, PRTSTTERREXEC);
		tstate = rEnd;
	}

	return tstate;
}

/*************** SYSTEM SETUP AND LOOP *****************/

void initPorts(){
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

	LoRaMgmtSetup();

	tgrp = &testConfig[0]; 	// assign pointer to pointer to TestgroupA
	tno = *tgrp; 			// assign pointer to pointer to test 1
	trn = &testResults[0];	// Init results pointer

	startTs = millis();		// snapshot starting time
}

void loop()
{
	if (testend)
		readInput();
	else{
		if (*tno)
			runTest(*tno);
		else
			printPrgMem(PRTSTTTBL, PRTSTTERRTEXT);
	}
  // received something
  debugSerial.read();
}
