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

 uint8_t EEMEM ee_bootCnt;	// reboot counter

/* PROGMEM string lists */

const char prtSttReboot[] PROGMEM = "Reboot counter read from device: ";
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
const char prtSttLoop[] PROGMEM = "-- LOOP 10 Seconds --\n";
const char prtSttSkipT[] PROGMEM = "Skip to next test\n";
const char prtSttSKipG[] PROGMEM = "Skip to next test group\n";
const char prtSttEndG[] PROGMEM = "End of test groups\n";
const char prtSttErrExec[] PROGMEM = "ERROR: during state execution\n";
const char prtSttErrText[] PROGMEM = "ERROR: test malfunction\n";
const char prtSttWrnConf[] PROGMEM = "WARN: Invalid test configuration\n";

PGM_P const prtSttStr[] PROGMEM = {prtSttReboot, prtSttStart, prtSttPoll, prtSttStop, prtSttRetry, prtSttEvaluate, prtSttAddMeas, prtSttReset, prtSttRestart, prtSttEnd, prtSttPollErr, prtSttLoop, prtSttSkipT, prtSttSKipG, prtSttEndG, prtSttErrExec, prtSttErrText, prtSttWrnConf};

#define PRTSTTREBOOT 0
#define PRTSTTSTART 1
#define PRTSTTPOLL 2
#define PRTSTTSTOP 3
#define PRTSTTRETRY 4
#define PRTSTTEVALUATE 5
#define PRTSTTADDMEAS 6
#define PRTSTTRESET 7
#define PRTSTTRESTART 8
#define PRTSTTEND 9
#define PRTSTTPOLLERR 10
#define PRTSTTLOOP 11
#define PRTSTTSKIPT 12
#define PRTSTTSKIPG 13
#define PRTSTTENDG 14
#define PRTSTTERREXEC 15
#define PRTSTTERRTEXT 16
#define PRTSTTWRNCONF 17

const char prtTblCR[] PROGMEM = " CR 4/";
const char prtTblDR[] PROGMEM = " DR ";
const char prtTblBW[] PROGMEM = " BW [khz] ";
const char prtTblFrq[] PROGMEM = " Freq [hz] ";
const char prtTblPwr[] PROGMEM = " pwr [dBm] ";
const char prtTblRssi[] PROGMEM = " rssi ";
const char prtTblSnr[] PROGMEM = " snr ";
const char prtTblTTx[] PROGMEM = " Time TX: ";
const char prtTblTRx[] PROGMEM = " Time RX: ";
const char prtTblTTl[] PROGMEM = " Time Total: ";
const char prtTblTms[] PROGMEM = " ms";

PGM_P const prtTblStr[] PROGMEM = {prtTblCR, prtTblDR, prtTblBW, prtTblFrq, prtTblPwr, prtTblRssi, prtTblSnr, prtTblTTx, prtTblTRx, prtTblTTl, prtTblTms};

#define PRTTBLCR 0
#define PRTTBLDR 1
#define PRTTBLBW 2
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
static int prgend;								// is test-program terminated?

/* 	Globals		*/

int debug = 1;

// Test data structure
typedef struct _testParam{
	int dataLen;			// Data length of the test data
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
TT_InitAll(){

	// Setup channels to All standard channels
	actChan = 8;
	return LoRaSetChannels(0xFF); // Enable channels 1-8 only;
}

static int
TT_Eval(){

	return 0;
}

/*************** TEST CONFIGURATIONS ********************/

// Test definition
static testParam_t testA1 = {
	15, 0,
	&TT_InitMono,
	&LoRaMgmtSend,
	&LoRaMgmtPoll,
	NULL,
	&TT_Eval,
	NULL,
};

static testParam_t testA2 = {
	15, 0,
	&TT_InitAll,
	&LoRaMgmtSend,
	&LoRaMgmtPoll,
	NULL,
	&TT_Eval,
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
		sprintf(buf, "%c;%02d;%02d;%07lu;%07lu;%lu;%02u;%02d;%03d;%03d",
				prntGrp, prntTno, i, trn->timeTx, trn->timeRx,
				trn->txFrq, trn->txDR, trn->txPwr,
				trn->rxRssi, trn->rxSnr);
				debugSerial.println(buf);
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
static bool confirmed = true;	// TODO: implement menu and switch, BUT should it be changed?
static int	retries; 			// un-conf send retries
static int	pollcnt;			// un-conf poll retries

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

		if (testNow->init)
			if ((ret = testNow->init()))
				break;

		writeSyncState(testNow->syncCode);

		// Set global test parameters
		LoRaSetGblParam(confirmed, testNow->dataLen);
		retries=0;
		pollcnt=0;

		tstate = rStart;
		printPrgMem(PRTSTTTBL, PRTSTTSTART);
		// fall-through
		// @suppress("No break at end of case")

	case rStart:

		if (testNow->start)
			if ((ret = testNow->start()) && ret != 1){
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

		if (testNow->run)
			if ((ret = testNow->run()) && (confirmed || (pollcnt < UNCF_POLL))){
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
		if (testNow->stop)
			if ((ret = testNow->stop()))
				break;

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
		if (testNow->evaluate)
			if ((ret = testNow->evaluate()))
				break;

		printPrgMem(PRTSTTTBL, PRTSTTADDMEAS);

		(void)LoRaMgmtGetResults(trn); // TODO: implement and use return value
		// pgm_read_word = read char pointer address from PROGMEM pos PRTTBLCR of the string array
		// strcpy_P = copy char[] from PRROGMEM at that address of PRROGMEM to buf
		// *.print = print that char to serial
		printPrgMem(PRTTBLTBL,PRTTBLCR);
		debugSerial.print(trn->lastCR);
		printPrgMem(PRTTBLTBL,PRTTBLDR);
		debugSerial.print(trn->txDR);
		printPrgMem(PRTTBLTBL,PRTTBLBW);
		debugSerial.println(trn->txBW);
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
		if (testNow->reset)
			if ((ret = testNow->reset()))
				break;

		delay(RESFREEDEL/actChan); // delay for modem resource free
		tstate = rInit;
		printPrgMem(PRTSTTTBL, PRTSTTRESTART);
		break;

	default:
	case rEnd:
		;
	}

	if (-1 == ret && (rStart != tstate) && (rRun != tstate) ){
		printPrgMem(PRTSTTTBL, PRTSTTERREXEC);
		tstate = rEnd;
	}

	return tstate;
}

static testParam_t ** tno = NULL;
static testParam_t *** tgrp = NULL;

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
		if (!prgend)
			printPrgMem(PRTSTTTBL, PRTSTTENDG);
		prgend = 1;
		return;
	}

	enum testRun res = runTest(*tno);

	if (res == rError)
		printPrgMem(PRTSTTTBL, PRTSTTERRTEXT);

	// test ended
	if ((res == rEnd || res == rError) && *tno){
		printPrgMem(PRTSTTTBL, PRTSTTSKIPT);
		tno++;
		prntTno++;

		printPrgMem(PRTSTTTBL, PRTSTTLOOP);

		// compute time to 10 seconds, aware of unsigned
		unsigned long wait = 10000l - min(10000l, (millis() - startTs));
		delay(wait);
		startTs = millis();
	}

	// end of tests of test group?
	if (!*tno){
		if (*tgrp){
			tgrp++; // next test group
			tno = *tgrp;
			prntGrp++;
			prntTno = 1;
			printPrgMem(PRTSTTTBL, PRTSTTENDG);
			startTs = millis();
		}
	}


}

/*************** SYSTEM SETUP AND LOOP *****************/

void initPorts(){
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
	
	printPrgMem(PRTSTTTBL,PRTSTTREBOOT);
	debugSerial.println(val);

//	if (!debug)
//		ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
//			eeprom_update_byte(&ee_bootCnt, ++val);
//		}

	LoRaMgmtSetup();

	tgrp = &testConfig[0]; 	// assign pointer to pointer to TestgroupA
	tno = *tgrp; 			// assign pointer to pointer to test 1
	trn = &testResults[0];	// Init results pointer

	prntGrp = 'A';			// Actual executing group
	prntTno = 1;			// actual executing testno

	startTs = millis();		// snapshot starting time
}

void loop()
{
  // call test selector
  selectTest();

  // received something
  debugSerial.read();
}
