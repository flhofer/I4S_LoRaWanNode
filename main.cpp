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
#define RESFREEDEL	40000		// ~resource freeing delay ETSI requirement air-time reduction

// Allocate initPorts in init section3 of code
void initPorts (void) __attribute__ ((naked)) __attribute__ ((section (".init3")));

/* EEPROM address */

 uint8_t EEMEM ee_bootCnt;	// reboot counter

/* PROGMEM string lists */

 const char prtSttReboot[] PROGMEM = "Reboot counter read from device: ";
 const char prtSttStart[] PROGMEM = "Start test\n";
 const char prtSttContinue[] PROGMEM = "Continue ";
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
 const char prtSttWrnFull[] PROGMEM = "*** RESULTS STORAGE FULL ***\n";

 PGM_P const prtSttStr[] PROGMEM = {prtSttReboot, prtSttStart, prtSttContinue, prtSttPoll, prtSttStop, prtSttRetry, prtSttEvaluate, prtSttAddMeas, prtSttReset, prtSttRestart, prtSttEnd, prtSttPollErr, prtSttLoop, prtSttSkipT, prtSttSKipG, prtSttEndG, prtSttErrExec, prtSttErrText, prtSttWrnConf, prtSttWrnFull};

 #define PRTSTTREBOOT 0
 #define PRTSTTSTART 1
 #define PRTSTTCONTINUE 2
 #define PRTSTTPOLL 3
 #define PRTSTTSTOP 4
 #define PRTSTTRETRY 5
 #define PRTSTTEVALUATE 6
 #define PRTSTTADDMEAS 7
 #define PRTSTTRESET 8
 #define PRTSTTRESTART 9
 #define PRTSTTEND 10
 #define PRTSTTPOLLERR 11
 #define PRTSTTLOOP 12
 #define PRTSTTSKIPT 13
 #define PRTSTTSKIPG 14
 #define PRTSTTENDG 15
 #define PRTSTTERREXEC 16
 #define PRTSTTERRTEXT 17
 #define PRTSTTWRNCONF 18
 #define PRTSTTWRNFULL 19

const char prtTblCR[] PROGMEM = " CR 4/";
const char prtTblSF[] PROGMEM = " SF ";
const char prtTblBW[] PROGMEM = " BW [khz] ";
const char prtTblFrq[] PROGMEM = " Freq [hz] ";
const char prtTblPwr[] PROGMEM = " pwr [dBm] ";
const char prtTblRssi[] PROGMEM = " rssi ";
const char prtTblSnr[] PROGMEM = " snr ";
const char prtTblTTx[] PROGMEM = " Time TX: ";
const char prtTblTRx[] PROGMEM = " Time RX: ";
const char prtTblTTl[] PROGMEM = " Time Total: ";
const char prtTblTms[] PROGMEM = " ms";

PGM_P const prtTblStr[] PROGMEM = {prtTblCR, prtTblSF, prtTblBW, prtTblFrq, prtTblPwr, prtTblRssi, prtTblSnr, prtTblTTx, prtTblTRx, prtTblTTl, prtTblTms};

#define PRTTBLCR 0
#define PRTTBLSF 1
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
static sLoRaResutls_t * trn;			// Pointer to actual entry
static uint8_t actChan = 16;					// active channels
static int prgend;								// is test-program terminated?

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
	sLoRaResutls_t * trn;
	testParam_t ** tno = NULL,
				*** tgrp = NULL;

	tgrp = &testConfig[0]; 	// assign pointer to pointer to TestgroupA
	tno = *tgrp; 			// assign pointer to pointer to test 1
	trn = &testResults[0]; // Init results pointer

	// for printing
	char grp = 'A';
	int no = 1;
	char buf[128];

	for (;((*tgrp)); tgrp++, grp++){
		for (; ((*tno)); tno++, no++){
			for (int i = 1; i< (*tno)->counter ; i++, trn++){
				sprintf(buf, "%c;%02d;%02d;%07lu;%07lu;%lu;%02u;%02d;%03d;%03d",
						grp, no, i, trn->timeTx, trn->timeRx,
						trn->txFrq, trn->txSF, trn->txPwr,
						trn->rxRssi, trn->rxSnr);

				debugSerial.println(buf);
			}
		}
		tno = *tgrp;
		no = 1;
	}
	exit(1); // now end program
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
static bool confirmed = true;	// TODO: implement menu and switch
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
		printPrgMem(PRTSTTTBL, PRTSTTWRNCONF);
		return rError;
	}

	// reset status on next test
	if (rEnd == tstate || rError == tstate )
		tstate = rInit; // we don't call with error/end

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
		printPrgMem(PRTSTTTBL, PRTSTTSTART);
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

		printPrgMem(PRTSTTTBL, PRTSTTCONTINUE);
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

		if (TST_RETRY > retries){
			tstate = rStart;
			printPrgMem(PRTSTTTBL, PRTSTTRETRY);
			delay(RESFREEDEL/actChan); // delay for modem resource free
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

		sLoRaResutls_t * res;
		res =  LoRaMgmtGetResults();
		// pgm_read_word = read char pointer address from PROGMEM pos PRTTBLCR of the string array
		// strcpy_P = copy char[] from PRROGMEM at that address of PRROGMEM to buf
		// *.print = print that char to serial
		printPrgMem(PRTTBLTBL,PRTTBLCR);
		debugSerial.print(res->lastCR);
		printPrgMem(PRTTBLTBL,PRTTBLSF);
		debugSerial.print(res->txSF);
		printPrgMem(PRTTBLTBL,PRTTBLBW);
		debugSerial.print(res->txBW);
		debugSerial.println("kHz ");
		printPrgMem(PRTTBLTBL,PRTTBLFRQ);
		debugSerial.print(res->txFrq);
		printPrgMem(PRTTBLTBL,PRTTBLPWR);
		debugSerial.print(res->txPwr);
		printPrgMem(PRTTBLTBL,PRTTBLRSSI);
		debugSerial.print(res->rxRssi);
		printPrgMem(PRTTBLTBL,PRTTBLSNR);
		debugSerial.println(res->rxSnr);

		printPrgMem(PRTTBLTBL,PRTTBLTTX);
		printScaled(res->timeTx);
		printPrgMem(PRTTBLTBL,PRTTBLTMS);
		printPrgMem(PRTTBLTBL,PRTTBLTTX);
		printScaled(res->timeRx);
		printPrgMem(PRTTBLTBL,PRTTBLTMS);
		printPrgMem(PRTTBLTBL,PRTTBLTTL);
		printScaled(res->timeToRx);
		printPrgMem(PRTTBLTBL,PRTTBLTMS);
		debugSerial.println();

		if (!testNow->results)
			testNow->results = trn; // apply to first

		if (trn < &testResults[TST_MXRSLT]){
			// Copy results to local counters, starts at the end
			(void)memcpy(trn, res, sizeof(sLoRaResutls_t));
			testNow->resultsSize++;
			trn++;
		}
		else
			printPrgMem(PRTSTTTBL, PRTSTTWRNFULL);

		// Test repeats?
		if (--testNow->counter <= 0){
			tstate = rEnd;
			printPrgMem(PRTSTTTBL, PRTSTTEND);
			break;
		}

		tstate = rReset;
		printPrgMem(PRTSTTTBL, PRTSTTRESET);

		// fall-through
		// @suppress("No break at end of case")

	case rReset:
		if (testNow->reset)
			if ((ret = testNow->reset()))
				break;

		//delay(RESFREEDEL/actChan); // delay for modem resource free
		tstate = rInit;
		printPrgMem(PRTSTTTBL, PRTSTTRESTART);
		break;

	default:
	case rEnd:
		;
	}

	if (ret == -1 ){
		printPrgMem(PRTSTTTBL, PRTSTTERREXEC);
		tstate = rEnd;
	}

	return tstate;
}

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

		printPrgMem(PRTSTTTBL, PRTSTTLOOP);

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
	trn = &testResults[0]; // Init results pointer

	startTs = millis();		// snapshot starting time
}

void loop()
{
  // call test selector
  selectTest();

  if (debugSerial.read() && prgend)
	  printTestResults();
}
