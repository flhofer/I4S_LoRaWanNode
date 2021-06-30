/*
 * main.cpp
 *
 *  Created on: May 25, 2020
 *      Author: Florian Hofer
 */

#include "main.h"
#include "LoRaMgmt.h"			// LoRaWan modem management

#include <util/atomic.h>		// Protected block routines
#include <avr/eeprom.h>			// EEPROM functions

#define TST_MXRSLT	30			// What's the max number of test results we allow?
#define LEDBUILDIN	PORTC7		// AVR build in led position, PORT C
#define KEYBUFF		73			// Max total usage of key buffers = 32 + 32 + 16 + 3*\0
#define KEYSIZE		32			// 32

// Allocate initPorts in init section3 of code
void initPorts (void) __attribute__ ((naked)) __attribute__ ((section (".init3")));

/* PROGMEM string lists */
const char prtSttStart[] PROGMEM = "Start test\n";
const char prtSttPoll[] PROGMEM = "Poll for answer\n";
const char prtSttStop[] PROGMEM = "Stop test\n";
const char prtSttRetry[] PROGMEM = "Retry\n";
const char prtSttEvaluate[] PROGMEM = "Evaluate\n";
const char prtSttReset[] PROGMEM = "Reset\n";
const char prtSttRestart[] PROGMEM = "Restart - Init\n";
const char prtSttEnd[] PROGMEM = "End test\n";
const char prtSttPollErr[] PROGMEM = "Poll - No response from server.\n";
const char prtSttDone[] PROGMEM = "done\n";
const char prtSttErrExec[] PROGMEM = "ERROR: during state execution\n";
const char prtSttErrText[] PROGMEM = "ERROR: test malfunction\n";
const char prtSttSelect[] PROGMEM = "Select Test:\n";
const char prtSttResults[] PROGMEM = "Results:\n";
const char prtSttValLong[] PROGMEM = "Error: too long value!\n";
const char prtSttValLongH[] PROGMEM = "Error: too long value! Remember using h to terminate hex\n";
const char prtSttInvalid[] PROGMEM = "Error: Invalid value!\n";
const char prtSttUnknown[] PROGMEM = "Error: Unknown command ";

PGM_P const prtSttStr[] PROGMEM = {prtSttStart, prtSttPoll, prtSttStop, prtSttRetry, prtSttEvaluate, prtSttReset, prtSttRestart, prtSttEnd,
		prtSttPollErr, prtSttDone, prtSttErrExec, prtSttErrText, prtSttSelect, prtSttResults, prtSttValLong, prtSttValLongH, prtSttInvalid, prtSttUnknown};

#define PRTSTTSTART		0
#define PRTSTTPOLL		1
#define PRTSTTSTOP		2
#define PRTSTTRETRY		3
#define PRTSTTEVALUATE	4
#define PRTSTTRESET		5
#define PRTSTTRESTART	6
#define PRTSTTEND		7
#define PRTSTTPOLLERR	8
#define PRTSTTDONE		9
#define PRTSTTERREXEC	10
#define PRTSTTERRTEXT	11
#define PRTSTTSELECT	12
#define PRTSTTRESULTS	13
#define PRTSTTVALLONG	14
#define PRTSTTVALLONGH	15
#define PRTSTTINVALID	16
#define PRTSTTUNKNOWN	17

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

#define PRTTBLCR	0
#define PRTTBLDR	1
#define PRTTBLCHMSK 2
#define PRTTBLFRQ	3
#define PRTTBLPWR	4
#define PRTTBLRSSI	5
#define PRTTBLSNR	6
#define PRTTBLTTX	7
#define PRTTBLTRX	8
#define PRTTBLTTL	9
#define PRTTBLTMS	10

#define PRTSTTTBL 0
#define PRTTBLTBL 1

/* Locals 		*/

// Working variables
static sLoRaResutls_t testResults[TST_MXRSLT];	// Storage for test results
static sLoRaConfiguration_t newConf;			// test Configuration
static char keyArray[KEYBUFF];					// static array containing init keys

/* 	Globals		*/

int debug = 1;

/* Declarations */
static void printPrgMem(int tbl, int pos);

/*************** MIXED STUFF ********************/

/*
 * printScaled(): Print integer as scaled value
 *
 * Arguments:	- value to print
 * 				- scale in x-ths, default (1/1000th)
 *
 * Return:		-
 */
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

/*
 * printTestResults(): Print LoRaWan communication test
 *
 * Arguments:	- count to print
 *
 * Return:		-
 */
static void
printTestResults(int count){
	// use all local, do not change global
	sLoRaResutls_t * trn = &testResults[0]; // Initialize results pointer

	// for printing
	char buf[128];

	printPrgMem(PRTSTTTBL, PRTSTTRESULTS);
	for (int i = 1; i<= min(TST_MXRSLT, count); i++, trn++){
		sprintf(buf, "%02d;%02u;%07lu;%07lu;0x%02X;%lu;%02u;%02d;%03d;%03d",
				i, //trn->testTime,
				trn->txCount, trn->timeTx, trn->timeRx,
				trn->chnMsk, trn->txFrq, trn->txDR, trn->txPwr,
				trn->rxRssi, trn->rxSnr);
		debugSerial.println(buf);
	}
}

/*
 * readSerialS(): parsing hex input strings
 *
 * Arguments:	- pointer to char* to put string
 *
 * Return:		-
 */
static void
readSerialS(char * retVal, int len){
	char nChar;
	int pos = 0;
	retVal[0] = '\0';
	while (1){
		if (pos > len ){
			printPrgMem(PRTSTTTBL, PRTSTTVALLONGH);
			break;
		}

		nChar = debugSerial.peek();
		switch (nChar)
		{
			case '0' ... '9': // Digits
			case 'A' ... 'F': // capital letters A-F
			case 'a' ... 'f': // small letters a-f
				debugSerial.read();
				retVal[pos] = nChar;
				pos++;
				break;
			case 'h':	// hex termination
				debugSerial.read();
				// fall-through
				// @suppress("No break at end of case")
			default:	// Not a number or HEX char/terminator
				retVal[pos] = '\0';
				return;
		}
	}
	retVal[pos-1] = '\0';
	return;
}

/*
 * readSerialH(): parsing hex input strings
 *
 * Arguments:	-
 *
 * Return:		- Hex number read
 */
static uint16_t
readSerialH(){
	char nChar;
	uint16_t retVal = 0;
	while (1){
		nChar = debugSerial.peek();
		if ((UINT16_MAX >> 4) < retVal && nChar != 'h'){
			printPrgMem(PRTSTTTBL, PRTSTTVALLONGH);
			break;
		}
		switch (nChar)
		{
			case '0' ... '9': // Digits
				debugSerial.read();
				retVal = retVal<<4 | (int16_t)(nChar - '0');
				break;
			case 'A' ... 'F': // capital letters A-F
				debugSerial.read();
				retVal = retVal<<4 | (int16_t)(nChar - 55); // 55 = A - 10
				break;
			case 'a' ... 'f': // small letters a-f
				debugSerial.read();
				retVal = retVal<<4 | (int16_t)(nChar - 87); // 87 = a - 10
				break;
			case 'h':	// hex termination
				debugSerial.read();
				// fall-through
				// @suppress("No break at end of case")
			default:	// Not a number or HEX char/terminator
				return retVal;
		}

	}
	return retVal;
}

/*
 * readSerialD(): parsing numeric input strings
 *
 * Arguments:	-
 *
 * Return:		- number read
 */
static uint16_t
readSerialD(){
	char A;
	uint16_t retVal = 0;
	while ((A = debugSerial.peek())
			&& (A < 58 && A >= 48)){
		if (log10((double)retVal) >= 4.0f ){
			printPrgMem(PRTSTTTBL, PRTSTTVALLONG);
			debugSerial.println();
			break;
		}
		debugSerial.read();
		retVal = retVal*10 + (int16_t)(A - 48);
	}
	return retVal;
}

static void
resetKeyBuffer(){
	memset(keyArray, 0, KEYBUFF );

	// key slots are the same for OTAA and ABP (union)
	newConf.appEui = &(keyArray[0]);			// Hex 32
	newConf.appKey = &(keyArray[KEYSIZE+1]);	// Hex 32
	newConf.devEui = &(keyArray[KEYSIZE*2+2]);	// Hex 8 or 16
}

// pgm_read_word = read char pointer address from PROGMEM pos PRTTBLCR of the string array
// strcpy_P = copy char[] from PRROGMEM at that address of PRROGMEM to buf
// *.print = print that char to serial
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

/*************** TEST MANAGEMENT FUNCTIONS*****************/

// Enumeration for test status
static enum { 	rError = -1,
				rInit = 0,
				rPrepare,
				rStart,
				rRun,
				rStop,

				rEvaluate = 15,
				rReset,

				rEnd = 20
			} tstate = rEnd;	// test run status

static enum {	qIdle = 0,
				qRun,
				qStop
			} testReq = qIdle;	// test request status

static int	retries; 			// un-conf send retries
static int	failed;				// some part failed

/*
 * runTest: test runner
 *
 * Arguments: - pointer to test structure of actual test
 *
 * Return:	  - test run enumeration status
 */
static void
runTest(){

	// reset at every call
	int ret = 0;

	LoRaMgmtMain();

	switch(tstate){

	case rInit:
		// reset at every test
		retries = 0;

		// reset status on next test
		memset(testResults,0, sizeof(testResults));

		if (LoRaMgmtSetup(&newConf, &testResults[0]))
		{
			tstate = rError;
			break;
		}
		tstate = rPrepare;
		printPrgMem(PRTSTTTBL, PRTSTTSTART);
		// fall-through
		// @suppress("No break at end of case")

	case rPrepare:
		if (testReq >= qStop ){
			tstate = rStop;
			break;
		}

		if (newConf.prep &&
			(ret = newConf.prep()) == 0)
				break;
		else if (ret < 0){
			tstate = rError;
			break;
		}

		tstate = rStart;
		// fall-through
		// @suppress("No break at end of case")

	case rStart:

		if (testReq >= qStop ){
			tstate = rStop;
			break;
		}

		if (newConf.start){
			if ((ret = newConf.start()) < 0){
				failed = 1;
				tstate = rStop;
				printPrgMem(PRTSTTTBL, PRTSTTERREXEC);
				printPrgMem(PRTSTTTBL, PRTSTTSTOP);
				break;
			}
			else if (ret == 0)	// Busy!
				break;
			else if (ret == 2) {
				tstate = rStop;
				printPrgMem(PRTSTTTBL, PRTSTTSTOP);
				break;
			}
		}
		printPrgMem(PRTSTTTBL, PRTSTTSTART);
		tstate = rRun;
		failed = 0;
		// fall-through
		// @suppress("No break at end of case")

	case rRun:

		if (testReq >= qStop ){
			tstate = rStop;
			break;
		}

		if (!newConf.run)
			break;

		if ((ret = newConf.run()) < 0){
			failed = 1;
			printPrgMem(PRTSTTTBL, PRTSTTPOLLERR);
		}
		else if (ret == 0)
			break;
		else if (ret == 2){
			tstate = rEvaluate;
			break;
		}

		tstate = rStop;
		printPrgMem(PRTSTTTBL, PRTSTTSTOP);
		retries++;
		// fall-through
		// @suppress("No break at end of case")

	case rStop:

		// unsuccessful and retries left?
		if (failed && (newConf.repeatSend > retries) && testReq < qStop){
			if (LoRaMgmtUpdt()){
				tstate = rStart;
				printPrgMem(PRTSTTTBL, PRTSTTRETRY);
			}
			break;
		}

		tstate = rEvaluate;
		printPrgMem(PRTSTTTBL, PRTSTTEVALUATE);
		// fall-through
		// @suppress("No break at end of case")

	case rEvaluate:
		{
			sLoRaResutls_t * trn = NULL;
			ret = LoRaMgmtGetResults(&trn);

			if (debug) {
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
			}

			// End of tests?
			if ((trn >= &testResults[TST_MXRSLT-1]) || (testReq >= qStop)){
				printPrgMem(PRTSTTTBL, PRTSTTEND);
				printTestResults((trn-&testResults[0])+1); // Typed difference !
				tstate = rEnd;
				testReq = qStop;
				break;
			}
		}

		tstate = rReset;
		printPrgMem(PRTSTTTBL, PRTSTTRESET);

		// fall-through
		// @suppress("No break at end of case")

	case rReset:
		if ((ret = LoRaMgmtRcnf()) == 1){
			printPrgMem(PRTSTTTBL, PRTSTTRESTART);
			retries = 0;
			tstate = rStart;
			break;
		}
		else if (ret == 0)
			break;

		// fall-through
		// @suppress("No break at end of case")

	case rError:
		printPrgMem(PRTSTTTBL, PRTSTTERREXEC);
		tstate = rEnd;
		testReq = qStop;
		break;

	default:
	case rEnd:
		if (testReq == qRun){
			// restart
			tstate = rInit;
		}
		else if (testReq != qIdle){
			// print stop
			printPrgMem(PRTSTTTBL, PRTSTTDONE);
			printPrgMem(PRTSTTTBL, PRTSTTSELECT);
			testReq = qIdle;
		}
	}

}

/*
 * readInput(): read input string
 *
 * Arguments:	-
 *
 * Return:		-
 */
void readInput() {

	signed char A;
	int intp = 0;
	while (debugSerial.available()){
		A = debugSerial.read();
		intp = 0;
		// work always
		switch (A){

		case '\n':
		case '\r':
			break;

		case 'm': // read test mode
			newConf.mode = (uint8_t)readSerialD();
			if (newConf.mode > 4){
				printPrgMem(PRTSTTTBL, PRTSTTINVALID);
				newConf.mode = 0; // set to default
			}
			resetKeyBuffer();
			newConf.confMsk &= ~CM_RJN;
			switch (newConf.mode)
			{
			default:
			case 0 : // off- mute
				newConf.prep = NULL;
				newConf.start = NULL;
				newConf.run = NULL;
				break;
			case 1 : // dumb LoRa
				newConf.prep = NULL;
				newConf.start = NULL;
				newConf.run = &LoRaMgmtSendDumb;
				newConf.frequency = 8683;
				newConf.bandWidth = 250;
				newConf.codeRate = 8;
				newConf.spreadFactor = 12;
				break;
			case 2 :  // LoRaWan
				newConf.prep = NULL;
				newConf.start = &LoRaMgmtSend;
				newConf.run = &LoRaMgmtPoll;
				break;
			case 3 : // LoRaRemote
				newConf.prep = &LoRaMgmtRemote;
				newConf.start = &LoRaMgmtSend;
				newConf.run = &LoRaMgmtPoll;
				break;
			case 4 : // LoRaWan force Join
				newConf.prep = NULL;
				newConf.start = NULL;
				newConf.run = &LoRaMgmtJoin;
				newConf.confMsk |= CM_RJN;
				break;
			}

			break;

		case 'p': // read Tx power index
			newConf.txPowerTst = (uint8_t)readSerialD();
			if (newConf.txPowerTst > 5 ){ //TODO: this is limiting to EU868
				printPrgMem(PRTSTTTBL, PRTSTTINVALID);
				newConf.txPowerTst = 0; // set to default
			}
			break;

		case 'l': // read data length
			newConf.dataLen = (uint8_t)readSerialD();
			if ((newConf.dataLen > 242 && newConf.mode >=2) // Maximum LoRaWan application payload
					|| newConf.dataLen == 0 ){
				printPrgMem(PRTSTTTBL, PRTSTTINVALID);
				newConf.dataLen = 1; // set to default
			}
			break;

		case 'r': // read repeat count for LoRaWan packets
			newConf.repeatSend = (uint8_t)readSerialD();
			if (newConf.repeatSend == 0 || newConf.repeatSend > TST_MXRSLT){
				printPrgMem(PRTSTTTBL, PRTSTTINVALID);
				newConf.repeatSend = 5; // set to default
			}

			break;
		case 'R': // set to run
			if (newConf.mode == 0) // do nothing
				break;

			if ((newConf.mode == 1 && newConf.frequency == 0) ||
				(newConf.mode >= 2 && (!newConf.devAddr ||
									   !newConf.appEui ||
									   !newConf.appKey))){
				printPrgMem(PRTSTTTBL, PRTSTTINVALID);
				break;
			}

			testReq = qRun;
			break;

		case 'S': // stop test
			testReq = max(qStop, testReq);
			printPrgMem(PRTSTTTBL, PRTSTTSTOP);
			break;

		case 'T': // Print type of micro-controller
			debugSerial.println(MICROVER);
			break;

		case 'I': // Print type of micro-controller
			{
				if (!newConf.devEui)
					resetKeyBuffer();
				const char * EUI = LoRaMgmtGetEUI(&newConf);
				if (EUI)
					debugSerial.println(EUI);
				else
					printPrgMem(PRTSTTTBL, PRTSTTERRTEXT);
			}
			break;

		case 'n': // disable debug print
			debug = 0;
			break;
		default:
			intp = 1;
		}
		if (intp && newConf.mode == 1)
			switch (A){
			case 'f': //TODO: this is limiting to EU868
				newConf.frequency = (long)readSerialD(); // TODO: 10 vs 100kHz
				if (newConf.frequency < 8630 || newConf.frequency > 8700 ){
					printPrgMem(PRTSTTTBL, PRTSTTINVALID);
					newConf.frequency = 8683; // set to default
				}
				break;

			case 'b': // read bandwidth
				newConf.bandWidth = (uint8_t)readSerialD();
				if (!(newConf.bandWidth == 250 ||
						newConf.bandWidth == 125 ||
						newConf.bandWidth == 62 ||
						newConf.bandWidth == 41 ||
						newConf.bandWidth == 31 ||
						newConf.bandWidth == 20 ||
						newConf.bandWidth == 15 ||
						newConf.bandWidth == 10 )){
					printPrgMem(PRTSTTTBL, PRTSTTINVALID);
					newConf.bandWidth = 250; // set to default
				}
				break;

			case 'c': // read code rate
				newConf.codeRate = (uint8_t)readSerialD();
				if (newConf.codeRate < 5 || newConf.codeRate > 8){
					printPrgMem(PRTSTTTBL, PRTSTTINVALID);
					newConf.codeRate = 8; // set to default
				}
				break;

			case 's': // read spread factor
				newConf.spreadFactor = (uint8_t)readSerialD();
				if (newConf.spreadFactor < 7 || newConf.spreadFactor > 12){
					printPrgMem(PRTSTTTBL, PRTSTTINVALID);
					newConf.spreadFactor = 12; // set to default
				}
				break;
			default:
				printPrgMem(PRTSTTTBL, PRTSTTUNKNOWN);
				debugSerial.println(A);
			}

		if (intp && newConf.mode >= 2)
			switch (A){

			case 'c': // set to confirmed
				newConf.confMsk &= ~CM_UCNF;
				break;
			case 'u': // set to unconfirmed
				newConf.confMsk |= CM_UCNF;
				break;

			case 'o': // set to otaa
				newConf.confMsk |= CM_OTAA;
				resetKeyBuffer();
				break;
			case 'a': // set to ABP
				newConf.confMsk &= ~CM_OTAA;
				resetKeyBuffer();
				break;

			case 'N': // Network session key for ABP
				readSerialS(newConf.nwkSKey, KEYSIZE);
				if (strlen(newConf.nwkSKey) < KEYSIZE){
					printPrgMem(PRTSTTTBL, PRTSTTINVALID);
					newConf.nwkSKey[0] = '\0';
				}
				break;

			case 'A': // Application session key for ABP
				readSerialS(newConf.appSKey, KEYSIZE);
				if (strlen(newConf.appSKey) < KEYSIZE){
					printPrgMem(PRTSTTTBL, PRTSTTINVALID);
					newConf.appSKey[0] = '\0';
				}
				break;

			case 'D': // Device address for ABP
				readSerialS(newConf.devAddr, KEYSIZE/4);
				if (strlen(newConf.devAddr) < KEYSIZE/4){
					printPrgMem(PRTSTTTBL, PRTSTTINVALID);
					newConf.devAddr[0] = '\0';
				}
				break;

			case 'K': // Application Key for OTAA
				readSerialS(newConf.appKey, KEYSIZE);
				if (strlen(newConf.appKey) < KEYSIZE){
					printPrgMem(PRTSTTTBL, PRTSTTINVALID);
					newConf.appKey[0] = '\0';
				}
				break;

			case 'E': // EUI address for OTAA
				readSerialS(newConf.appEui, KEYSIZE/2);
				if (strlen(newConf.appEui) < KEYSIZE/2){
					printPrgMem(PRTSTTTBL, PRTSTTINVALID);
					newConf.appEui[0] = '\0';
				}
				break;

			case 'C':
				newConf.chnMsk = readSerialH();
				if (newConf.chnMsk < 0x01 || newConf.chnMsk > 0xFFFF ){ //TODO: this is limiting to EU868
					printPrgMem(PRTSTTTBL, PRTSTTINVALID);
					newConf.chnMsk = 0xFF; // set to default
				}
				break;

			case 'd': // read data rate
				newConf.dataRate = (uint8_t)readSerialD();
				if (newConf.dataRate > 5){ // we exclude 6 and 7 for now
					printPrgMem(PRTSTTTBL, PRTSTTINVALID);
					newConf.dataRate = 5; // set to default
				}
				break;
			default:
				printPrgMem(PRTSTTTBL, PRTSTTUNKNOWN);
				debugSerial.println(A);
			}
	}

}

/*************** SYSTEM SETUP AND LOOP *****************/

void initPorts(){
	// Led 13 out
	DDRC |= (0x01 << LEDBUILDIN);
	PORTC &= ~(0x01 << LEDBUILDIN);
}

/*
 *	Setup(): system start
 *
 *	The setup function is called once at startup of the sketch
 */
void setup()
{
	// Initialize Serial ports 0
	// setup serial for debug, disable if no connection after 10 seconds
	debugSerial.begin(9600);
	int waitSE = 999;	// wait for 10 seconds, -10 ms for computation
	while (!debugSerial && waitSE) {
	  //delay(10);// -- Included in Serial_::operator()
	  waitSE--;
	}
	debug = ((waitSE));	// reset debug flag if time is elapsed

	// Blink once Led buildin to show program start
	PORTC |= (0x01 << LEDBUILDIN);
	delay (500);
	PORTC &= ~(0x01 << LEDBUILDIN);

	printPrgMem(PRTSTTTBL, PRTSTTSELECT);
	debugSerial.flush();
}

/*
 *  loop(): main call
 *
 *  The loop function is called in an endless loop
 */
void loop()
{
	if (testReq == qIdle)
		readInput();
	else{
		runTest();
		// received something?
		if (debugSerial.peek())
			//stop?
			readInput();
	}
}
