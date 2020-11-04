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

// DevAddr, NwkSKey, AppSKey and the frequency plan
const char *devAddr = LORA_DEVADDR;
const char *nwkSKey = LORA_NWSKEY;
const char *appSKey = LORA_APSKEY;

// Select frequency plan between TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_EU868

// Modem constructor
TheThingsNetwork ttn(loraSerial, debugSerial,
					freqPlan, TTN_DEFAULT_SF, TTN_DEFAULT_FSB);

unsigned long lastTime = 0; // store last measurement

/*************** CALLBACK FUNCTIONS ********************/

/*
 * onMessage: Callback function for LoraRx
 * Arguments: - Byte vector with payload
 * 			  - vector length
 * 			  - LoRaWan Port
 * Return:	  -
 */
static void onMessage(const uint8_t *payload, size_t size, port_t port){
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
 * onBeforeTx: Callback function for before LoRa TX
 * Arguments: -
 *
 * Return:	  -
 */
static void onBeforeTx(){
	startTimer();
}

/*
 * onAfterTx: Callback function for after LoRa TX
 * Arguments: -
 *
 * Return:	  -
 */
static void onAfterTx(){
	lastTime = getTimer();
	if (!debug)
		return;

	debugSerial.print("Time TX: ");
	debugSerial.print(lastTime/1000);
	debugSerial.print(".");
	debugSerial.print(lastTime %1000);
	debugSerial.println(" ms");
}

/*
 * onAfterTx: Callback function for after LoRa RX
 * Arguments: -
 *
 * Return:	  -
 */
static void onAfterRx(){
	lastTime = getTimer();
	if (!debug)
		return;

	debugSerial.print("Time RX: ");
	debugSerial.print(lastTime/1000);
	debugSerial.print(".");
	debugSerial.print(lastTime %1000);
	debugSerial.println(" ms");
}

/*************** TEST SEND FUNCTIONS ********************/

/*
 * LoRaMgmtSendConf: send a confirmed message. If no response arrives
 * 		within timeout, return 1 (busy)
 *
 * Arguments: -
 *
 * Return:	  status of polling, 0 ok, -1 error, 1 busy
 */
int LoRaMgmtSendConf(){

	  // Prepare PayLoad of 1 byte to indicate LED status
	  byte payload[1];
	  payload[0] = (digitalRead(LED_BUILTIN) == HIGH) ? 1 : 0;

	  // Send it off
	  ttn_response_t ret = ttn.sendBytes(payload, sizeof(payload), 1, true);

	  switch (ret) {

		  // TX only
	  case TTN_SUCCESSFUL_TRANSMISSION:
		  // ACK receive ok
	  case TTN_SUCCESSFUL_RECEIVE:
		  return 0;

	  case TTN_UNSUCESSFUL_RECEIVE:
		  // probably busy
		  return 1;

	  default:
	  case TTN_ERROR_SEND_COMMAND_FAILED:
	  case TTN_ERROR_UNEXPECTED_RESPONSE:
		  debugSerial.println("Unable to send payload!\n");
		  return -1;
	  }
}

/*
 * LoRaMgmtPoll: poll function for confirmed and delayed TX
 *
 * Arguments: -
 *
 * Return:	  status of polling, 0 ok, -1 error, 1 busy
 */
int LoRaMgmtPoll(){


	return 0;
}

/*************** MANAGEMENT FUNCTIONS ********************/

/*
 * LoRaMgmtGetTime: getter for last measured time
 * Arguments: -
 *
 * Return:	  ulong with time in ms
 */
unsigned long LoRaMgmtGetTime(){
	return lastTime;
}

/*
 * LoRaMgmtSetup: setup LoRaWan communication with modem
 * Arguments: -
 *
 * Return:	  -
 */
void LoRaMgmtSetup(){

	installTimer(); // setup timer1 registers

	// Initialize Serial1
	loraSerial.begin(57600);

	debugSerial.println("-- PERSONALIZE");
	ttn.personalize(devAddr, nwkSKey, appSKey);

//	ttn.setClass(CLASS_C);
	ttn.onMessage(&onMessage);
	ttn.onBeforeTx(&onBeforeTx);
	ttn.onAfterTx(&onAfterTx);
	ttn.onAfterRx(&onAfterRx);

	debugSerial.println("-- STATUS");
	ttn.showStatus();
}


