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

/*
 * onMessage: Callback function for LoraRx
 * Arguments: - Byte vector with payload
 * 			  - vector length
 * 			  - LoRaWan Port
 * Return:	  -
 */
void onMessage(const uint8_t *payload, size_t size, port_t port){
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

void onBeforeTx(){
	startTimer();
}

void onAfterTx(){
	unsigned long time = getTimer();
	if (!debug)
		return;

	debugSerial.print("Time TX: ");
	debugSerial.print(time/1000);
	debugSerial.print(".");
	debugSerial.print(time %1000);
	debugSerial.println(" ms");
}

void onAfterRx(){
	unsigned long time = getTimer();
	if (!debug)
		return;

	debugSerial.print("Time RX: ");
	debugSerial.print(time/1000);
	debugSerial.print(".");
	debugSerial.print(time %1000);
	debugSerial.println(" ms");
}

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

void LoRaMgmtLoop(){

	  // Prepare PayLoad of 1 byte to indicate LED status
	  byte payload[1];
	  payload[0] = (digitalRead(LED_BUILTIN) == HIGH) ? 1 : 0;

	  // Send it off
	  if (TTN_SUCCESSFUL_TRANSMISSION != ttn.sendBytes(payload, sizeof(payload), 1, true))
		  debugSerial.println("Unable to send payload!\n");

}
