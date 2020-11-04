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

int tno = 0;
int debug = 1;

void initVariant(){

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
		  delay(10);
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

  LoRaMgmtLoop();

  delay(10000); // TODO: Class C must scan all the time
  debugSerial.read();
}
