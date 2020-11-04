/*
 * main.h
 *
 *  Created on: May 25, 2020
 *      Author: Florian Hofer
 */

#ifndef _NOILoraSendAPB_H_
#define _NOILoraSendAPB_H_

#include "Arduino.h" // Arduino Framework CORE -- I/O & Serial ports

// Serial connection definition
#define debugSerial Serial		// USB Serial
#define loraSerial Serial1		// Hardware serial

// EUI TTN Uno, ATmega32u4 0004A30B0021DD46
// DevAddr, NwkSKey, AppSKey and the frequency plan
#define LORA_DEVADDR	"01234567"
#define LORA_NWSKEY		"01234567890abcdef01234567890abcd"
#define LORA_APSKEY		"01234567890abcdef01234567890abcd"

//global variable declarations
extern int debug;				// Global flag Debug console attached (PC)

//Do not add code below this line
#endif /* _NOILoraSendAPB_H_ */
