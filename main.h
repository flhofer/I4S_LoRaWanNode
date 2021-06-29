/*
 * main.h
 *
 *  Created on: May 25, 2020
 *      Author: Florian Hofer
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#include "Arduino.h" // Arduino Framework CORE -- I/O & Serial ports

// Serial connection definition
#define debugSerial Serial		// USB Serial
#define loraSerial Serial1		// Hardware serial

//#define LORA_DEBUG 		debugSerial
#define MICROVER		"AVR_1.0V"

//global variable declarations
extern int debug;				// Global flag Debug console attached (PC)

//Do not add code below this line
#endif /* _MAIN_H_ */
