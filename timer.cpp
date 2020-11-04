/*
 * time.cpp
 *
 *  Created on: May 26, 2020
 *      Author: Florian Hofer
 */

#include "timer.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// keep timer overflows to count for total time
volatile unsigned int overflows = 0;
const unsigned long res =  4; // =(1 << 6*10000000)/F_CPU;

void installTimer(){
	// reset settings for timer/counter
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1C = 0;

	TIMSK1 = _BV(TOIE1);    // interrupt mask register
}

/*
 * Starts the timer, by setting the clock source and scaler
 */
void startTimer(){
	TCNT1 = 0; // reset after use
	overflows = 0;
	TCCR1B =  _BV(CS11) | _BV(CS10); // 64 prescaler
}

/*
 * Returns elapsed time in microseconds, 'res' resolution
 */
unsigned long getTimer(){
	unsigned int temp_timer;
	unsigned long ticks;

	//		TCCR1B = 0;    //stop the timer, prescaler = 0 -> use atomic to avoid iterruptions
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		temp_timer = TCNT1;
		ticks = ((((unsigned long)overflows) << 16) | (unsigned long)temp_timer) - 4;
	}
	ticks *= res; // to ms
	return ticks;
}


ISR(TIMER1_OVF_vect)
{
  overflows++;
}//end ISR
