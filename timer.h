/*
 * time.h
 *
 *  Created on: May 26, 2020
 *      Author: Florian Hofer
 */

#ifndef TIMER_H_
#define TIMER_H_

#ifdef __cplusplus
extern "C"{
#endif

void installTimer();
void startTimer();
unsigned long getTimer();

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* TIMER_H_ */
