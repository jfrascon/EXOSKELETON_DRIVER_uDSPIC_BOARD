/*******************************************************************************
* File:   timer.c
* Version 0.1
* Author: José González
          Francisco Rascón

 * Drivers for the timers of the system. Contains all functions and
 definitions related to time.
 ******************************************************************************/

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

/** Interrupt prototype */
extern void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void);
extern void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt(void);

/** Function prototypes */
void timer1_init();
void timer2_init();
void timer3_init();

int32_t timer1_get();
int32_t timer2_get();

void timer1_set(int32_t val);
void timer2_set(int32_t val);

int32_t overf1_read();
int32_t overf2_read();
int32_t overf3_read();

void overf1_write(int32_t value);
void overf2_write(int32_t value);
void overf3_write(int32_t value);

// void delay_ms(uint32_t duration);
// void delay_us(uint32_t duration_us);

#endif /* TIMER_H */
