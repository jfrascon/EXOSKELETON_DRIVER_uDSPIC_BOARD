/*******************************************************************************
* File:   timer.c
* Version 0.1
* Author: José González
          Francisco Rascón

 * Drivers for the timers of the system. Contains all functions and
 definitions related to time.
 ******************************************************************************/

#include "timer.h"
#include <xc.h>

/** Global variables */
/* Timer 1 overflow's counter */
int32_t g_overf1 = 0;
int32_t g_overf2 = 0;
int32_t g_overf3 = 0;

/** Timer 1 Interrupt - Motor Speed Count */
extern void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void) {
  ClrWdt();

  if (g_overf1 < 42)
    g_overf1++;
  _T1IF = 0;
}

/** Timer 2 Interrupt - PID event */
extern void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt(void) {
  // ClrWdt();

  g_overf2++;
  // if (g_overf2 < 20) g_overf2++;
  _T2IF = 0;
}

/** Timer 3 Interrupt - Send messages */
extern void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt(void) {

  ClrWdt();

  if (g_overf3 < 2) {

    g_overf3++;
  }

  _T3IF = 0;
}

/** Timer1 Setup
 * Timer set to 1:8, with a period of 60000, it interrupts every 30ms
 */
void timer1_init() {

  T1CON = 0;
  // Set prescaler to 1:8
  T1CONbits.TCKPS = 1;
  // Timer period
  PR1 = 60000;
  // Set timer interruption
  _T1IP = 1;
  _T1IF = 0;
  // Turns on the timer
  T1CONbits.TON = 1;
}

int32_t timer1_get() { return TMR1; }

void timer1_set(long val) { TMR1 = val; }

/** Reads out the timer overflow's counter of timer 1 */
int32_t overf1_read() { return g_overf1; }

/** Writes a value to timer overflow's counter of timer 1 */
void overf1_write(int32_t value) { g_overf1 = value; }

/** Timer set to 1:8, with a period of 2000, this should be set to 1ms */
void timer2_init() {

  T2CON = 0;
  // Set prescaler to 1:256
  T2CONbits.TCKPS = 1;
  // Timer period
  PR2 = 2000;
  // Set Timer interruption
  _T2IP = 1;
  _T2IF = 0;
  // Turns on the timer
  T2CONbits.TON = 1;
}

/** Reads out the timer overflow's counter of timer 1 */
int32_t overf2_read() { return g_overf2; }

/** Writes a value to timer overflow's counter of timer 2 */
void overf2_write(int32_t value) { g_overf2 = value; }

/** Timer set to 1:8, with a period of 2000, this should be set to 1ms */
void timer3_init() {

  T3CON = 0;
  // Set prescaler to 1:256
  T3CONbits.TCKPS = 1;
  // Timer period
  PR3 = 2000;
  // Set Timer interruption
  _T3IP = 1;
  _T3IF = 0;
  // Turns on the timer
  T3CONbits.TON = 1;
}

/** Reads out the timer overflow's counter of timer 3 */
int32_t overf_read3() { return g_overf3; }

/** Writes a value to timer overflow's counter of timer 3 */
void overf3_write(int32_t value) { g_overf3 = value; }

/*
void delay_ms(uint32_t duration_ms)
{
    __delay32((uint32_t) (((uint64_t) (duration_ms * FCY)) / 1000ULL));
}*/

/*
void delay_us(uint32_t duration_us)
{
    // offset of 30 us displayed in oscilloscope;
    __delay32((uint32_t) (((uint64_t) ((duration_us - 30) * FCY)) /
1000000ULL));

}*/
