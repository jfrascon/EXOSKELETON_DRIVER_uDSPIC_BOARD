/*******************************************************************************
* File:   pwm.c
* Version 0.1
* Author: Francisco Rascón
          José González
 ******************************************************************************/

#include "pwm.h"
#include "constants.h"
#include <math.h>
#include <stdint.h>
#include <string.h> //memset
#include <xc.h>

inline static void pwm_init_(struct PWM *pwm);
inline static void pwm_set_duty_cycle_(struct PWM *pwm);

// ##### ##### ##### ##### ##### ##### ##### ##### ##### #####
//
// MICRO-CONTROLLER INDEPENDENT CODE
//
// ##### ##### ##### ##### ##### ##### ##### ##### ##### #####

void pwm_init(struct PWM *pwm, float T_ms, float max_duty_cycle) {
  memset(pwm, 0, sizeof(struct PWM));

  if (T_ms < 0.0f)
    pwm->T_ms = fabsf(T_ms);
  else
    pwm->T_ms = T_ms;

  pwm->duty_cycle = 0.0f;

  if (max_duty_cycle > 1.00f)
    max_duty_cycle = 1.00f;
  else if (max_duty_cycle < 0.00f && max_duty_cycle > -1.00f)
    max_duty_cycle = fabsf(max_duty_cycle);
  else
    pwm->max_duty_cycle = max_duty_cycle;

  pwm_init_(pwm);
}

void pwm_set_duty_cycle(struct PWM *pwm, float duty_cycle) {
  pwm->duty_cycle = fabsf(duty_cycle);
  if (pwm->duty_cycle > pwm->max_duty_cycle)
    pwm->duty_cycle = pwm->max_duty_cycle;

  pwm_set_duty_cycle_(pwm);
}

// ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### #####
// #####
//
// MICRO-CONTROLLER DEPENDENT CODE (PRIVATE FUNCTIONS, ONLY AVAILABLE IN THIS
// FILE)
//
// ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### #####
// #####

inline static void pwm_init_(struct PWM *pwm) {
  // 00 => free running mode
  // 01 => single event mode
  // 10 => continuous up/down counting mode
  // 11 => continuous up/down mode with interrupts for double PWM updates
  //_PTMOD = 0b00;

  // _PTCKPS = 0x00 => prescale 1:1
  // _PTCKPS = 0x01 => prescale 1:4
  // _PTCKPS = 0x10 => prescale 1:16
  // _PTCKPS = 0x11 => prescale 1:64
  uint16_t prescale = 64;

  switch (prescale) {
  case 1: {
    _PTCKPS = 0;
    break;
  }
  case 4: {
    _PTCKPS = 1;
    break;
  }
  case 16: {
    _PTCKPS = 2;
    break;
  }
  case 64: {
    _PTCKPS = 3;
    break;
  }
  }

  // 0000 => 1:1 Postscale
  // _PTOPS = 0b0000;
  // 0 => PWM time // _PTSIDL = 0b0;base runs in CPU Idle mode
  // _PTSIDL = 0b0;
  // Clear PWM timer counter
  // _PTMR = 0b000000000000000;
  // _PTDIR = 0b0
  // PTPER = (Fcy / (Fpwm * prescale)) - 1
  // XT = 8 MHz
  // PLL = 8
  // Fosc = XT * PLL = 8 * 8 = 64 MHz
  // Fcy = Fosc / 4 = 16 MHz
  _PTPER = ((float)FCY / ((1000.0f / pwm->T_ms) * prescale)) - 1;

  // PWMCON1 = 0x0101; // Enable PWM1
  PWMCON1 = 0x0202; // Enable PWM2

  // PDC1 = 0;
  // PDC2 = 0;
  _PTEN = 0b1; // Enable PWM time base
}

inline static void pwm_set_duty_cycle_(struct PWM *pwm) {
  // PDC1 = (int16_t) (2 * pwm->duty_cycle * _PTPER);
  PDC2 = (int16_t)(2 * pwm->duty_cycle * _PTPER);
}
