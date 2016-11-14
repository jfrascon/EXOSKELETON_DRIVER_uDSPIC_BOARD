/*******************************************************************************
* File:   pwm.h
* Version 0.1
* Author: Francisco Rascón
          José González
******************************************************************************/

#ifndef PWM_H
#define PWM_H

struct PWM {
  float T_ms;
  float duty_cycle;
  float min_duty_cycle;
  float max_duty_cycle;
};

void pwm_init(struct PWM *pwm, float T_ms, float max_duty_cycle);
void pwm_set_duty_cycle(struct PWM *pwm, float duty_cycle);

#endif
