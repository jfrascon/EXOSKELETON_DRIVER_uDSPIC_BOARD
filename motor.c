/*******************************************************************************
* File:   motor.c
* Version 0.1
* Author: Francisco Rascón
          José González
******************************************************************************/

#include "motor.h"
#include "constants.h"
#include <math.h>
#include <string.h> //memset
#include <xc.h>

inline static void motor_set_dir_(enum mot_dir dir);
inline static void motor_set_status_(enum mot_status status);

// ##### ##### ##### ##### ##### ##### ##### ##### ##### #####
//
// MICRO-CONTROLLER INDEPENDENT CODE
//
// ##### ##### ##### ##### ##### ##### ##### ##### ##### #####

void motor_init(struct Motor *motor, int16_t num_poles, int16_t max_speed_rpm,
                float max_current_amp) {
  memset(motor, 0, sizeof(struct Motor));

  motor->dir = MOTOR_STOP;
  motor_set_status(motor, MOTOR_ENABLED);
  motor->num_poles = num_poles;
  motor->max_speed_rpm = max_speed_rpm;
  motor->speed_rpm = 0;
  motor->max_current_amp = max_current_amp;
  motor->current_amp = 0.0f;
}

float motor_get_current_amp(struct Motor *motor) { return motor->current_amp; }

float motor_get_current_normalized(struct Motor *motor) {
  return (motor->current_amp / motor->max_current_amp);
}

void motor_set_current_amp(struct Motor *motor, float current_amp) {
  if (current_amp < 0.0f)
    motor->current_amp = 0.0f;
  else if (current_amp > motor->max_current_amp)
    motor->current_amp = motor->max_current_amp;
  else
    motor->current_amp = current_amp;
}

void motor_set_current_normalized(struct Motor *motor,
                                  float current_normalized) {
  if (current_normalized < 0.0f)
    motor->current_amp = 0.0f;
  else if (current_normalized > 1.0f)
    motor->current_amp = motor->max_current_amp;
  else
    motor->current_amp = (current_normalized * motor->max_current_amp);
}

enum mot_dir motor_get_dir(struct Motor *motor) { return motor->dir; }

void motor_set_dir(struct Motor *motor, enum mot_dir dir,
                   int16_t command_motor) {
  if (dir == MOTOR_CW || dir == MOTOR_CCW)
    motor->dir = dir;
  else
    motor->dir = MOTOR_STOP;

  if (command_motor)
    motor_set_dir_(motor->dir);
}

float motor_get_speed_normalized(struct Motor *motor) {
  return (((float)motor->speed_rpm) / motor->max_speed_rpm);
}

int16_t motor_get_speed_rpm(struct Motor *motor) { return motor->speed_rpm; }

void motor_set_speed_normalized(struct Motor *motor, float speed_normalized) {
  if (speed_normalized < -1.0f || speed_normalized > 1.0f)

    motor->speed_rpm = motor->max_speed_rpm;
  else
    motor->speed_rpm = fabsf(speed_normalized) * motor->max_speed_rpm;

  if (speed_normalized < 0)
    motor_set_dir(motor, MOTOR_CW, 0);
  else
    motor_set_dir(motor, MOTOR_CCW, 0);
}

void motor_set_speed_rpm(struct Motor *motor, int16_t speed_rpm) {
  motor->speed_rpm = fabsf(speed_rpm);

  if (motor->speed_rpm > motor->max_speed_rpm)
    motor->speed_rpm = motor->max_speed_rpm;

  if (speed_rpm < 0.0f)
    motor_set_dir(motor, MOTOR_CW, 0);
  else
    motor_set_dir(motor, MOTOR_CCW, 0);
}

enum mot_status motor_get_status(struct Motor *motor) { return motor->status; }

void motor_set_status(struct Motor *motor, enum mot_status status) {
  if (status == MOTOR_ENABLED)
    motor->status = MOTOR_ENABLED;
  else
    motor->status = MOTOR_DISABLED;

  motor_set_status_(motor->status);
}

// ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### #####
// #####
//
// MICRO-CONTROLLER DEPENDENT CODE (PRIVATE FUNCTIONS, ONLY AVAILABLE IN THIS
// FILE)
//
// ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### #####
// #####

inline static void motor_set_status_(enum mot_status status) {
  // ESCON_MOT_ENABLE = status;
}

inline static void motor_set_dir_(enum mot_dir dir) {

  // ESCON_CW = !dir;
  // ESCON_CCW = dir;

  switch (dir) {
  case MOTOR_STOP: {
    ESCON_CW = 0;
    ESCON_CCW = 0;

    // ALERT_LED_OFF;
    // OK_LED_OFF;
    // CAN_LED_OFF;
    break;
  }

  case MOTOR_CCW: {
    ESCON_CW = 0;
    ESCON_CCW = 1;

    // ALERT_LED_ON;
    // OK_LED_OFF;
    // CAN_LED_OFF;
    break;
  }

  case MOTOR_CW: {
    ESCON_CW = 1;
    ESCON_CCW = 0;

    // ALERT_LED_OFF;
    // OK_LED_OFF;
    // CAN_LED_ON;
    break;
  }
  }
}
