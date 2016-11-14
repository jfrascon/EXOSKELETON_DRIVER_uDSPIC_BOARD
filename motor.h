/*******************************************************************************
* File:   motor.h
* Version 0.1
* Author: Francisco Rascón
          José González
********************************************************************************/

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

enum mot_dir { MOTOR_CW, MOTOR_CCW, MOTOR_STOP };

enum mot_status { MOTOR_DISABLED, MOTOR_ENABLED };

struct Motor {
  enum mot_dir dir;
  enum mot_status status;
  int16_t num_poles;
  int16_t max_speed_rpm;
  int16_t speed_rpm;
  float max_current_amp;
  float current_amp;
};

void motor_init(struct Motor *motor, int16_t num_poles, int16_t max_speed_rpm,
                float max_current_amp);
float motor_get_current_amp(struct Motor *motor);
void motor_set_current_amp(struct Motor *motor, float current_amp);
float motor_get_current_normalized(struct Motor *motor);
void motor_set_current_normalized(struct Motor *motor,
                                  float current_normalized);
enum mot_dir motor_get_dir(struct Motor *motor);
void motor_set_dir(struct Motor *motor, enum mot_dir dir,
                   int16_t command_motor);
float motor_get_speed_normalized(struct Motor *motor);
void motor_set_speed_normalized(struct Motor *motor, float speed_normalized);
int16_t motor_get_speed_rpm(struct Motor *motor);
void motor_set_speed_rpm(struct Motor *motor, int16_t speed_rpm);
enum mot_status motor_get_status(struct Motor *motor);
void motor_set_status(struct Motor *motor, enum mot_status status);

#endif
