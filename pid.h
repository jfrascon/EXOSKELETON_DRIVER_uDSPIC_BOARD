/*******************************************************************************
* File:   PID.h
* Version 0.1
* Author: Francisco Rascón
          José González

* Implements the PID loops for position and torque control
******************************************************************************/

#ifndef PID_H
#define PID_H

#include <stdint.h>

#define PID_TOLERANCE 0.000f // 0.00%
#define PID_CONTROLLERS_NUMBER 4

// enum pid_status {
//    PID_DISABLED = 0, PID_ENABLED = 1
//};
#define PID_DISABLED 0
#define PID_ENABLED 1

// enum pid_id {
//    POS_PID_CONTROLLER_ID = 0, LVL_PID_CONTROLLER_ID = 1,
//    TOR_PID_CONTROLLER_ID = 2, TCL_PID_CONTROLLER_ID = 3,
//    PID_CONTROLLER_ID_ERROR = 4
//};
#define POS_PID_CONTROLLER_ID 0
#define LVL_PID_CONTROLLER_ID 1
#define TOR_PID_CONTROLLER_ID 2
#define TCL_PID_CONTROLLER_ID 3
#define PID_CONTROLLER_ID_ERROR 4

// enum pid_constant_index {
//    KP_INDEX = 0,
//    KI_INDEX = 1,
//    KD_INDEX = 2,
//};
#define KP_INDEX 0
#define KI_INDEX 1
#define KD_INDEX 2

// g stands for generic, so gPID ==> generic PID

struct gPID {
  float *lower_setpoints;
  float *upper_setpoints;
  float setpoint;
  float *measured_ouput;
  float control_output;
  float acum_error;
  float prev_error;

  float tacit_integrator;
  float pid_output;

  // k_coeffs[0] is Kp;
  // k_coeffs[1] is Ki;
  // k_coeffs[2] is Kd;
  float **k_coeffs;
  int16_t *control_ouput_signs;
  int16_t status;
  int16_t controller_id;
};

// ##########################################################################################
//
// DECLARATIONS OF MICRO-CONTROLLER INDEPENDENT PUBLIC FUNCTIONS
//
// ##########################################################################################
int16_t pid_config_controller(struct gPID *gpid, int16_t controller_id,
                              int16_t check_actual_controller_id);

float pid_get_control_output(struct gPID *gpid);
float pid_get_measured_output(struct gPID *gpid);
float pid_get_setpoint(struct gPID *gpid);
void pid_set_setpoint(struct gPID *gpid, float *setpoint);

int16_t pid_get_id(struct gPID *gpid);
int16_t pid_set_id(struct gPID *gpid, int16_t controller_id);

float pid_get_k(struct gPID *gpid, int16_t constant_index);
void pid_set_k(struct gPID *gpid, float *k, int16_t constant_index);
void pid_set_k_coeffs(struct gPID *gpid, float **k_coeffs);

float pid_get_lower_setpoint(struct gPID *gpid);
float pid_get_upper_setpoint(struct gPID *gpid);
void pid_get_lower_upper_setpoint(struct gPID *gpid, float *lower_setpoint,
                                  float *upper_setpoint);
void pid_set_lower_upper_setpoints(struct gPID *gpid, float *lower_setpoints,
                                   float *upper_setpoints);
int16_t pid_get_control_output_sign(struct gPID *gpid);
void pid_set_control_output_signs(struct gPID *gpid,
                                  int16_t *control_output_signs);

float pid_get_tacit_integrator(struct gPID *gpid);
float pid_get_pid_output(struct gPID *gpid);
// float pid_get_norm_factor(struct gPID* gpid);
// void pid_set_norm_factors(struct gPID* gpid, float* norm_factor);
int16_t pid_get_status(struct gPID *gpid);
void pid_set_status(struct gPID *gpid, int16_t pid_status);
int16_t pid_init(struct gPID *gpid, int16_t controller_id, float **k_coeffs,
                 float *lower_setpoints, float *upper_setpoints,
                 int16_t *control_output_signs);
void pid_run(struct gPID *gpid, float *measured_output_norm);
void pid_TL_run(struct gPID *gpid, float *measured_output, float interaction,
                float K_TL);

#endif
