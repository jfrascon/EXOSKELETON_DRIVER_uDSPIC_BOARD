/*******************************************************************************
 * File:   PID.h
 * Version 0.1
 * Author: Francisco Rascón Crespo
           José González

 Implements the PID loops for position and torque control. The position control
 will have the controller_id = 0 and the torque control has the id = 1;
 This version implements the PID using the dsp.h library in order to improve
 speed of calculation, but has to be tested in a real applications!.
******************************************************************************/

#include "pid.h"
#include "constants.h"
#include <string.h> //memset
#include <xc.h>

// ##########################################################################################
//
// DECLARATIONS OF MICRO-CONTROLLER INDEPENDENT PRIVATE FUNCTIONS (ONLY
// AVAILABLE IN THIS FILE)
//
// ##########################################################################################

static void pid_check_k(float *k);

// ##########################################################################################
//
// DEFINITIONS OF MICRO-CONTROLLER INDEPENDENT PRIVATE FUNCTIONS (ONLY AVAILABLE
// IN THIS FILE)
//
// ##########################################################################################

// k must be from 0.0f to MAX_NORMALIZED_REAL_VALUE

static void pid_check_k(float *k) {
  if (*k < K_MIN)
    *k = K_MIN;
  else if (*k > K_MAX)
    *k = K_MAX;
}

// ##########################################################################################
//
// DEFINITIONS OF MICRO-CONTROLLER INDEPENDENT PUBLIC FUNCTIONS
//
// ##########################################################################################

int16_t pid_config_controller(struct gPID *gpid, int16_t controller_id,
                              int16_t check_actual_controller_id) {
  // Same controller ==> nothing to do
  if (check_actual_controller_id && (pid_get_id(gpid) == controller_id))
    return 1;

  if (pid_set_id(gpid, controller_id))
    return -1;

  // Clean PID history
  gpid->prev_error = 0.00f;
  gpid->acum_error = 0.00f;

  return 0;
}

float pid_get_control_output(struct gPID *gpid) {
  // if pid is disabled return 0
  if (!pid_get_status(gpid))
    return 0;

  return gpid->control_output;
}

/* Get the setpoint of the PID */
float pid_get_measured_output(struct gPID *gpid) {
  return *(gpid->measured_ouput);
}

/* Get the setpoint of the PID */
float pid_get_setpoint(struct gPID *gpid) { return gpid->setpoint; }

void pid_set_setpoint(struct gPID *gpid, float *setpoint) {
  if (*setpoint < pid_get_lower_setpoint(gpid) ||
      *setpoint > pid_get_upper_setpoint(gpid))
    pid_set_status(gpid, PID_DISABLED);
  else
    pid_set_status(gpid, PID_ENABLED);

  gpid->setpoint = *setpoint;
}

/* Get the controller_id that is being used */
int16_t pid_get_id(struct gPID *gpid) { return gpid->controller_id; }

/* Set the controller_id that will be used */
int16_t pid_set_id(struct gPID *gpid, int16_t controller_id) {

  if (controller_id == POS_PID_CONTROLLER_ID ||
      controller_id == LVL_PID_CONTROLLER_ID ||
      controller_id == TOR_PID_CONTROLLER_ID ||
      controller_id == TCL_PID_CONTROLLER_ID) {
    gpid->controller_id = controller_id;
    return 0;
  }

  gpid->controller_id = PID_CONTROLLER_ID_ERROR;
  return -1;
}

/*
 * kp, ki, kd from 0.0f to MAX_NORMALIZED_REAL_VALUE
 */
float pid_get_k(struct gPID *gpid, int16_t constant_index) {
  if (constant_index == KP_INDEX || constant_index == KI_INDEX ||
      constant_index == KD_INDEX)
    return gpid->k_coeffs[gpid->controller_id][constant_index];
  else
    return 0.0f;
}

/* Set the kp, ki, kd coefficient for the PID.
 * kp, ki, kd from 0.0f to MAX_NORMALIZED_REAL_VALUE
 */
void pid_set_k(struct gPID *gpid, float *k, int16_t constant_index) {
  if (constant_index == KP_INDEX || constant_index == KI_INDEX ||
      constant_index == KD_INDEX) {
    pid_check_k(k);
    gpid->k_coeffs[gpid->controller_id][constant_index] = *k;
  }
}

/* Set the coefficients for the PID.
 * k_coeffs[i] from 0.0f to MAX_NORMALIZED_REAL_VALUE
 */
void pid_set_k_coeffs(struct gPID *gpid, float **k_coeffs) {
  int16_t i;
  int16_t j;
  for (i = 0; i < PID_CONTROLLERS_NUMBER; i++) {
    for (j = 0; j < 3; j++)
      pid_check_k(k_coeffs[i] + j);
  }

  gpid->k_coeffs = k_coeffs;
}

float pid_get_lower_setpoint(struct gPID *gpid) {
  return gpid->lower_setpoints[gpid->controller_id];
}

float pid_get_upper_setpoint(struct gPID *gpid) {
  return gpid->upper_setpoints[gpid->controller_id];
}

void pid_get_lower_upper_setpoint(struct gPID *gpid, float *lower_setpoint,
                                  float *upper_setpoint) {
  *lower_setpoint = gpid->lower_setpoints[gpid->controller_id];
  *upper_setpoint = gpid->upper_setpoints[gpid->controller_id];
}

void pid_set_lower_upper_setpoint(struct gPID *gpid, float *lower_setpoints,
                                  float *upper_setpoints) {
  float *lower_upper_setpoints[2] = {lower_setpoints, upper_setpoints};
  float max;

  int16_t i;
  int16_t j;
  for (i = 0; i < PID_CONTROLLERS_NUMBER; i++) {
    for (j = 0; j < 2; j++) {
      if (lower_upper_setpoints[j][i] < -1.0f)
        lower_upper_setpoints[j][i] = -1.0f;
      else if (lower_upper_setpoints[j][i] > +1.0f)
        lower_upper_setpoints[j][i] = +1.0f;
    }

    if (lower_setpoints[i] > upper_setpoints[i]) {
      max = lower_setpoints[i];
      lower_setpoints[i] = upper_setpoints[i];
      upper_setpoints[i] = max;
    }
  }

  gpid->lower_setpoints = lower_setpoints;
  gpid->upper_setpoints = upper_setpoints;
}

int16_t pid_get_control_output_sign(struct gPID *gpid) {
  return gpid->control_ouput_signs[gpid->controller_id];
}

void pid_set_control_output_signs(struct gPID *gpid,
                                  int16_t *control_output_signs) {
  int16_t i;
  // Check all signs are +1 or -1, nothing more.
  for (i = 0; i < PID_CONTROLLERS_NUMBER; i++) {
    if (control_output_signs[i] >= 0)
      control_output_signs[i] = 1;
    else
      control_output_signs[i] = -1;
  }
  gpid->control_ouput_signs = control_output_signs;
}

/*
float pid_get_norm_factors(struct gPID* gpid)
{
    return gpid->norm_factors[gpid->controller_id];
}
 */

/*
void pid_set_norm_factor(struct gPID* gpid, float* norm_factors)
{
    int16_t i;
    for(i = 0; i < PID_CONTROLLERS_NUMBER; i++)
    {
        if(norm_factors[i] == 0.0f)
            norm_factors[i] = 1.0f;
    }

    gpid->norm_factors = norm_factors;
}*/

/* Check if the position PID is enabled or not */
int16_t pid_get_status(struct gPID *gpid) { return gpid->status; }

/* Set the flag enable for the position PID */
void pid_set_status(struct gPID *gpid, int16_t pid_status) {
  if (pid_status == PID_ENABLED) {
    gpid->status = PID_ENABLED;
    // JF20160628
    // ALERT_LED_OFF;
    // OK_LED_ON;
    // JF20160628
    return;
  }
  gpid->status = PID_DISABLED;
  // JF20160628
  // ALERT_LED_ON;
  // OK_LED_OFF;
  // JF20160628
}

/* Get the setpoint of the PID */
float pid_get_tacit_integrator(struct gPID *gpid) {
  return gpid->tacit_integrator;
}

/* Get the setpoint of the PID */
float pid_get_pid_output(struct gPID *gpid) { return gpid->pid_output; }

int16_t pid_init(struct gPID *gpid, int16_t controller_id, float **k_coeffs,
                 float *lower_setpoints, float *upper_setpoints,
                 int16_t *control_output_signs) {
  memset(gpid, 0, sizeof(struct gPID));

  pid_set_k_coeffs(gpid, k_coeffs);
  pid_set_lower_upper_setpoint(gpid, lower_setpoints, upper_setpoints);
  pid_set_control_output_signs(gpid, control_output_signs);

  if (pid_config_controller(gpid, controller_id, 0))
    return -1;

  pid_set_status(gpid, PID_DISABLED);
  return 0;
}

void pid_run(struct gPID *gpid, float *measured_output) {

  // int16_t measured_output_norm_within_range = 1;
  // if(measured_output_norm < pid_get_lower_setpoint(gpid) ||
  // measured_output_norm > pid_get_upper_setpoint(gpid))
  //{
  //    measured_output_norm_within_range = 0;
  //}
  // else
  //{
  //}
  // pid_set_status(gpid, (pid_get_status(gpid) &
  // normalized_measured_output_within_range));

  gpid->measured_ouput = measured_output;

  // If PID is disabled ==> nothing to do with it ==> function ends here!!
  if (!pid_get_status(gpid))
    return;

  float measured_output_backup = *(gpid->measured_ouput);
  float tolerance_lower_limit = gpid->setpoint - PID_TOLERANCE;
  float tolerance_upper_limit = gpid->setpoint + PID_TOLERANCE;

  if (tolerance_lower_limit < pid_get_lower_setpoint(gpid))
    tolerance_lower_limit = pid_get_lower_setpoint(gpid);

  if (tolerance_upper_limit > pid_get_upper_setpoint(gpid))
    tolerance_upper_limit = pid_get_upper_setpoint(gpid);

  if ((*(gpid->measured_ouput) >= tolerance_lower_limit) &&
      (*(gpid->measured_ouput) <= tolerance_upper_limit))
    *(gpid->measured_ouput) = gpid->setpoint;

  // ===================== CALCULATE CONTROL OUTPUT ==========================

  float error = (gpid->setpoint - *(gpid->measured_ouput));

  if (gpid->k_coeffs[gpid->controller_id][1] != 0) // if ( Ki != 0 )
    gpid->acum_error += (gpid->k_coeffs[gpid->controller_id][1] *
                         error); // gpid->acum_error +=  error;
  else
    gpid->acum_error = 0;

  gpid->control_output =
      ((gpid->k_coeffs[gpid->controller_id][0] * error) + gpid->acum_error +
       (gpid->k_coeffs[gpid->controller_id][2] * (error - gpid->prev_error)));
  gpid->prev_error = error;

  // =========================================================================

  // Restore original measured output. This procedure is useful in case of the
  // original measured output was between
  // lower limit and upper limit but not exactly the setpoint.
  *(gpid->measured_ouput) = measured_output_backup;
}

void pid_TL_run(struct gPID *gpid, float *measured_output, float interaction,
                float K_TL) {
  // GAP: variable to select the type of controller
  //    enum tl_control_type tl_control_type = TL_CONTROL_TYPE;
  uint8_t tl_control_type = -1;

  // int16_t measured_output_norm_within_range = 1;
  // if(measured_output_norm < pid_get_lower_setpoint(gpid) ||
  // measured_output_norm > pid_get_upper_setpoint(gpid))
  //{
  //    measured_output_norm_within_range = 0;
  //}
  // else
  //{
  //}
  // pid_set_status(gpid, (pid_get_status(gpid) &
  // normalized_measured_output_within_range));

  gpid->measured_ouput = measured_output;

  // If PID is disabled ==> nothing to do with it ==> function ends here!!
  if (!pid_get_status(gpid))
    return;

  float measured_output_backup = *(gpid->measured_ouput);
  float tolerance_lower_limit = gpid->setpoint - PID_TOLERANCE;
  float tolerance_upper_limit = gpid->setpoint + PID_TOLERANCE;

  if (tolerance_lower_limit < pid_get_lower_setpoint(gpid))
    tolerance_lower_limit = pid_get_lower_setpoint(gpid);

  if (tolerance_upper_limit > pid_get_upper_setpoint(gpid))
    tolerance_upper_limit = pid_get_upper_setpoint(gpid);

  if ((*(gpid->measured_ouput) >= tolerance_lower_limit) &&
      (*(gpid->measured_ouput) <= tolerance_upper_limit))
    *(gpid->measured_ouput) = gpid->setpoint;

  // ===================== CALCULATE CONTROL OUTPUT ==========================

  float error = (gpid->setpoint - *(gpid->measured_ouput));

  if (gpid->k_coeffs[gpid->controller_id][1] != 0.0f)
    gpid->acum_error += (gpid->k_coeffs[gpid->controller_id][1] *
                         error); // gpid->acum_error +=  error;
  else
    gpid->acum_error = 0.0f;

  gpid->pid_output =
      ((gpid->k_coeffs[gpid->controller_id][0] * error) + gpid->acum_error +
       (gpid->k_coeffs[gpid->controller_id][2] * (error - gpid->prev_error)));

  if (K_TL == 0.0f)
    gpid->tacit_integrator = 0.0f;
  else {
    // calculate the integrator for the tacit learning.
    // if ((interaction < 0.05) && (interaction > 0.01))
    //        interaction = 0;
    // GAP:
    if (tl_control_type == TL_ALPHA_ANGLE_CTRL ||
        tl_control_type == TL_FIX_LINK_ANGLE_CTRL)
      gpid->tacit_integrator += K_TL * (interaction);
    else if (tl_control_type == TL_ALPHA_SPEED_CTRL ||
             tl_control_type == TL_FIX_LINK_SPEED_CTRL)
      gpid->tacit_integrator =
          K_TL * (interaction); // GAP: not integrator anymore
    // GAP
  }

  // Add the interaction to the PID output.
  // GAP: sign changed
  gpid->control_output = gpid->pid_output + gpid->tacit_integrator;
  // gpid->control_output = gpid->pid_output - gpid->tacit_integrator;
  // GAP

  gpid->prev_error = error;

  // =========================================================================

  // Restore original measured output. This procedure is useful in case of the
  // original measured output was between
  // lower limit and upper limit but not exactly the setpoint.
  *(gpid->measured_ouput) = measured_output_backup;
}
