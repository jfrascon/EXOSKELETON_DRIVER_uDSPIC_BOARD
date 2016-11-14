/*******************************************************************************
* File: sensors.h
* Version: 0.1
* Authors: Francisco Rascón
           José González
           Andrea Ortiz

* Drivers for the analog to digital conversion. Contains all functions and
 definitions related to sensors and data conversion.
******************************************************************************/

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

int16_t sensors_get_incremental_encoder_angle(void);
void sensors_qei_init(int16_t encoder_max_counts);
float sensors_get_current_amps(int16_t current_raw);
float sensors_get_external_load_cell_force_newtons(
    int16_t external_load_cell_measurement_digital);
int16_t sensors_get_temperature_celsius(int16_t temperature_digital);
void sensors_get_theoretical_torque_newtons(
    float *alpha_angle_rad, float *precompression,
    float *internal_load_cell_force, float *alpha_torque_Nm,
    float *internal_load_cell_torque_Nm);
float sensors_speed_read(int16_t speed);

#endif
