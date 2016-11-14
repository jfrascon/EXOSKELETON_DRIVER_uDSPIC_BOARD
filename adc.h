/*******************************************************************************
* File: adc.c
* Version 0.1
* Author: Francisco Rascón
          Jose Gonzalez
* Drivers for the analog to digital conversion. Contains all functions and
 definitions related to sensors and data conversion.
******************************************************************************/

#ifndef ADC_H
#define ADC_H

#include <stdint.h>

#define NUM_ADC_CHANNELS 8
#define NUM_ADC_SAMPLES 10

enum adc_channel {
  INTERNAL_LOAD_CELL_CHANNEL = 0,
  CURRENT_SENSOR_CHANNEL = 1,
  FIRST_EXTRA_SENSOR_CHANNEL = 3,
  SECOND_EXTRA_SENSOR_CHANNEL = 4,
  THIRD_EXTRA_SENSOR_CHANNEL = 5,
  DIG_BOARD_TEMPERATURE_SENSOR_CHANNEL = 6
};

extern void __attribute__((__interrupt__, __auto_psv__)) _ADCInterrupt(void);
void adc_init();
int16_t adc_read(int16_t channel);

#endif
