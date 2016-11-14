/*******************************************************************************
* File:   adc.c
* Version 0.1
* Author: Francisco Rascón
          Jose Gonzalez

* Drivers for the analog to digital conversion. Contains all functions and
 definitions related to data conversion.
******************************************************************************/

#include "adc.h"
#include <xc.h>

static int16_t adcbuf_history[NUM_ADC_CHANNELS][NUM_ADC_SAMPLES] = {{0}};
static uint16_t adcbuf_accumulator[NUM_ADC_CHANNELS] = {0};
static uint16_t measure = 0;
static uint16_t j = 0;

/* ADC Setup.
 * Clock = 64 MHz
 * Tcy = 1/(64 MHz/4) = 62,5 ns
 * TAD = Tcy (ADCS+1)/2 => Tad = 2 us
 * Auto sample time = 13 TAD
 * Conversion time = 12 TAD (always)
 * AD time = 25 TAD = 50 us
 * AD interrupt = 6 x 50 us = 300 us
 * For testing I have changed the ADCS to 5 and the interrupt every 2 samples.
 * This means that the AD interrupt should happen every 9,37 us.
 */
void adc_init() {
  // Auto conv, output without signal
  ADCON1 = 0x00E4;
  // Interrupt each 6 conv
  ADCON2 = 0x0414;
// ADCON2 = 0x0404; // test to interrupt every 2 samples
// The following lines are only valid if no quadrature sensor is used
#ifndef QUADRATURE_MODULE_ENABLED
  // TAD = 13; ADCS = 60
  ADCON3 = 0x0D3C;
  // ADCON3 = 0x0D05;  //to test the ADCS = 5
  ADCHS = 0x0000;
  // RB0, RB1, RB3, RB4, RB5, RB6, RB7 as analog
  ADPCFG = 0xFF04;
  // Scan 0, 1, 2, 3, 4, 5, 6, 7
  ADCSSL = 0x00FF;
#else
// Init the ADC for not reading inputs: 3, 4 and 5.
#endif
  // Clear interrupt flag
  _ADIF = 0;
  // ADC ON
  ADCON1bits.ADON = 1;
}

/* AD Conversion Interrupt */
extern void __attribute__((__interrupt__, __auto_psv__)) _ADCInterrupt(void) {
  // restart_wdt();

  measure = ADCBUF0; // Internal load cell (ILC)
  adcbuf_accumulator[0] =
      adcbuf_accumulator[0] - adcbuf_history[0][j] + measure;
  adcbuf_history[0][j] = measure;

  measure = ADCBUF1; // Current
  adcbuf_accumulator[1] =
      adcbuf_accumulator[1] - adcbuf_history[1][j] + measure;
  adcbuf_history[1][j] = measure;

#ifndef QUADRATURE_MODULE_ENABLED // For channel 3, 4 and 5
  measure = ADCBUF3;              // External load cell (ELC)
  adcbuf_accumulator[3] =
      adcbuf_accumulator[3] - adcbuf_history[3][j] + measure;
  adcbuf_history[3][j] = measure;
#endif

  j++;

  if (j == NUM_ADC_SAMPLES) // Circular buffer
    j = 0;

  _ADIF = 0;
  // cli_ADC();
}

int16_t adc_read(int16_t channel) {
  if (channel < 0 || channel > (NUM_ADC_CHANNELS - 1))
    return 0;

  return (int16_t)((((float)adcbuf_accumulator[channel]) / NUM_ADC_SAMPLES) +
                   0.5f);
}

/* AD Conversion Interrupt
extern void __attribute__((__interrupt__, __auto_psv__)) _ADCInterrupt(void)
{

    //restart_wdt();

    // Shift values one cell to the right. adcbuf_history[i][end] is lost.
    int16_t i;
    for(i = 0; i < NUM_ADC_CHANNELS; i++)
        memcpy(adcbuf_history[i] + 1, adcbuf_history[i], ((NUM_ADC_SAMPLES - 1)
* sizeof (int16_t)));

    // Read the actual value.
    adcbuf_history[0][0] = ADCBUF0;
    //adcbuf_history[1][0] = ADCBUF1;
    //adcbuf_history[2][0] = ADCBUF2; NOT USED
#ifndef QUADRATURE_MODULE_ENABLE
    adcbuf_history[3][0] = ADCBUF3;
    //adcbuf_history[4][0] = ADCBUF4;
    //adcbuf_history[5][0] = ADCBUF5;
#endif
    //adcbuf_history[6][0] = ADCBUF6;
    //adcbuf_history[7][0] = ADCBUF7;
    //adcbuf_history[8][0] = ADCBUF8; NOT USED

    _ADIF = 0;
    //cli_ADC();
}

// Read ADC channel
int16_t adc_read(int16_t channel)
{

    if(channel < 0 || channel > (NUM_ADC_CHANNELS - 1))
        return 0;

    // Reset adcbuf_accumulator[channel]
    adcbuf_accumulator[channel] = 0;

    // Calculate the mean of the selected adc channel
    int i;
    for(i = 0; i < NUM_ADC_SAMPLES; i++)
        adcbuf_accumulator[channel] += adcbuf_history[channel][i];

    return (int16_t) ((((float) adcbuf_accumulator[channel]) / NUM_ADC_SAMPLES)
+ 0.5f);

    //return (adcbuf_accumulator[channel] / NUM_ADC_SAMPLES);
}*/
