/*******************************************************************************
* File:   sensors.c
* Version 0.1
* Author: Francisco Rascón
          José González

* Implements all the transformations routines necessary for the sensors.
******************************************************************************/

#include "sensors.h"
#include "constants.h"
#include <math.h>
#include <xc.h>

// extern int16_t flag_init; //GAP: flag to detect initilialization of the
// optical encoder angle and start applying the mech stop supervisor

extern void __attribute__((interrupt, auto_psv)) _QEIInterrupt(void) {
  _LATF5 ^= 1;
  _LATB8 = ~_LATF5;
  POSCNT = 0;

  // Clear QEI interrupt flag
  _QEIIF = 0;

  // GAP:
  // for the mechanical stop supervisor, I wait for the first time I detect
  // where the zero is, to start applying the supervisor (otherwise, I do not
  // know where I really am, and could try to block the control before the
  // mechanical stop))
  // if (flag_init == 0) flag_init=1;
}

void sensors_qei_init(int16_t encoder_max_counts) {
  // BIOMOT project is using an incremental optical encoder with multiplying
  // factor of 4

  // Configure QEI's pins as inputs.
  _TRISB3 = 0b1; // INDX
  _TRISB4 = 0b1; // QEA
  _TRISB5 = 0b1; // QEB
  // Configure QEI's pins as inputs of digital type, not analog type.
  _PCFG3 = 0b1; // INDX
  _PCFG4 = 0b1; // QEA
  _PCFG5 = 0b1; // QEB

  // Configure QEI Control Register

  // UDSRC (QEICON<0>). Position Counter Direction Selection Control bit. This
  // bit is only useful when QEI module is configured as a timer.
  // TQCS (QEICON<1>).  Timer Clock Source Select bit. This bit is only useful
  // when QEI module is configured as a timer.
  _POSRES = 0b1; // Index pulse resets position counter.
  // TQCKPS (QEICON<4:3>). Timer Input Clock Prescale Select bit. These bits are
  // only useful when QEI module is configured as a timer.
  // TQGATE (QEICON<5>). Timer Gated Time Accumulation Enable bit. These bits
  // are only useful when QEI module is configured as a timer gate.
  _PCDOUT = 0b0; // Position counter direction status output disabled (normal
                 // i/o pin operation)

#if SWAP_INC_OPT_ENC == 1
  _SWPAB = 0b1; // Phase A and Phase B inputs swapped
#else
  _SWPAB = 0b0; // Phase A and Phase B inputs not swapped
#endif

  //_QEIM = 0b111; // Quadrature encoder interface enabled (x4 mode) with
  // position counter reset by match (MAXCNT)
  _QEIM = 0b110; // Quadrature encoder interface enabled (x4 mode) with index
                 // pulse reset of position counter

  //_UPDN (QEICON<11>) Read only bit when QEIM = 1XX
  // INDEX (QEICON<12>) Index pin state status bit (read only)
  _QEISIDL = 0b0; // Continue module operation in idle mode
  _CNTERR = 0b0;  // Clean position count

  // Configure Digital Filter Control Register

  _QECK = 0b011; // QEA/QEB/INDX digital filter clock divide select bits. 1:16
                 // clook divide
  _QEOUT = 0b1;  // QEA/QEB/INDX digital filter output enabled
  _CEID = 0b1;   // Interrupts due to count errors are disabled.

  _IMV0 = 0;
  _IMV1 = 0;

  POSCNT = 0;

  // Example
  // encoder_max_counts = 2000 cpr
  // encoder_max_counts * 4 = 8000 cpr ==> from 0 to 7999 (last registrable
  // count index)
  // 8000 counts is equivalent to 360 degrees or 0 degrees.
  // 7999 corresponds to 7999/8000 = 359.955 degrees
  // Resolution 360 * (1/8000) = 0.045 degrees
  MAXCNT = (4 * encoder_max_counts) - 1;

  _QEIIP = 0b111; // Priority 1
  _QEIIF = 0b0;
}

int16_t sensors_get_incremental_encoder_angle(void) { return POSCNT; }

// CURRENT TRANSFORMATIONS ROUTINES

/* Read the value of the filtered current in amps [A]
 * 0 A ==> 2.5 V ==> 512 (For an ADC with Vref- = 0 V, Vref+ = 5 V and 10 bits)
 * 5 A ==> 3.5 V ==> 512 * (3.5 / 2.5) = 716.8
 */
float sensors_get_current_amps(int16_t current_digital) {
  // y = m * (x - 512)
  // m = 5/(716.8 - 512) = 5 / (((3.5/2.5)*512) - 512) = 5 / (512 * ((3.5/2.5) -
  // 1))
  //   = (5 * 2.5) / 512 = 25 / 1024
  // y = m * (x - 512) = (25/1024) * (x - 512) = ((25/1024) * x)  - 12.5
  return (((25.0f / 1024) * current_digital) - 12.5f);
}

// LOAD CELL TRANSFORMATIONS ROUTINES

float sensors_get_external_load_cell_force_newtons(
    int16_t external_load_cell_force_digital) {
  return ((0.8f * external_load_cell_force_digital) + 7.522f);
}

// TEMPERATURE TRANSFORMATIONS ROUTINES

/* Read the value of the filtered temperature in celsius */
int16_t sensors_get_temperature_celsius(int16_t temperature_digital) {
  if (temperature_digital < 102)
    return 0;
  else
    return (temperature_digital - 102) / 2;
}

// TORQUE TRANSFORMATIONS ROUTINE

void sensors_get_theoretical_torque_newtons(
    float *alpha_angle_rad, float *precompression,
    float *internal_load_cell_force, float *alpha_torque_Nm,
    float *internal_load_cell_torque_Nm) {
  // BIOMOT
  float A_term = sqrtf(0.003725f - (0.003500f * cos(*alpha_angle_rad)));
  *alpha_torque_Nm = 92.75f * (1.0f + ((*precompression - 0.015f) / A_term)) *
                     sin(*alpha_angle_rad);
  *internal_load_cell_torque_Nm = 0;
  //*theoretical_torque_by_internal_load_cell = (internal_load_cell_force *
  //(0.0026152f / A_term) * sin(alpha_angle_rad));
}
