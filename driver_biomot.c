/*******************************************************************************
* File:   H2R_driver_V3.c
* Version: 100.1
* Author: Francisco Rascón
          Jose Gonzalez
          Andrea Ortiz
          Marco Matteo Bassa
          Francisco Resquin

* Main Function of H2R Project
* MCU Configuration MACROS
  For more info see:
  file:///Applications/microchip/xc16/v1.21/docs/config_docs/30F4011.html
*******************************************************************************/

#include "adc.h"
#include "can1mbps.h"
#include "constants.h"
#include "motor.h"
#include "pid.h"
#include "pwm.h"
#include "sensors.h"
#include "spi.h"
#include "timer.h"

// include constants.h before this files
#include <dsp.h>
#include <libpic30.h>
#include <math.h>
#include <string.h>
#include <xc.h>

// fract gpid_abc_coeff[3] __attribute__((section(".xbss, bss, xmemory")));
// fract gpid_control_hist[3] __attribute__((section(".ybss, bss, ymemory")));

static void db_apply_control_action(struct gPID *gpid, struct Motor *motor,
                                    struct PWM *pwm);
static void db_calibrate_magnetic_encoder(int16_t *mag_enc_ref_pos_digital);
static void db_create_joint_can_packet(int16_t *joint_can_packet,
                                       int16_t fix_link_angle_digital_16,
                                       int16_t alpha_angle_digital_16,
                                       int16_t extra_value_1,
                                       int16_t extra_value_2);
static void db_create_pid_can_packet(int16_t *pid_can_packet,
                                     struct gPID *gpid);
static void db_create_sensor_can_packet(int16_t *sensor_can_packet,
                                        enum error_code error_4,
                                        int16_t velocity_rpm_14,
                                        int16_t current_digital_10,
                                        int16_t lever_arm_digital_16);
static void db_error(int16_t board_id, int16_t *sensor_can_packet,
                     enum error_code error, struct gPID *gpid,
                     struct Motor *motor);
static int16_t db_receive_can_data(int16_t board_id, struct gPID *gpid,
                                   float **vec_measured_output_norm,
                                   int16_t *rx_can_packet,
                                   float *precompression, int32_t *event,
                                   float *aux_var,
                                   int16_t *mag_enc_ref_pos_digital,
                                   int16_t *opt_enc_ref_pos_digital);
static float *db_select_measured_output_norm(struct gPID *gpid,
                                             float **vec_measured_output);

static void db_second_light_code();
static void db_third_light_code();
static void db_welcome_led();

// static void db_create_torque_can_packet(int16_t* torque_can_packet, int16_t
// alpha_torque_digital, int16_t internal_load_cell_torque_digital, uint16_t
// seq_16);

// GAP: apply PWM with push off
// static void db_apply_fwd_control_action(float control_output, struct Motor*
// motor, struct PWM* pwm);

// GAP: event reception function
// static int16 db_receive_event(int16 board_id, int16* rx_can_packet, float*
// event);

// static int16_t exponential_weighted_moving_average(int16_t y_n_1, int16_t
// x_n, float* A_coeff, float* B_coeff);
static int16_t exponential_weighted_moving_average(int16_t y_n_1, int16_t x_n,
                                                   int16_t q);

int16_t flag_init = 0; // GAP: flag to detect initilialization of the optical
                       // encoder angle and start applying the mech stop
                       // supervisor
// extern
uint16_t aux_controller_id;
uint16_t aux_variable_id;
float aux_variable[6];

// Oscillator: Primary Oscillator & XT w/PLL 8x & Sw Disabled, Mon Disabled.
_FOSC(PRI &XT_PLL8 &CSW_FSCM_OFF);

// Watch Dog: WDT_OFF
_FWDT(WDT_OFF);

// FBORPOR: POR Timer Value 64 ms & Brown Out Voltage 2.7V & PBOR Enable Disable
//_FBORPOR(PBOR_OFF & MCLR_DIS)
_FBORPOR(PWRT_64 &BORV27 &PBOR_ON &MCLR_DIS);

static void db_calibrate_magnetic_encoder(int16_t *mag_enc_ref_pos_digital) {

  int16_t alpha_angle_digital = 0;
  int16_t agc_digital_aasae = 0;
  int16_t mag_digital_aasae = 0;
  int16_t error_aasae = 0;

  while (spi_get_agc_mag_ang_absolute_encoder(
      ALPHA_ANGLE_SPI_ABS_ENC_CHANNEL, &agc_digital_aasae, &mag_digital_aasae,
      &alpha_angle_digital, &error_aasae))
    ;

  uint16_t effective_max_counts_spi_abs_enc = MAX_COUNTS_IN_SPI_ABS_ENC + 1;
  uint16_t half_effective_max_counts_spi_abs_enc =
      effective_max_counts_spi_abs_enc >> 1;

  if (alpha_angle_digital > half_effective_max_counts_spi_abs_enc)
    alpha_angle_digital -= effective_max_counts_spi_abs_enc;

  *mag_enc_ref_pos_digital += alpha_angle_digital;

  if (*mag_enc_ref_pos_digital >= effective_max_counts_spi_abs_enc)
    *mag_enc_ref_pos_digital -= effective_max_counts_spi_abs_enc;
  else if (*mag_enc_ref_pos_digital < 0)
    *mag_enc_ref_pos_digital += effective_max_counts_spi_abs_enc;

  spi_set_zero_position_absolute_encoder(ALPHA_ANGLE_SPI_ABS_ENC_CHANNEL,
                                         *mag_enc_ref_pos_digital, 0);
}

static void db_welcome_led() {
  int16_t i = 0;

  while (i < 2) {

    ALERT_LED_ON;
    OK_LED_OFF;
    CAN_LED_OFF;
    __delay_ms(125);

    ALERT_LED_ON;
    OK_LED_ON;
    CAN_LED_OFF;
    __delay_ms(125);

    ALERT_LED_ON;
    OK_LED_ON;
    CAN_LED_ON;
    __delay_ms(125);

    ALERT_LED_OFF;
    OK_LED_OFF;
    CAN_LED_OFF;
    __delay_ms(125);

    ALERT_LED_OFF;
    OK_LED_OFF;
    CAN_LED_ON;
    __delay_ms(125);

    ALERT_LED_OFF;
    OK_LED_ON;
    CAN_LED_ON;
    __delay_ms(125);

    ALERT_LED_ON;
    OK_LED_ON;
    CAN_LED_ON;
    __delay_ms(125);

    ALERT_LED_OFF;
    OK_LED_OFF;
    CAN_LED_OFF;
    __delay_ms(125);

    i++;
  }
}

static void db_second_light_code() {
  int16_t i = 0;

  while (i < 8) {
    ALERT_LED_ON;
    CAN_LED_ON;
    OK_LED_ON;
    __delay_ms(125);

    ALERT_LED_OFF;
    CAN_LED_OFF;
    OK_LED_OFF;
    __delay_ms(125);

    i++;
  }
}

static void db_third_light_code() {
  int16_t i = 0;

  // CAN_LED_OFF;
  // OK_LED_OFF;
  ALERT_LED_OFF;

  while (i < 8) {
    ALERT_LED_ON;
    __delay_ms(125);

    ALERT_LED_OFF;
    __delay_ms(125);

    i++;
  }
}

static int16_t db_receive_can_data(int16_t board_id, struct gPID *gpid,
                                   float **vec_measured_output_norm,
                                   int16_t *rx_can_packet,
                                   float *precompression, int32_t *event,
                                   float *aux_var,
                                   int16_t *mag_enc_ref_pos_digital,
                                   int16_t *opt_enc_ref_pos_digital) {
  // Read from CAN bus. id is -1 if there isn't a suitable message for this
  // board.
  int32_t can_id = can_receive(rx_can_packet);

  if (can_id != -1) {
    int16_t message_id = can_id - board_id;
    int16_t *controller_id = (int16_t *)rx_can_packet + 2;
    float *ptr_to_rx_value = (float *)rx_can_packet;
    int16_t controller_config;

    // Common code for messages sent to driver board
    switch (message_id) {
    case KP_MESSAGE_ID:
    case KI_MESSAGE_ID:
    case KD_MESSAGE_ID:
    case SETPOINT_MESSAGE_ID:
    case PRECOMPRESSION_MESSAGE_ID: {
      controller_config =
          pid_config_controller(gpid, ((*controller_id) & 0x00FF), 1);

      aux_controller_id = (*controller_id) & 0x00FF;
      // Error: controller_id not recognized
      if (controller_config == -1)
        return -1;

      break;
    }

    default: { break; }
    }

    // Specific code for messages sent to driver board
    switch (message_id) {
    // Receives Kp constant and the PID mode in format [MSB X X LSB]
    case KP_MESSAGE_ID: {
      // Controller_id changed
      if (controller_config == 0)
        pid_set_setpoint(gpid, db_select_measured_output_norm(
                                   gpid, vec_measured_output_norm));

      pid_set_k(gpid, ptr_to_rx_value, KP_INDEX);
      break;
    }
    // Receives Ki constant and the PID mode in format [MSB X X LSB]
    case KI_MESSAGE_ID: {
      // Controller_id changed
      if (controller_config == 0)
        pid_set_setpoint(gpid, db_select_measured_output_norm(
                                   gpid, vec_measured_output_norm));

      pid_set_k(gpid, ptr_to_rx_value, KI_INDEX);
      break;
    }
    // Receives D constant and the PID mode in format [MSB X X LSB]
    case KD_MESSAGE_ID: {
      // Controller_id changed
      if (controller_config == 0)
        pid_set_setpoint(gpid, db_select_measured_output_norm(
                                   gpid, vec_measured_output_norm));

      pid_set_k(gpid, ptr_to_rx_value, KD_INDEX);
      break;
    }
    // Receives setpoint and controller mode and start data transmission (CM)
    // [SP_MSB X X SP_LSB CM]
    case SETPOINT_MESSAGE_ID: {
      pid_set_setpoint(gpid, ptr_to_rx_value);
      break;
    }
    // Stop data transmission
    case PRECOMPRESSION_MESSAGE_ID: {
      // Controller_id changed
      if (controller_config == 0)
        pid_set_setpoint(gpid, db_select_measured_output_norm(
                                   gpid, vec_measured_output_norm));

      *precompression = *ptr_to_rx_value;

      _LATB8 ^= 1;
      break;
    }

    case EVENT_MESSAGE_ID: {
      *event = *((int32_t *)ptr_to_rx_value);
      break;
    }

    case REF_ALPHA_ANGLE_MESSAGE_ID: {
      db_calibrate_magnetic_encoder(mag_enc_ref_pos_digital);
      break;
    }

    case REF_FIX_LINK_ANGLE_MESSAGE_ID: {
      *opt_enc_ref_pos_digital = POSCNT;

      // if (*opt_enc_ref_pos_digital < 0)
      //    *opt_enc_ref_pos_digital += (MAXCNT + 1);
      // else if (*opt_enc_ref_pos_digital >= (MAXCNT + 1))
      //    *opt_enc_ref_pos_digital -= (MAXCNT + 1);

      break;
    }

    case ENCODERS_CALIBRATION_MESSAGE_ID: {
      db_calibrate_magnetic_encoder(mag_enc_ref_pos_digital);
      *opt_enc_ref_pos_digital = POSCNT;
      break;
    }

    case AUX_VAR_MESSAGE_ID: {
      aux_variable_id = (*controller_id) & 0x00FF;
      aux_variable[aux_variable_id] = *ptr_to_rx_value;
      break;
    }

    default: {
      //*event = 0;
      // NOTHING TO DO
    }
    }
  }

  return 0;
}

static void db_error(int16_t board_id, int16_t *sensor_can_packet,
                     enum error_code error, struct gPID *gpid,
                     struct Motor *motor) {
  motor_set_status(motor, MOTOR_DISABLED);
  motor_set_dir(motor, MOTOR_STOP, 1);
  pid_set_status(gpid, PID_DISABLED);
  db_create_sensor_can_packet(sensor_can_packet, error, 0x07FF, 0x01FF, 0x7FFF);
  can_send(board_id + SENSORS_MESSAGE_ID, sensor_can_packet);
  __delay_us(200);
}

/* Create a position package:
 *   fix_link_angle in digital mode (15 bits)
 *   alpha_angle in digital mode (15 bits)
 *   extra_value_1 in digital mode (16 bits).
 *   extra_value_2 in digital mode (2 bits).
 */
static void db_create_joint_can_packet(int16_t *joint_can_packet,
                                       int16_t fix_link_angle_digital_16,
                                       int16_t alpha_angle_digital_16,
                                       int16_t extra_value_1,
                                       int16_t extra_value_2) {
  // Clean the packet
  memset(joint_can_packet, 0, NUM_WORDS_2BYTES_TX * sizeof(int16_t));

  joint_can_packet[0] = fix_link_angle_digital_16;
  joint_can_packet[1] = alpha_angle_digital_16;
  joint_can_packet[2] = extra_value_1;
}

static void db_create_pid_can_packet(int16_t *pid_can_packet,
                                     struct gPID *gpid) {
  // Clean the packet
  memset(pid_can_packet, 0, NUM_WORDS_2BYTES_TX * sizeof(int16_t));

  // setpoint        from -1.0f to MAX_NORMALIZED_REAL_VALUE
  // control_output  from -1.0f to MAX_NORMALIZED_REAL_VALUE
  // measured_output from -1.0f to MAX_NORMALIZED_REAL_VALUE
  pid_can_packet[0] = Float2Fract(pid_get_setpoint(gpid));
  pid_can_packet[1] = Float2Fract(pid_get_control_output(gpid));
  pid_can_packet[2] = Float2Fract(pid_get_measured_output(gpid));
}

/* Send the sensor data through CAN
 * Creates a package to send sensor data.
 * Data is stored in 5 packages:
 *  error (4 bits)
 *  velocity (14 bits)
 *  temperature (10 bits)
 *  current (10 bits)
 *  load cell force (10 bits)
 */
static void db_create_sensor_can_packet(int16_t *sensor_can_packet,
                                        enum error_code error_4,
                                        int16_t velocity_rpm_14,
                                        int16_t current_digital_10,
                                        int16_t lever_arm_digital_16) {
  // Reset the packet
  memset(sensor_can_packet, 0, NUM_WORDS_2BYTES_TX * sizeof(int16_t));

  sensor_can_packet[0] = ((error_4 & 0x000F) | (velocity_rpm_14 << 4));
  sensor_can_packet[1] = (((velocity_rpm_14 & 0x3000) >> 12) |
                          current_digital_10); // GAP: still 4 MSB free
  sensor_can_packet[2] = lever_arm_digital_16;
}

static float *db_select_measured_output_norm(struct gPID *gpid,
                                             float **vec_measured_output_norm) {
  // measured_output must be, in normal conditions, from -1.0f (-100%) to +1.0f
  // (+100%) (actually MAX_NORMALIZED_REAL_VALUE, 99.9969482%)
  float *measured_output_norm;

  switch (pid_get_id(gpid)) {
  // For position PID
  case POS_PID_CONTROLLER_ID: {
    measured_output_norm = vec_measured_output_norm[0];
    break;
  }
  // For speed PID
  case LVL_PID_CONTROLLER_ID: {
    measured_output_norm = vec_measured_output_norm[1];
    break;
  }
  // For torque PID
  case TOR_PID_CONTROLLER_ID:
  case TCL_PID_CONTROLLER_ID: {
    measured_output_norm = vec_measured_output_norm[2];
    break;
  }
  case PID_CONTROLLER_ID_ERROR:
  default: {
    // nothing to do. This case never occurs
  }
  }

  return measured_output_norm;
}

/*static void db_apply_fwd_control_action(float control_output, struct Motor*
motor, struct PWM* pwm)
{
    if(control_output <= -0.01f) // control_output <= -1%
        motor_set_dir(motor, MOTOR_CW, 1);
    else if(control_output >= 0.01f) // control_output >= +1%
        motor_set_dir(motor, MOTOR_CCW, 1);
    else //control_output is > -1% && control_output is < +1%
        motor_set_dir(motor, MOTOR_STOP, 1);

    pwm_set_duty_cycle(pwm, (fabsf(control_output) + 0.09f));
}
 */
static void db_apply_control_action(struct gPID *gpid, struct Motor *motor,
                                    struct PWM *pwm) {
  /*
  float control_output;
  if(joint_type == LEFT_JOINT)
      control_output = -pid_get_control_output(gpid);
  else
      control_output = pid_get_control_output(gpid);
   */

  float control_output =
      pid_get_control_output_sign(gpid) * pid_get_control_output(gpid);

  // ESCON DRIVER WORKS WITH DUTY CYCLES BETWEEN 10% AND 90% SO WE ADD AN OFFSET
  // TO THE CONTROL OUTPUT
  // OF 9% IN ORDER TO SPEED UP THE MOTOR RESPONSE.
  // if control_output is > -1% && control_output is < +1% the motor is not
  // commanded in any direction because
  // the duty cycle would be less than 10%
  //
  // if control_output >= +1% (positive) ==> duty_cycle >= 10% and CCW turn
  // if control_output <= -1% (negative) ==> duty_cycle >= 10% and CW turn
  //
  //  control_output = +1% ==> |+1%| + 9% ==> duty_cycle = 10% y CCW
  //  control_output = -1% ==> |-1%| + 9% ==> duty_cycle = 10% y  CW
  //  control_output = +2% ==> |+2%| + 9% ==> duty_cycle = 11% y CCW
  //  control_output = -2% ==> |+2%| + 9% ==> duty_cycle = 11% y  CW
  // ...
  //  control_output = +81% ==> |+81%| + 9% ==> abs_control_output = 90% y CCW
  //  control_output = -81% ==> |+81%| + 9% ==> abs_control_output = 90% y  CW

  if (control_output <= -0.01f) // control_output <= -1%
    motor_set_dir(motor, MOTOR_CW, 1);
  else if (control_output >= 0.01f) // control_output >= +1%
    motor_set_dir(motor, MOTOR_CCW, 1);
  else // control_output is > -1% && control_output is < +1%
    motor_set_dir(motor, MOTOR_STOP, 1);

  pwm_set_duty_cycle(pwm, (fabsf(control_output) + 0.09f));
}

// static int16_t exponential_weighted_moving_average(int16_t y_n_1, int16_t
// x_n, float* A_coeff, float* B_coeff)

static int16_t exponential_weighted_moving_average(int16_t y_n_1, int16_t x_n,
                                                   int16_t q) {

  // y[n] = A * y[n - 1] + B * x[n]
  // A_coeff = 1 - 1/2^q  ,  B_coeff = 1/2^q
  // y[n] = (1 - 1/2^q) * y[n - 1] + 1/2^q * x[n] = y[n-1] - 1/2^q * y[n - 1] +
  // 1/2^q * x[n]
  //      = y[n-1] - y[n-1] >> q + x[n] >> q;

  // return (int16_t) (((*A_coeff) * y_n_1) + ((*B_coeff) * x_n));
  return (y_n_1 - (y_n_1 >> q) + (x_n >> q));
}

int16_t main(void) {
  uint16_t board_id = BOARD_ID;
  uint16_t controller_id = PID_CONTROLLER_ID;
  // uint16_t flag_controller_id = 0;
  aux_controller_id = TOR_PID_CONTROLLER_ID;

  int16_t fix_link_angle_digital;
  float fix_link_angle_norm;
  float fix_link_angle_deg;
  float fix_link_angle_deg_previous = 0.0f;
  uint16_t fix_link_speed_count = 0;
  float fix_link_speed_accumulator = 0.0f;
  float fix_link_speed_history[SPEED_COMP_AVG_COUNT] = {0.0f};
  float fix_link_speed = 0.0f;
  float fix_link_speed_current = 0.0f;

  // Alpha Angle Spi Absolute Encoder ==> aasae
  int16_t error_aasae;
  int16_t agc_digital_aasae;
  int16_t mag_digital_aasae;
  int16_t alpha_angle_digital;
  int16_t alpha_angle_digital_backup;
  float alpha_angle_norm;
  float alpha_angle_rad;
  float alpha_angle_deg;
  float alpha_angle_deg_previous = 0.0f;

  uint16_t effective_max_counts_spi_abs_enc = MAX_COUNTS_IN_SPI_ABS_ENC + 1;
  uint16_t half_effective_max_counts_spi_abs_enc =
      effective_max_counts_spi_abs_enc >> 1; //(divide by 2);
  uint16_t alpha_speed_count = 0;
  float alpha_speed_accumulator = 0.0f;
  float alpha_speed_history[SPEED_COMP_AVG_COUNT] = {0.0f};
  float alpha_speed = 0.0f;
  float alpha_speed_current = 0.0f;

  int16_t lever_arm_angle_digital;
  float lever_arm_angle_norm;
  float lever_arm_angle_norm_aux;
  float lever_arm_angle_deg;
  float lever_arm_angle_deg_previous = 0.0f;
  uint16_t lever_arm_speed_count = 0;
  float lever_arm_speed_accumulator = 0.0f;
  float lever_arm_speed_history[SPEED_COMP_AVG_COUNT] = {0.0f};
  float lever_arm_speed = 0.0f;
  float lever_arm_speed_current = 0.0f;

  // enum po_controller_mode po_controller_mode = PO_CONTROLLER_MODE;

  // Torque variables
  float torque_norm;

  float alpha_torque_Nm;
  float alpha_torque_norm;

  int16_t external_load_cell_force_digital;
  int16_t external_load_cell_force_digital_backup = 0;

  float internal_load_cell_force_N;
  float internal_load_cell_torque_Nm;
  float internal_load_cell_torque_norm;

  float precompression = 0.0f;
  float precompression_ = 0.002f;
  int32_t event = 0;

  float aux_var[6] = {0, 0, 0, 0, 0, 0};

  float aux_control_output = 0;

  // float A_ewma = 0.9375f;
  // float B_ewma = 0.0625f;

  float *vec_measured_output_norm[3] = {0};

  float k_pos[3] = {DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD};
  float k_lvl[3] = {DEFAULT_LVL_KP, DEFAULT_LVL_KI, DEFAULT_LVL_KD};
  float k_tor[3] = {DEFAULT_TOR_KP, DEFAULT_TOR_KI, DEFAULT_TOR_KD};
  float k_tcl[3] = {DEFAULT_TCL_KP, DEFAULT_TCL_KI, DEFAULT_TCL_KD};
  float *gpid_k[4] = {k_pos, k_lvl, k_tor, k_tcl};
  int16_t control_output_signs[PID_CONTROLLERS_NUMBER] = {
      -1, FIX_LINK_ANGLE_SIGN, FIX_LINK_ANGLE_SIGN, FIX_LINK_ANGLE_SIGN};

  float lower_setpoints[4] = {LOWER_POS_SETPOINT, LOWER_LVL_SETPOINT,
                              LOWER_TOR_SETPOINT, LOWER_TCL_SETPOINT};
  float upper_setpoints[4] = {UPPER_POS_SETPOINT, UPPER_LVL_SETPOINT,
                              UPPER_TOR_SETPOINT, UPPER_TCL_SETPOINT};

  struct gPID gpid;
  struct Motor motor;
  float motor_speed_norm;
  struct PWM pwm;
  // float mot_current_amp = 0.00f;
  // float mot_speed_rpm = 0.00f;
  // int16_t temperature_board_digital = 0;

  int16_t rx_can_packet[3] = {0};
  int16_t pid_can_packet[NUM_WORDS_2BYTES_TX] = {0};
  int16_t joint_can_packet[NUM_WORDS_2BYTES_TX] = {0};
  int16_t sensor_can_packet[NUM_WORDS_2BYTES_TX] = {0};
  // int16_t torque_can_packet[3] = {0};

  uint16_t counter_event = 0;
  int16_t previous_event = 0;
  int16_t event_flag = 0;

  uint16_t init_counter = 0; // GAP: counter for the delay of the initialization
                             // of hard-coded controllers
  uint16_t hard_coded_event_counter =
      0; // GAP: controller for the hard-coded sending of the event

  float counter_ramp = 0.0f;

  float control_output_aux = 0;
  float control_output_sign_aux = 0;

  int16_t echo_event_message[NUM_WORDS_2BYTES_TX] = {0};

  // Disable interruption
  dis_int();

  // I/O port config
  TRISB = 0b1111111011111111;
  TRISC = 0b1111111111111111;
  TRISD = 0b1111111111110000;
  TRISE = 0b1111111011000000;
  TRISF = 0b1111111110011111;

  // Disable watchdog timer
  //_FWDT(WDT_OFF);

  // Periperal init setup
  // adc_init();
  can_init();
  can_mask(CAN1_BUFFER0_MASK);
  can_filter(CAN1_BUFFER0_FILTER);
  can_start();

  spi_init();

  // timer1_init();
  timer2_init();
  // timer3_init();

  //==========================================================================
  sensors_qei_init(MAX_COUNTS_IN_INC_OPT_ENC);
  int16_t effective_max_counts_inc_opt_enc = MAXCNT + 1; /// MAXCNT + 1;
  int16_t half_effective_max_counts_inc_opt_enc =
      effective_max_counts_inc_opt_enc / 2;
  //==========================================================================

  // Enable global interrupts
  en_int();

  // Start ADC interrupt
  //_ADIE = 1;

  // Start CN and Timer interrupt
  // Enable CN interrupt
  _CNIE = 1;

  // Enable Timer 2 interrupt
  //_T1IE = 1;
  _T2IE = 1;

  _QEIIE = 1;

  // Enable Watchdog
  //_FWDT(WDT_ON);

  db_welcome_led();

  motor_init(&motor, 2, 100, 100.00f);
  pwm_init(&pwm, T_MS, MAX_DUTY_CYCLE);

  if (pid_init(&gpid, controller_id, gpid_k, lower_setpoints, upper_setpoints,
               control_output_signs)) {
    db_error(board_id, sensor_can_packet, PID_CONTROLLER_CONFIG_ERROR, &gpid,
             &motor);
    return -1;
  }

  int16_t mag_enc_ref_pos_digital = 0; // 9650;
  db_calibrate_magnetic_encoder(&mag_enc_ref_pos_digital);
  // spi_set_zero_position_absolute_encoder(ALPHA_ANGLE_SPI_ABS_ENC_CHANNEL,
  // mag_enc_ref_pos, 0);

  int16_t opt_enc_ref_pos_digital = 0;

/*
struct can_received_data
{
    struct gPID* gpid;
    float** vec_measured_output_norm;
    int16_t* rx_can_packet;
    float* precompression;
    int32_t* event;
    float* aux_var;
    int16_t *mag_enc_ref_pos_digital;
    int16_t *opt_enc_ref_pos_digital;
} crd;

crd.gpid = &gpid;
crd.*/

#if (HARD_CODED_CONTROLLER == 1)
  // GAP: added a delay of some seconds in the start and then enable the PID
  // controller with a setpoint of zero for the zero impedance controller
  __delay_ms(INIT_DELAY);
  gpid.controller_id = aux_controller_id;
  gpid.setpoint = 0;
  gpid.status = PID_ENABLED;
  ALERT_LED_OFF;
  OK_LED_ON;
  precompression = 0.2f / 1000.0f;
#endif

  while (1) {
    if (overf2_read() > PID_PERIOD) { //
      overf2_write(0);
      // restart_wdt();

      if (init_counter <= 100)
        init_counter++; // GAP: counter for the delay of the initialization
                        // (specially for hard-coded controllers)

      // motor_set_current_amp(&motor, 50.0f);
      // motor_set_speed_rpm(&motor, -10);

      fix_link_angle_digital = sensors_get_incremental_encoder_angle();
      fix_link_angle_digital -= opt_enc_ref_pos_digital;

      // From 0/4000 = 0.00 (0.00%, 0.00 degrees) to 7999/8000 = 0.9999389648
      // (99.99389648%, 359.978027328 degrees)
      // fix_link_angle must be symmetrical around zero value, so:
      // fix_link_angle_digital = 0    ==>    0/8000 = 0.00     (00.00%,  =>
      // 0.00 degrees)            ==> +0    (+0,           => +0 degrees)
      // ...
      // fix_link_angle_digital = 4000 ==> 4000/8000 = 0.500    (50.00%,  =>
      // 180.00 degrees)          ==> +4000 (+0.50,        => +180.00 degrees)
      // fix_link_angle_digital = 4001 ==> 4001/8000 = 0.500125 (50.0125% =>
      // 180.02197265625 degrees) ==> -3999 (-0.499875 => -179.97802734375
      // degrees)
      // ...
      // fix_link_angle_digital = 7999 ==> 7999/8000 = 0.999875 (99.9875% =>
      // 359.97802734375 degrees) ==> -1    (-0.000125 => -0.02197265625
      // degrees)

      // fix_link_angle_digital from (-3999, -1) U (0, 4000) ==> (-0.499875 ,
      // -0.000125) U (0.00, 0.50) ==> (-179.97802734375 degrees, +180.00
      // degrees)
      // fix_link_angle_digital resolution ==> 360.00 * (1/8000) = 0.045 degrees

      if (fix_link_angle_digital > half_effective_max_counts_inc_opt_enc) {
        fix_link_angle_digital -= effective_max_counts_inc_opt_enc;

      } else if (fix_link_angle_digital <
                 -half_effective_max_counts_inc_opt_enc) {
        fix_link_angle_digital += effective_max_counts_inc_opt_enc;
      }

      fix_link_angle_norm =
          (((float)fix_link_angle_digital) / effective_max_counts_inc_opt_enc);
      fix_link_angle_deg = fix_link_angle_norm * 360.0f;

      // Compute fix link speed
      fix_link_speed_current =
          (fix_link_angle_deg - fix_link_angle_deg_previous) *
          125.0f; // freq=125Hz
      fix_link_speed_accumulator =
          fix_link_speed_accumulator -
          fix_link_speed_history[fix_link_speed_count] + fix_link_speed_current;
      fix_link_speed_history[fix_link_speed_count] = fix_link_speed_current;
      fix_link_speed_count++;

      fix_link_speed = fix_link_speed_accumulator / fix_link_speed_count;
      fix_link_angle_deg_previous = fix_link_angle_deg;

      if (fix_link_speed_count >= SPEED_COMP_AVG_COUNT) // Circular buffer
        fix_link_speed_count = 0;

      if (spi_get_agc_mag_ang_absolute_encoder(
              ALPHA_ANGLE_SPI_ABS_ENC_CHANNEL, &agc_digital_aasae,
              &mag_digital_aasae, &alpha_angle_digital, &error_aasae)) {
        db_second_light_code();
        // db_error(board_id, sensor_can_packet,
        // LEVER_ARM_SPI_ABSOLUTE_ENCODER_ERROR, &gpid, &motor);
      }

      // From 0/16384 = 0.00 (0.00%, 0.00 degrees) to 16383/16384 = 0.9999389648
      // (99.99389648%, 359.978027328 degrees)
      // alpha_angle must be symmetrical around zero value, so:
      // alpha_angle_digital = 0     ==>     0/16384 = 0.00         (00.00%,
      // => 0.00 degrees)            ==> +0    (+0,           => +0 degrees)
      // ...
      // alpha_angle_digital = 8192  ==>  8192/16384 = 0.500        (50.00%,
      // => 180.00 degrees)          ==> +8192 (+0.50,        => +180.00
      // degrees)
      // alpha_angle_digital = 8193  ==>  8193/16384 = 0.5000610352
      // (50.00610352% => 180.02197265625 degrees) ==> -8191 (-0.4999389648 =>
      // -179.97802734375 degrees)
      // ...
      // alpha_angle_digital = 16383 ==> 16383/16384 = 0.9999389648
      // (99.99389648% => 359.97802734375 degrees) ==> -1    (-0.0000610351 =>
      // -0.02197265625 degrees)

      // alpha_angle_digital from (-8191, -1) U (0, 8192) ==> (-0.4999389648,
      // -0.0000610351) U (0.00, 0.50) ==> (-179.97802734375 degrees, +180.00
      // degrees)
      // alpha_angle_digital resolution ==> 360.00 * (1/16384) = 0.02197265625
      // degrees

      if (alpha_angle_digital > half_effective_max_counts_spi_abs_enc)
        alpha_angle_digital -= effective_max_counts_spi_abs_enc;

      // AJUSTAR EL SIGNO DEL ALPHA ANGLE SI PROCEDE.
      alpha_angle_digital = ALPHA_SIGN * alpha_angle_digital;

      alpha_angle_digital = exponential_weighted_moving_average(
          alpha_angle_digital_backup, alpha_angle_digital, 4);
      alpha_angle_digital_backup = alpha_angle_digital;

      alpha_angle_norm =
          (((float)alpha_angle_digital) / effective_max_counts_spi_abs_enc);
      alpha_angle_deg = alpha_angle_norm * 360.0f;
      alpha_angle_rad = alpha_angle_norm * 6.28318530717959f;

      alpha_speed_current =
          (alpha_angle_deg - alpha_angle_deg_previous) * 125.0f; // freq=125Hz
      alpha_speed_accumulator = alpha_speed_accumulator -
                                alpha_speed_history[alpha_speed_count] +
                                alpha_speed_current;
      alpha_speed_history[alpha_speed_count] = alpha_speed_current;
      alpha_speed_count++;

      alpha_speed = alpha_speed_accumulator / alpha_speed_count;
      alpha_angle_deg_previous = alpha_angle_deg;

      if (alpha_speed_count >= SPEED_COMP_AVG_COUNT) // Circular buffer
        alpha_speed_count = 0;

      // GAP: computation of absolute position of lever arm
      lever_arm_angle_norm = fix_link_angle_norm + alpha_angle_norm;

      if (lever_arm_angle_norm > 0.5f)
        lever_arm_angle_norm -= 1.0f;
      else if (lever_arm_angle_norm < -0.5f)
        lever_arm_angle_norm += 1.0f;

      lever_arm_angle_digital =
          lever_arm_angle_norm * effective_max_counts_inc_opt_enc;
      lever_arm_angle_deg = lever_arm_angle_norm * 360.0f;

      // Compute lever arm speed
      lever_arm_speed_current =
          (lever_arm_angle_deg - lever_arm_angle_deg_previous) *
          125.0f; // freq=125Hz??
      lever_arm_speed_accumulator =
          lever_arm_speed_accumulator -
          lever_arm_speed_history[lever_arm_speed_count] +
          lever_arm_speed_current;
      lever_arm_speed_history[lever_arm_speed_count] = lever_arm_speed_current;
      lever_arm_speed_count++;

      lever_arm_speed = lever_arm_speed_accumulator / lever_arm_speed_count;
      lever_arm_angle_deg_previous = lever_arm_angle_deg;

      if (lever_arm_speed_count >= SPEED_COMP_AVG_COUNT) // Circular buffer
        lever_arm_speed_count = 0;

      // ====================================================================
      //                    TORQUE BASED ON ALPHA ANGLE (BIOMOT)
      // ====================================================================
      internal_load_cell_force_N =
          1470.9975f * (((float)adc_read(INTERNAL_LOAD_CELL_CHANNEL)) / 1023);
      // sensors_get_theoretical_torque_newtons(&alpha_angle_rad,
      // &precompression, &internal_load_cell_force_N, &alpha_torque_Nm,
      // &internal_load_cell_torque_Nm);
      sensors_get_theoretical_torque_newtons(
          &alpha_angle_rad, &precompression_, &internal_load_cell_force_N,
          &alpha_torque_Nm, &internal_load_cell_torque_Nm);

      if (fabs(alpha_torque_Nm) > MAX_TOR_NM)
        alpha_torque_Nm = (alpha_torque_Nm > 0 ? MAX_TOR_NM : -MAX_TOR_NM);

      alpha_torque_norm = alpha_torque_Nm / MAX_TOR_NM;

      // internal_load_cell_torque_norm = internal_load_cell_torque_Nm /
      // MAX_TOR_NM;
      torque_norm = alpha_torque_norm;
      // ====================================================================

      motor_speed_norm = motor_get_speed_normalized(&motor);

      vec_measured_output_norm[0] = &lever_arm_angle_norm;
      vec_measured_output_norm[1] = &motor_speed_norm;
      vec_measured_output_norm[2] = &torque_norm;

#if (HARD_CODED_EVENT == 1)
      hard_coded_event_counter++;
      if (hard_coded_event_counter > 1000) {
        hard_coded_event_counter = 0;
        event = 1;
        // ALERT_LED_ON;
      } else {
        event = 0;
        // ALERT_LED_OFF;
      }
#endif

      if (db_receive_can_data(board_id, &gpid, vec_measured_output_norm,
                              rx_can_packet, &precompression, &event,
                              aux_variable, &mag_enc_ref_pos_digital,
                              &opt_enc_ref_pos_digital)) {
        db_third_light_code();
        // db_error(board_id, sensor_can_packet, PID_CONTROLLER_CONFIG_ERROR,
        // &gpid, &motor);
        // return -1;
      }

      if (aux_variable[0] == 1) {
        ALERT_LED_ON;
      }
      if (aux_variable[0] == 2) {
        ALERT_LED_OFF;
      }

#if (ECHO_EVENT == 1)
      // GAP: send a packet to see the delay between sending a receiving an
      // event from Simulink
      if (event_flag == 1) {
        echo_event_message[0] = 1;
        can_send(ECHO_ID, echo_event_message);
        __delay_us(200);
      } else if (event_flag == 0) {
        echo_event_message[0] = 0;
        can_send(ECHO_ID, echo_event_message);
        __delay_us(200);
      }
#endif

      external_load_cell_force_digital = exponential_weighted_moving_average(
          external_load_cell_force_digital_backup,
          adc_read(FIRST_EXTRA_SENSOR_CHANNEL), 1);
      external_load_cell_force_digital_backup =
          external_load_cell_force_digital;

#if (PO_CONTROLLER_MODE == NORMAL_BOOST)
      gpid.controller_id = aux_controller_id;
      if (event_flag == 0 && event == 1 && previous_event == 0) {
        event_flag = 1;
        counter_event = 0;
      }

      if (event_flag == 1 && counter_event < 80) // GAP: I have added a timeout
                                                 // for the event of 18 at 125
                                                 // hz --> 144ms
      {
        counter_event++;

        ALERT_LED_ON;

        if (precompression * 1000.0f > 0.09f) {
          gpid.control_ouput_signs[gpid.controller_id] = 1;
          gpid.control_output = 1000 * precompression;
        }
      } else {
        counter_event = 0;
        event_flag = 0;

        ALERT_LED_OFF;

// GAP: controller type selection
#if (TL_CONTROL_TYPE == NO_TL)
        pid_run(&gpid, db_select_measured_output_norm(
                           &gpid, vec_measured_output_norm));
#elif (TL_CONTROL_TYPE == TL_ALPHA_ANGLE_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   alpha_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_ALPHA_SPEED_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   alpha_speed, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_ANGLE_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   fix_link_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_SPEED_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   fix_link_speed, precompression);
#endif
      }

#elif (PO_CONTROLLER_MODE == NORMAL_BOOST_AND_DZ)
      gpid.controller_id = aux_controller_id;
      if (event_flag == 0 && event == 1 && previous_event == 0) {
        event_flag = 1;
        counter_event = 0;
      }

      if (event_flag == 1 && counter_event < 40) // GAP: this is for the DZ
      {
        counter_event++;

        ALERT_LED_ON;

        if (precompression * 1000.0f > 0.09f) {
          gpid.control_ouput_signs[gpid.controller_id] = 1;
          gpid.control_output = 1000 * precompression;
        }

      } // GAP: DZ
      else if (event_flag == 1 && counter_event < 80) {
        counter_event++;
        gpid.control_output = 0; // dead zone to absorb the change from one
                                 // controller to the other
      } else {
        counter_event = 0;
        event_flag = 0;

        ALERT_LED_OFF;

// GAP: controller type selection
#if (TL_CONTROL_TYPE == NO_TL)
        pid_run(&gpid, db_select_measured_output_norm(
                           &gpid, vec_measured_output_norm));
#elif (TL_CONTROL_TYPE == TL_ALPHA_ANGLE_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   alpha_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_ALPHA_SPEED_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   alpha_speed, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_ANGLE_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   fix_link_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_SPEED_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   fix_link_speed, precompression);
#endif
      }

#elif (PO_CONTROLLER_MODE == NORMAL_BOOST_AND_TRANSITION)
      gpid.controller_id = aux_controller_id;
      if (event_flag == 0 && event == 1 && previous_event == 0) {
        event_flag = 1;
        counter_event = 0;
      }

      if (event_flag == 1 && counter_event < 40) // GAP: this is for the DZ
      {
        counter_event++;

        ALERT_LED_ON;

        if (precompression * 1000.0f > 0.09f) {
          gpid.control_ouput_signs[gpid.controller_id] = 1;
          gpid.control_output = 1000 * precompression;
        }
        counter_ramp = 0;

      } // GAP: ramp
      else if (event_flag == 1 && counter_event >= 40) {
        if (counter_ramp < 1.0f) {
          counter_ramp = counter_ramp + 0.01f;

#if (TL_CONTROL_TYPE == NO_TL)
          pid_run(&gpid, db_select_measured_output_norm(
                             &gpid, vec_measured_output_norm));
#elif (TL_CONTROL_TYPE == TL_ALPHA_ANGLE_CTRL)
          pid_TL_run(&gpid, db_select_measured_output_norm(
                                &gpid, vec_measured_output_norm),
                     alpha_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_ALPHA_SPEED_CTRL)
          pid_TL_run(&gpid, db_select_measured_output_norm(
                                &gpid, vec_measured_output_norm),
                     alpha_speed, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_ANGLE_CTRL)
          pid_TL_run(&gpid, db_select_measured_output_norm(
                                &gpid, vec_measured_output_norm),
                     fix_link_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_SPEED_CTRL)
          pid_TL_run(&gpid, db_select_measured_output_norm(
                                &gpid, vec_measured_output_norm),
                     fix_link_speed, precompression);
#endif

          if (precompression * 1000.0f > 0.09f) {
            gpid.control_output =
                gpid.control_output * counter_ramp +
                1 * 1000 * precompression * (1.0f - counter_ramp);
          }
        } else
          event_flag = 0;
      } else {
        // counter_event = 0;

        ALERT_LED_OFF;

// GAP: controller type selection
#if (TL_CONTROL_TYPE == NO_TL)
        pid_run(&gpid, db_select_measured_output_norm(
                           &gpid, vec_measured_output_norm));
#elif (TL_CONTROL_TYPE == TL_ALPHA_ANGLE_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   alpha_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_ALPHA_SPEED_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   alpha_speed, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_ANGLE_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   fix_link_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_SPEED_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   fix_link_speed, precompression);
#endif
      }

#elif (PO_CONTROLLER_MODE == TRANSITION)
      gpid.controller_id = aux_controller_id;
      if (event_flag == 0 && event == 1 && previous_event == 0) {
        event_flag = 1;
        CAN_LED_ON;
      }

      if (event_flag == 1 && counter_event < 40) {
        counter_event++;

        ALERT_LED_ON;

        if (precompression * 1000.0f > 0.09f) {
          gpid.control_ouput_signs[gpid.controller_id] = 1;
          gpid.control_output = 1000 * precompression;
          // db_apply_control_action(&gpid, &motor, &pwm);
        }
        counter_ramp = 0;
      } else if (event_flag == 1 && counter_event >= 40) {
        if (counter_ramp < 1.0f) {
          counter_ramp = counter_ramp + 0.005f;

#if (TL_CONTROL_TYPE == NO_TL)
          pid_run(&gpid, db_select_measured_output_norm(
                             &gpid, vec_measured_output_norm));
#elif (TL_CONTROL_TYPE == TL_ALPHA_ANGLE_CTRL)
          pid_TL_run(&gpid, db_select_measured_output_norm(
                                &gpid, vec_measured_output_norm),
                     alpha_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_ALPHA_SPEED_CTRL)
          pid_TL_run(&gpid, db_select_measured_output_norm(
                                &gpid, vec_measured_output_norm),
                     alpha_speed, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_ANGLE_CTRL)
          pid_TL_run(&gpid, db_select_measured_output_norm(
                                &gpid, vec_measured_output_norm),
                     fix_link_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_SPEED_CTRL)
          pid_TL_run(&gpid, db_select_measured_output_norm(
                                &gpid, vec_measured_output_norm),
                     fix_link_speed, precompression);
#endif

          if (precompression * 1000.0f > 0.09f) {
            gpid.control_output =
                gpid.control_output * counter_ramp +
                1 * 1000 * precompression * (1.0f - counter_ramp);
          }
        } else
          event_flag = 0;
      } else {
        // counter_ramp = 1.0f;
        counter_event = 0;
        ALERT_LED_OFF;

#if (TL_CONTROL_TYPE == NO_TL)
        pid_run(&gpid, db_select_measured_output_norm(
                           &gpid, vec_measured_output_norm));
#elif (TL_CONTROL_TYPE == TL_ALPHA_ANGLE_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   alpha_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_ALPHA_SPEED_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   alpha_speed, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_ANGLE_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   fix_link_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_SPEED_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   fix_link_speed, precompression);
#endif
      }

#elif (PO_CONTROLLER_MODE == SPRING_LOCK)
      if (event_flag == 0 && event == 1 && previous_event == 0) {
        event_flag = 1;
        counter_event = 0;
      }
      // event_flag=0;
      if (event_flag == 1 && counter_event < 100) // GAP: I have added a timeout
                                                  // for the event of 18 at 125
                                                  // hz --> 144ms
      {
        counter_event++;

        CAN_LED_ON;
        ALERT_LED_ON;

        // GAP: for the spring lock, fully change the approach to that of the SW
        // stop while the counter is active
        gpid.controller_id = 0; // position controller
        // lever_arm_angle_norm_aux = SPRING_LOCK_ANGLE;
        // vec_measured_output_norm[0] = &lever_arm_angle_norm_aux;
        gpid.setpoint = SPRING_LOCK_ANGLE;
        // gpid.status = PID_ENABLED;
        // pid_run(&gpid, db_select_measured_output_norm(&gpid,
        // vec_measured_output_norm));
      } else // if (counter_event>=100)
      {
        // if (flag_controller_id==0)
        gpid.controller_id = aux_controller_id;
        counter_event = 0;
        event_flag = 0;
        CAN_LED_OFF;
        ALERT_LED_OFF;
      }
#if (TL_CONTROL_TYPE == NO_TL)
      pid_run(&gpid,
              db_select_measured_output_norm(&gpid, vec_measured_output_norm));
#elif (TL_CONTROL_TYPE == TL_ALPHA_ANGLE_CTRL)
      pid_TL_run(&gpid, db_select_measured_output_norm(
                            &gpid, vec_measured_output_norm),
                 alpha_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_ALPHA_SPEED_CTRL)
      pid_TL_run(&gpid, db_select_measured_output_norm(
                            &gpid, vec_measured_output_norm),
                 alpha_speed, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_ANGLE_CTRL)
      pid_TL_run(&gpid, db_select_measured_output_norm(
                            &gpid, vec_measured_output_norm),
                 fix_link_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_SPEED_CTRL)
      pid_TL_run(&gpid, db_select_measured_output_norm(
                            &gpid, vec_measured_output_norm),
                 fix_link_speed, precompression);
#endif

#elif (PO_CONTROLLER_MODE == POSITIONAL_BOOST)
      gpid.controller_id = aux_controller_id;
      if (event_flag == 0 && event == 1 && previous_event == 0) {
        event_flag = 1;
        counter_event = 0;
      }

      if (event_flag == 1 && counter_event < 60) // GAP: this is for the DZ
      {
        counter_event++;

        ALERT_LED_ON;

        if (precompression * 1000.0f > 0.09f) {
          gpid.control_output = 0;
          db_apply_control_action(&gpid, &motor, &pwm);
          lever_arm_angle_norm_aux = -(MIN_LIMIT_ANGLE + SW_HARD_STOP);
          vec_measured_output_norm[0] = &lever_arm_angle_norm_aux;
          gpid.controller_id = 0; // position controller
          pid_run(&gpid, db_select_measured_output_norm(
                             &gpid, vec_measured_output_norm));
          gpid.control_output = -1000 * precompression;
        }
        counter_ramp = 0;
        // aux_control_output=-gpid.control_output;

      } /*
           //GAP: ramp
           else if(event_flag == 1 && counter_event >=40)
           {
               gpid.controller_id=aux_controller_id;
               if (counter_ramp < 1.0f) {
                   counter_ramp=counter_ramp+0.01f;

                   # if (TL_CONTROL_TYPE == NO_TL)
                       pid_run(&gpid, db_select_measured_output_norm(&gpid,
           vec_measured_output_norm));
                   # elif (TL_CONTROL_TYPE == TL_ALPHA_ANGLE_CTRL)
                       pid_TL_run(&gpid, db_select_measured_output_norm(&gpid,
           vec_measured_output_norm), alpha_angle_norm, precompression);
                   # elif (TL_CONTROL_TYPE == TL_ALPHA_SPEED_CTRL)
                       pid_TL_run(&gpid, db_select_measured_output_norm(&gpid,
           vec_measured_output_norm), alpha_speed, precompression);
                   # elif (TL_CONTROL_TYPE == TL_FIX_LINK_ANGLE_CTRL)
                       pid_TL_run(&gpid, db_select_measured_output_norm(&gpid,
           vec_measured_output_norm), fix_link_angle_norm, precompression);
                   # elif (TL_CONTROL_TYPE == TL_FIX_LINK_SPEED_CTRL)
                       pid_TL_run(&gpid, db_select_measured_output_norm(&gpid,
           vec_measured_output_norm), fix_link_speed, precompression);
                   # endif

                   if(precompression * 1000.0f > 0.09f){
                       gpid.control_output=gpid.control_output * counter_ramp +
           aux_control_output * (1.0f-counter_ramp);
                   }
               }
               else event_flag = 0;
           }*/
      else {
        event_flag = 0;

        gpid.controller_id = aux_controller_id;
        ALERT_LED_OFF;

// GAP: controller type selection
#if (TL_CONTROL_TYPE == NO_TL)
        pid_run(&gpid, db_select_measured_output_norm(
                           &gpid, vec_measured_output_norm));
#elif (TL_CONTROL_TYPE == TL_ALPHA_ANGLE_CTRL)
        pid_TL_run(&gpid, db_elect_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   alpha_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_ALPHA_SPEED_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   alpha_speed, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_ANGLE_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   fix_link_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_SPEED_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   fix_link_speed, precompression);
#endif
      }
#elif (PO_CONTROLLER_MODE == RAMP_BOOST)
      gpid.controller_id = aux_controller_id;
      if (event_flag == 0 && event == 1 && previous_event == 0) {
        event_flag = 1;
        counter_ramp = 0;
        // counter_event=0;
      }

      if (event == 2 && event_flag == 1) {
        event_flag = 2;
        counter_ramp = 0;
        // counter_event=0;
      }

#if (BOARD_ID == LEFT_ANKLE_ID || BOARD_ID == RIGHT_ANKLE_ID)
      if (event_flag == 1 && precompression > 0) {
        counter_ramp = counter_ramp + precompression * 1000;
        // counter_event++;

        ALERT_LED_ON;

#if (TL_CONTROL_TYPE == NO_TL)
        pid_run(&gpid, db_select_measured_output_norm(
                           &gpid, vec_measured_output_norm));
#elif (TL_CONTROL_TYPE == TL_ALPHA_ANGLE_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   alpha_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_ALPHA_SPEED_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   alpha_speed, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_ANGLE_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   fix_link_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_SPEED_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   fix_link_speed, precompression);
#endif

        gpid.control_output =
            gpid.control_output + PO_SIGN * (counter_ramp / 100);
      }
#endif
      else {
        event_flag = 0;

        ALERT_LED_OFF;

// GAP: controller type selection
#if (TL_CONTROL_TYPE == NO_TL)
        pid_run(&gpid, db_select_measured_output_norm(
                           &gpid, vec_measured_output_norm));
#elif (TL_CONTROL_TYPE == TL_ALPHA_ANGLE_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   alpha_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_ALPHA_SPEED_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   alpha_speed, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_ANGLE_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   fix_link_angle_norm, precompression);
#elif (TL_CONTROL_TYPE == TL_FIX_LINK_SPEED_CTRL)
        pid_TL_run(&gpid, db_select_measured_output_norm(
                              &gpid, vec_measured_output_norm),
                   fix_link_speed, precompression);
#endif
      }

#endif
    }
    /*if (lever_arm_angle_norm < MAX_LIMIT_ANGLE && lever_arm_angle_norm >
    MAX_LIMIT_ANGLE - 0.01f) {//0.003f) {
        //gpid.control_output=0; //dead zone to absorb the change from one
    controller to the other
        gpid.control_output = gpid.control_output * (lever_arm_angle_norm -
    MAX_LIMIT_ANGLE);
        ALERT_LED_ON;
    }
    if (lever_arm_angle_norm > MIN_LIMIT_ANGLE && lever_arm_angle_norm <
    MIN_LIMIT_ANGLE + 0.01f) {//0.003f) {
        gpid.control_output=gpid.control_output*(lever_arm_angle_norm -
    MIN_LIMIT_ANGLE);
        ALERT_LED_ON;
    }*/

    // gpid.control_ouput_signs[gpid.controller_id]=-1*gpid.control_ouput_signs[gpid.controller_id];

    if (init_counter >=
        100) // GAP: add a delay for the initiation of hard-coded controllers
    {
#if SW_STOP == 1
      // GAP:
      // Mechanical stop supervisor:
      if (lever_arm_angle_norm < MIN_LIMIT_ANGLE + SW_SOFT_STOP) {
        control_output_aux = gpid.control_output;
        gpid.control_output =
            (abs((MIN_LIMIT_ANGLE + SW_HARD_STOP) - lever_arm_angle_norm) /
             (SW_SOFT_STOP - SW_HARD_STOP)) *
            control_output_aux;
        ALERT_LED_ON;
        // pid_run(&gpid, db_select_measured_output_norm(&gpid,
        // vec_measured_output_norm));
      } else if (lever_arm_angle_norm > MAX_LIMIT_ANGLE - SW_SOFT_STOP) {
        control_output_aux = gpid.control_output;
        gpid.control_output =
            (abs((MAX_LIMIT_ANGLE - SW_HARD_STOP) - lever_arm_angle_norm) /
             (SW_SOFT_STOP - SW_HARD_STOP)) *
            control_output_aux;
        ALERT_LED_ON;
        // pid_run(&gpid, db_select_measured_output_norm(&gpid,
        // vec_measured_output_norm));
      }
#elif SW_STOP == 2
      if (lever_arm_angle_norm <
          MIN_LIMIT_ANGLE + SW_SOFT_STOP) { // plantarflexion
        control_output_aux = gpid.control_output;
        // gpid.control_output = 0;
        // db_apply_control_action(&gpid, &motor, &pwm);
        gpid.control_output =
            (abs((MIN_LIMIT_ANGLE + SW_HARD_STOP) - lever_arm_angle_norm) /
             (SW_SOFT_STOP - SW_HARD_STOP)) *
            control_output_aux;
        // ALERT_LED_ON;
        CAN_LED_OFF;
        if (lever_arm_angle_norm < MIN_LIMIT_ANGLE + SW_HARD_STOP) {
          CAN_LED_ON;
          lever_arm_angle_norm_aux = MIN_LIMIT_ANGLE + SW_HARD_STOP;
          vec_measured_output_norm[0] = &lever_arm_angle_norm_aux;
          gpid.controller_id = 0; // position controller
          pid_run(&gpid, db_select_measured_output_norm(
                             &gpid, vec_measured_output_norm));
        }
      }
/*else if(lever_arm_angle_norm > MAX_LIMIT_ANGLE - SW_SOFT_STOP &&
event_flag==0)
{ //dorsiflexion //do not limit whan applying the PO
    control_output_aux = gpid.control_output;
    gpid.control_output = 0;
    db_apply_control_action(&gpid, &motor, &pwm);
    gpid.control_output = (abs((MAX_LIMIT_ANGLE - SW_HARD_STOP) -
lever_arm_angle_norm) / (SW_SOFT_STOP - SW_HARD_STOP)) * control_output_aux;
    //ALERT_LED_ON;
    CAN_LED_OFF;
    if(lever_arm_angle_norm > MAX_LIMIT_ANGLE - SW_HARD_STOP)
    {
        CAN_LED_ON;
        lever_arm_angle_norm_aux = MAX_LIMIT_ANGLE - SW_HARD_STOP;
        vec_measured_output_norm[0] = &lever_arm_angle_norm_aux;
        gpid.controller_id = 0; //position controller
        pid_run(&gpid, db_select_measured_output_norm(&gpid,
vec_measured_output_norm));
    }
}*/
#elif SW_STOP == 3
      if (lever_arm_angle_norm < MIN_LIMIT_ANGLE + SW_HARD_STOP) {
        lever_arm_angle_norm_aux = MIN_LIMIT_ANGLE + SW_HARD_STOP;
        vec_measured_output_norm[0] = &lever_arm_angle_norm_aux;
        gpid.controller_id = 0; // position controller
        // gpid.setpoint = MIN_LIMIT_ANGLE;
        ALERT_LED_ON;
        control_output_aux = gpid.control_output;
        control_output_sign_aux =
            -1 * ALPHA_SIGN * gpid.control_ouput_signs[gpid.controller_id];
        pid_run(&gpid, db_select_measured_output_norm(
                           &gpid, vec_measured_output_norm));
        if (gpid.control_output > control_output_aux) {
          gpid.control_output =
              (abs(lever_arm_speed / 20)) * control_output_aux;
          gpid.control_ouput_signs[gpid.controller_id] =
              control_output_sign_aux;
        }
      } else if (lever_arm_angle_norm > MAX_LIMIT_ANGLE - SW_HARD_STOP) {
        lever_arm_angle_norm_aux = MAX_LIMIT_ANGLE - SW_HARD_STOP;
        vec_measured_output_norm[0] = &lever_arm_angle_norm_aux;
        gpid.controller_id = 0; // position controller
        // gpid.setpoint = MAX_LIMIT_ANGLE;
        ALERT_LED_ON;
        control_output_aux = gpid.control_output;
        control_output_sign_aux =
            -1 * ALPHA_SIGN * gpid.control_ouput_signs[gpid.controller_id];
        pid_run(&gpid, db_select_measured_output_norm(
                           &gpid, vec_measured_output_norm));
        if (gpid.control_output > control_output_aux) {
          gpid.control_output =
              (abs(lever_arm_speed / 20)) * control_output_aux;
          gpid.control_ouput_signs[gpid.controller_id] =
              control_output_sign_aux;
        }
      }
#elif SW_STOP == 4
      if (lever_arm_angle_norm < MIN_LIMIT_ANGLE + SW_SOFT_STOP) {
        control_output_aux = gpid.control_output;
        gpid.control_output =
            (abs((MIN_LIMIT_ANGLE + SW_MIDDLE_STOP) - lever_arm_angle_norm) /
             (SW_SOFT_STOP - SW_MIDDLE_STOP)) *
            control_output_aux;
        ALERT_LED_ON;
        CAN_LED_OFF;
        if (lever_arm_angle_norm <
            MIN_LIMIT_ANGLE + (SW_HARD_STOP + SW_MIDDLE_STOP)) {
          CAN_LED_ON;
          lever_arm_angle_norm_aux = MIN_LIMIT_ANGLE + SW_HARD_STOP;
          vec_measured_output_norm[0] = &lever_arm_angle_norm_aux;
          gpid.controller_id = 0; // position controller
          pid_run(&gpid, db_select_measured_output_norm(
                             &gpid, vec_measured_output_norm));
          gpid.control_output =
              (1.0f -
               (abs((MIN_LIMIT_ANGLE + SW_HARD_STOP) - lever_arm_angle_norm) /
                (SW_MIDDLE_STOP - SW_HARD_STOP))) *
              control_output_aux;
        }
      } else if (lever_arm_angle_norm > MAX_LIMIT_ANGLE - SW_SOFT_STOP) {
        control_output_aux = gpid.control_output;
        gpid.control_output =
            (abs((MAX_LIMIT_ANGLE - SW_MIDDLE_STOP) - lever_arm_angle_norm) /
             (SW_SOFT_STOP - SW_MIDDLE_STOP)) *
            control_output_aux;
        ALERT_LED_ON;
        CAN_LED_OFF;
        if (lever_arm_angle_norm > MAX_LIMIT_ANGLE - SW_HARD_STOP) {
          CAN_LED_ON;
          lever_arm_angle_norm_aux = MAX_LIMIT_ANGLE - SW_HARD_STOP;
          vec_measured_output_norm[0] = &lever_arm_angle_norm_aux;
          gpid.controller_id = 0; // position controller
          pid_run(&gpid, db_select_measured_output_norm(
                             &gpid, vec_measured_output_norm));
          gpid.control_output =
              (1.0f -
               (abs((MAX_LIMIT_ANGLE - SW_HARD_STOP) - lever_arm_angle_norm) /
                (SW_MIDDLE_STOP - SW_HARD_STOP))) *
              control_output_aux;
        }
      }
#elif SW_STOP == 5
      if (lever_arm_angle_norm < MIN_LIMIT_ANGLE + SW_HARD_STOP) {
        CAN_LED_ON;
        lever_arm_angle_norm_aux = MIN_LIMIT_ANGLE + SW_HARD_STOP;
        vec_measured_output_norm[0] = &lever_arm_angle_norm_aux;
        gpid.controller_id = 0; // position controller
        pid_run(&gpid, db_select_measured_output_norm(
                           &gpid, vec_measured_output_norm));
      }
#endif
      db_apply_control_action(&gpid, &motor, &pwm);
    }

    db_create_joint_can_packet(joint_can_packet, fix_link_angle_digital,
                               alpha_angle_digital,
                               Float2Fract(alpha_torque_norm), 0);
    db_create_pid_can_packet(pid_can_packet, &gpid);
    db_create_sensor_can_packet(sensor_can_packet, 0, 0,
                                adc_read(CURRENT_SENSOR_CHANNEL),
                                lever_arm_angle_digital);

    previous_event = event;

#if (NUM_WORDS_2BYTES_TX == 3)

    can_send(board_id + JOINT_STATE_MESSAGE_ID, joint_can_packet);
    __delay_us(200);

    can_send(board_id + PID_MESSAGE_ID, pid_can_packet);
    __delay_us(200);

    can_send(board_id + SENSORS_MESSAGE_ID, sensor_can_packet);
    __delay_us(200);
#else

    can_send(0x0013, pid_can_packet);
    __delay_us(200);

    can_send(0x0014, sensor_can_packet);
    __delay_us(200);

    can_send(0x0015, joint_can_packet);
    __delay_us(200);
#endif
  }

  // Check motor safety
  // if(bldc_check_mot_temp(temp_cel) ||
  // bldc_check_mot_overcurrent(mot_current))
  //{
  //    error = 4;
  //    db_error(board_id, &pid_pos, &motor);
  //    return -1;
  //}
  return 0;
}
