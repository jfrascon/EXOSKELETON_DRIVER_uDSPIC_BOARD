/*******************************************************************************
* File:   defines.h
* Author: Francisco Rascón

* Define the Data Types, ports definitions, etc. used in the main program.
********************************************************************************/

#ifndef MAIN_H
#define MAIN_H

#include "pid.h"

// FCY = 16 MHz
#define FCY 16000000UL

// General interruption
#define dis_int() __builtin_disi(0x3FFF)
#define en_int() __builtin_disi(0x0000)

#define HARD_CODED_CONTROLLER                                                  \
  0 // 0- just attend to external controller; 1- start with a hard-coded
    // controller
#define HARD_CODED_EVENT                                                       \
  0 // 0- just wait for external event; 1- periodically send internal event

#define INIT_DELAY 5000 // hard-coded initialization delay in ms

#define SPRING_LOCK_ANGLE 0.00f; // Position for locking the spring to load it

#define NORMAL_BOOST 0
#define NORMAL_BOOST_AND_DZ 1
#define NORMAL_BOOST_AND_TRANSITION 2
#define TRANSITION 3
#define SPRING_LOCK 4
#define POSITIONAL_BOOST 5
#define RAMP_BOOST 6

#define PO_CONTROLLER_MODE                                                     \
  RAMP_BOOST // NORMAL_BOOST NORMAL_BOOST_AND_DZ NORMAL_BOOST_AND_TRANSITION
             // TRANSITION SPRING_LOCK POSITIONAL_BOOST RAMP_BOOST

#define ECHO_EVENT 0  // 0 - do not send echo; 1 - send echo
#define ECHO_ID 0x200 // ID for the echo of the event

#define SW_STOP                                                                \
  5 // 0 - not active; 1 - ramp down; 2 - ramp down + positional stop; 3 - speed
    // dependent stop; 4 - ramp down + ramp up of positional stop; 5 -
    // positional stop
#define SW_SOFT_STOP                                                           \
  0.011f // 4 degrees // this is the difference of degrees to the mech limit
         // value from where the actuator starts to drop the actuation when
         // approaching the mech limit
#define SW_HARD_STOP                                                           \
  0.003f // 1 degree // this is the difference of degrees to the mech limit
         // value from where the actuator must never pass actuated
#define SW_MIDDLE_STOP                                                         \
  ((SW_SOFT_STOP - SW_HARD_STOP) / 2) +                                        \
      SW_HARD_STOP // 2.5 degrees // this is the difference of degrees to the
                   // mech limit value from where the ramp up positional stop
                   // starts its actuation

// Constant definitions
#define PID_PERIOD 3

#define ALERT_LED_OFF (_LATD0 = 0b0)
#define ALERT_LED_ON (_LATD0 = 0b1)
#define OK_LED_OFF (_LATB8 = 0b0)
#define OK_LED_ON (_LATB8 = 0b1)
#define CAN_LED_OFF (_LATF5 = 0b0)
#define CAN_LED_ON (_LATF5 = 0b1)

#define ESCON_CW _LATE3
#define ESCON_CCW _LATE4
#define ESCON_MOT_ENABLE _LATE5

#define K_MIN -100.00f
#define K_MAX +100.00f

#define TWO_POW_14 16384.0f // 14 bits
#define TWO_POW_10 1024.0f  // 10 bits
#define MAX_FORCE_LOAD_CELL 0.0f

#define MAX_POS_DEG 360.00f
#define UPPER_POS_SETPOINT (+0.50f)
#define LOWER_POS_SETPOINT (-0.50f)
#define DEFAULT_POS_KP 3.0f // 0.50f//0.89f
#define DEFAULT_POS_KI 0.0005f
#define DEFAULT_POS_KD 0.00f // 0.07f

#define MAX_LVL_DEG 360.0f
#define UPPER_LVL_SETPOINT (+0.50f)
#define LOWER_LVL_SETPOINT (-0.50f)
#define DEFAULT_LVL_KP 0.0f
#define DEFAULT_LVL_KI 0.0f
#define DEFAULT_LVL_KD 0.0f

// Torque express in Nm
#define MAX_TOR_NM 25.00f // Associated with alpha angle of
#define UPPER_TOR_SETPOINT (+1.00f)
#define LOWER_TOR_SETPOINT (-1.00f)
#if (HARD_CODED_CONTROLLER == 1)
#define DEFAULT_TOR_KP 3.5f // GAP: start with the zero impedance controller
#else
#define DEFAULT_TOR_KP 0.6f
#endif
#define DEFAULT_TOR_KI 0.000f
#define DEFAULT_TOR_KD 0.0f

#define MAX_TCL 0.00f // Associated with alpha angle of
#define UPPER_TCL_SETPOINT (+1.00f)
#define LOWER_TCL_SETPOINT (-1.00f)
#define DEFAULT_TCL_KP 0.0f
#define DEFAULT_TCL_KI 0.0f
#define DEFAULT_TCL_KD 0.0f

#define T_MS 0.25f
#define MAX_DUTY_CYCLE 0.90f
#define MAX_COUNTS_IN_INC_OPT_ENC                                              \
  2000 // 2000 counts with associated index from 0 to 1999

//#define ROUND(X) (X<0.00f?((int16_t)(X-0.50f)):((int16_t)(X+0.50f)))

#define B 0.0467f  // m Length of the lever arm
#define D 0.0560f  // m Length of the fix link
#define K 68600.0f // N/m stiffness of the MACCEPA spring

#define QUADRATURE_MODULE_ENABLED 1

enum error_code {
  PID_CONTROLLER_CONFIG_ERROR = 1,
  FIX_LINK_SPI_ABSOLUTE_ENCODER_ERROR = 2,
  LEVER_ARM_SPI_ABSOLUTE_ENCODER_ERROR = 3
};

#define LEFT_HIP_ID 32
#define LEFT_KNEE_ID 64
#define LEFT_ANKLE_ID 96
#define RIGHT_HIP_ID 128
#define RIGHT_KNEE_ID 160
#define RIGHT_ANKLE_ID 192

#define KP_MESSAGE_ID 1             // 10
#define KI_MESSAGE_ID 2             // 20
#define KD_MESSAGE_ID 3             // 30
#define SETPOINT_MESSAGE_ID 4       // 40
#define PRECOMPRESSION_MESSAGE_ID 5 // 80
#define EVENT_MESSAGE_ID 6          // 512
#define REF_ALPHA_ANGLE_MESSAGE_ID 7
#define REF_FIX_LINK_ANGLE_MESSAGE_ID 8
#define AUX_VAR_MESSAGE_ID 9
#define ENCODERS_CALIBRATION_MESSAGE_ID 10
#define PID_MESSAGE_ID 29         // 70
#define JOINT_STATE_MESSAGE_ID 30 // 60
#define SENSORS_MESSAGE_ID 31     // 50

#define NO_TL 0
#define TL_ALPHA_ANGLE_CTRL 1
#define TL_ALPHA_SPEED_CTRL 2
#define TL_FIX_LINK_ANGLE_CTRL 3
#define TL_FIX_LINK_SPEED_CTRL 4

#define NUM_WORDS_2BYTES_TX 3 // 4

// LEFT_HIP_ID, LEFT_KNEE_ID, LEFT_ANKLE_ID, RIGHT_HIP_ID, RIGHT_KNEE_ID,
// RIGHT_ANKLE_ID
#define BOARD_ID LEFT_KNEE_ID

#if BOARD_ID == LEFT_HIP_ID

#define SWAP_INC_OPT_ENC 0
#define FIX_LINK_ANGLE_SIGN 1
#define CAN1_BUFFER0_FILTER 0x020
#define MIN_LIMIT_ANGLE
#define MAX_LIMIT_ANGLE
#define ALPHA_SIGN

#elif BOARD_ID == LEFT_KNEE_ID

#define SWAP_INC_OPT_ENC 1
#define FIX_LINK_ANGLE_SIGN -1
#define CAN1_BUFFER0_FILTER 0x040
#define MIN_LIMIT_ANGLE
#define MAX_LIMIT_ANGLE
#define ALPHA_SIGN +1

#elif BOARD_ID == LEFT_ANKLE_ID

#define SWAP_INC_OPT_ENC 0
#define FIX_LINK_ANGLE_SIGN 1
#define CAN1_BUFFER0_FILTER 0x060
#define MIN_LIMIT_ANGLE -0.064f // 23.04�
#define MAX_LIMIT_ANGLE 0.036f  // 12.96�
#define ALPHA_SIGN 1
#define PO_SIGN 1

#elif BOARD_ID == RIGHT_HIP_ID

#define SWAP_INC_OPT_ENC 1
#define FIX_LINK_ANGLE_SIGN -1
#define CAN1_BUFFER0_FILTER 0x080
#define MIN_LIMIT_ANGLE
#define MAX_LIMIT_ANGLE
#define ALPHA_SIGN

#elif BOARD_ID == RIGHT_KNEE_ID

#define SWAP_INC_OPT_ENC 0
#define FIX_LINK_ANGLE_SIGN -1
#define CAN1_BUFFER0_FILTER 0x0A0
#define MIN_LIMIT_ANGLE
#define MAX_LIMIT_ANGLE
#define ALPHA_SIGN +1

#elif BOARD_ID == RIGHT_ANKLE_ID

#define SWAP_INC_OPT_ENC 1
#define FIX_LINK_ANGLE_SIGN -1
#define CAN1_BUFFER0_FILTER 0x0C0
#define MIN_LIMIT_ANGLE -0.065f // 23.4�
#define MAX_LIMIT_ANGLE 0.03f   // 10.8�
#define ALPHA_SIGN 1
#define PO_SIGN -1

#endif

#define CAN1_BUFFER0_MASK 0xFF0 // 0xFE0

#if (HARD_CODED_CONTROLLER == 1)
#define PID_CONTROLLER_ID                                                      \
  TOR_PID_CONTROLLER_ID // GAP: start with the zero impedance controller
#else
#define PID_CONTROLLER_ID POS_PID_CONTROLLER_ID
#endif
#define SPEED_COMP_AVG_COUNT 10
#define TL_CONTROL_TYPE                                                        \
  NO_TL // NO_TL, TL_ALPHA_ANGLE_CTRL, TL_ALPHA_SPEED_CTRL,
        // TL_FIX_LINK_ANGLE_CTRL, TL_FIX_LINK_SPEED_CTRL

#endif
