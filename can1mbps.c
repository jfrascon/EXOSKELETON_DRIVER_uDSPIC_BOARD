/*****************************************************************************
* Low level configuration of CAN1 and CAN2 for DsPIC30F Microcontrollers.    *
* Configuration was performed in microcontrollers register's level. To be    *
  able to understand and modifying it, if necessary, please refer to the CAN *
  session of the DsPIC30F Family Reference Manual:                           *
  http://ww1.microchip.com/downloads/en/DeviceDoc/70046D.pdf                 *
* CAN 1 and 2 are configured to 1 Mbps, standard ID messages and to send     *
  6 bytes each message.                                                      *
* Author: Francisco Rascón                                                   *
*****************************************************************************/

#include "constants.h"

#include "can1mbps.h"
#include <libpic30.h>
#include <xc.h>
#

//******************************************
// Subroutine of CAN Initialization        *
//******************************************

void can_init() {
  // Control Register
  C1CTRL = 0x0C00;

  // Tx register
  C1TX0SID = 0x0000;

#if (NUM_WORDS_2BYTES_TX == 3)

  C1TX0DLC =
      0x0030; // Transmits 6 bytes (3 words of 2 bytes)      00 110 000 => 30
//                                                                       (6)
#else

  C1TX0DLC = 0x0040; // Transmits 8 bytes (4 words of 2 bytes)

#endif

  // RX buffer registers
  // If RXB0 is full and another message is accepted this message is store in
  // RXB1
  // as long as this buffer is empty. If RXB1 is full the message is discarded.
  // 0x0004 (DBEN = 1) ==> OVERFLOW  RXB0 to RXB1
  // 0x0000 (DBEN = 0) ==> NO OVERFLOW
  C1RX0CON = 0x0004;

  C1RX1CON = 0x0000;

  C1RX0SID = 0x0000;
  C1RX1SID = 0x0000;

  // C1RX0DLC = 0x0000;
  // C1RX1DLC = 0x0000;

  C1RX0DLC |= 0x0006; // Cambiar por 0x0006 y ya
  C1RX1DLC |= 0x0006; // Cambiar pot 0X0006 y ya
  C1RX0DLC &= 0xFFF6;
  C1RX1DLC &= 0xFFF6;

  // Rx Filter Registers
  C1RXF0SID = 0x0000;
  C1RXF1SID = 0x0000;
  C1RXF2SID = 0x0000;
  C1RXF3SID = 0x0000;
  C1RXF4SID = 0x0000;
  C1RXF5SID = 0x0000;

  // Rx Mask Filter registers
  // C1RXM0SID = 0xFFFF;
  // C1RXM1SID = 0xFFFF;
  C1RXM0SID = 0x0001;
  C1RXM1SID = 0x0001;

  // CAN Baud Rate Registers
  // If C1CFG1:BRP<5:0> = 0000000 then
  // Baud Rate Prescaler: 0
  // Tq = 2(Baud Rate Prescaler + 1)/Fcan = 2/Fcan
  // CANCKS = 1 ==> Fcan = 1*Fcy = 16 MHz ==> Tq = 2/Fcan = 0.125 us
  // Synchronized Jump Width: 1xTq

  // Propagation Time Segment: 1xTq
  // Phase Buffer Segment 1: 3xTq
  // Sample Rate: Bus line is sampled once at the sample point.
  // Phase Buffer Segment 2: 3xTq
  // Wakeup Filter:  Off

  // Bit Time = (Synchronized Jump Width + Propagation Time Segment +
  // + Phase Buffer Segment + Phase Buffer Segment 2) = 1xTq + 1xTq + 3xTq +
  // + 3xTq = 8xTq
  // 8xTq = 8 x 0.125 us = 1 us => CANfreq = 1 Mbps
  C1CFG1 = 0x0000;
  C1CFG2 = 0x0290;

  // Interrupt Registers
  C1INTF = 0x0000;
  C1INTE = 0x0000;
}

//*************************
// Starting CAN1       ****
//*************************

void can_start() { C1CTRL = 0x0800; }

void can_startLoopBack() {
  int16_t flag = 0xFFFF;

  // Control Register
  // 1- Set normal mode operation, set to "000" bits <10:8> (REQOP field)
  // 2- Check if configuration mode was setup, read bit<7:5> (OPMODE field),
  //   must set be to "100"

  while (flag != 0x0040) {
    C1CTRL |= 0x0200;         // set to one bit 9
    C1CTRL &= 0xFAFF;         // set to zero bits <8:10>
    flag = (C1CTRL & 0x00E0); // verify if normal mode was set
  }
}

//*****************************************************************************
// CAN1 Sending                                                               *
//*****************************************************************************

void can_send(uint16_t id, int16_t *mens) {
  C1TX0B1 = mens[0];
  C1TX0B2 = mens[1];
  C1TX0B3 = mens[2];

  id &= 0x07FF; // identifier 11 bits
  id <<= 5;
  int16_t tx0sid = id;
  tx0sid &= 0xF800;
  id <<= 5;
  id >>= 8;
  tx0sid |= id;

  C1TX0SID = tx0sid;

  C1TX0CONbits.TXREQ = 1;
}

//*****************************************************************************
// CAN1 Receiving                                                             *
//*****************************************************************************

int32_t can_receive(int16_t *msg) {
  static int8_t read_buffer_1 = 0;
  int32_t id = -1; // NO CAMBIAR A INT16_T NI A UINT16_T

  if (read_buffer_1) {
    msg[0] = C1RX1B1;
    msg[1] = C1RX1B2;
    msg[2] = C1RX1B3;

    id = C1RX1SID;
    id <<= 3;
    id >>= 5;

    read_buffer_1 = 0;

    C1RX1CONbits.RXFUL = 0;
  }

  else if (C1RX0CONbits.RXFUL) {
    msg[0] = C1RX0B1;
    msg[1] = C1RX0B2;
    msg[2] = C1RX0B3;

    id = C1RX0SID;
    id <<= 3;
    id >>= 5;

    if (C1RX1CONbits.RXFUL)
      read_buffer_1 = 1;

    C1RX0CONbits.RXFUL = 0;
  }

  return id;
}

//*****************************************************************************
// CAN1 Filter                                                                *
//*****************************************************************************

void can_filter(int16_t can1_filter_11b) {
  // Only RXF0 active
  can1_filter_11b <<= 2;

  C1RXF0SID = can1_filter_11b;
  C1RXF1SID = can1_filter_11b;

  C1RXF2SID = 0xFFFF;
  C1RXF3SID = 0xFFFF;
  C1RXF4SID = 0xFFFF;
  C1RXF5SID = 0xFFFF;
}

//*****************************************************************************
// CAN1 Filter Mask                                                           *
//*****************************************************************************

void can_mask(int16_t can1_mask_11b) {
  // Only RXM0 active
  can1_mask_11b <<= 2;

  C1RXM0SID = can1_mask_11b;
  C1RXM0SIDbits.MIDE = 1;

  // Solo quiero que el buffer1 reciba datos por desbordamiento del buffer0.
  // No quiero que pueda recibir datos de otro modo.
  // Usar una combinaciÛn de m·scara 0xFFFF y filtros de 2 a 5 = 0x0000
  C1RXM1SID = 0xFFFF;
  C1RXM1SIDbits.MIDE = 1;
}
