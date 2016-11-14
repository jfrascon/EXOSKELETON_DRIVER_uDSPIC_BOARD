/*****************************************************************************
* Low level configuration of CAN1 and CAN2 for DsPIC30F Microcontrollers     *
* CAN 1 and 2 are configured to 1 Mbps, standard ID messages and to send     *
  6 bytes each message.                                                      *
                                                                             *
* Author: Francisco Rascón                                                   *
******************************************************************************/

#ifndef CAN1MBPS_H
#define CAN1MBPS_H

#include <stdint.h>

//#define  TXREQ (C1TX0CONbits.TXREQ)
//#define  RXFUL  (C1RX0CONbits.RXFUL)
//#define  MIDE  (C1RXM0SIDbits.MIDE)
//#define  RX0IF (C1INTFbits.RX0IF)
//#define  RX0IE (C1INTEbits.RX0IE)
//#define  RXFUL1 (C1RX1CONbits.7)

void can_filter(int16_t can1_filter_11b);
void can_init();
void can_mask(int16_t can1_mask_11b);
int32_t can_receive(int16_t *msg);
void can_send(uint16_t id, int16_t *mens);
void can_start();
void can_startLoopBack();

#endif
