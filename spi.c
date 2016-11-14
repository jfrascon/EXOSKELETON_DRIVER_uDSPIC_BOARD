/*******************************************************************************
* File: spi.c
* Version: 0.1
* Author: Francisco Rascón

* Driver for the SPI communication of the H2R robot. Contains all functions and
 definitions related to SPI communication.
*******************************************************************************/

#include "spi.h"
#include <xc.h>

static int16_t spi_calc_even_parity(int32_t data);
static void spi_enable_port(int16_t channel);
static void spi_disable_port(int16_t channel);
static void spi_rcv_msg(int16_t *data);
static void spi_snd_msg(int16_t *data);
static void spi_tx_rx_sequence_absolute_encoder(int16_t channel, int16_t data,
                                                int16_t *response);

static int16_t spi_calc_even_parity(int32_t data) {
  int counter = 0;

  while (data) {
    if (data & 0x1)
      ++counter;
    data >>= 1;
  }

  // counter with even number ends in 0 ==> return 0
  // counter with ood  number ends in 1 ==> return 1
  return (counter & 0x1);
}

void spi_init() {
  /* Set E0 as output */
  _TRISB2 = 0;
  /* Set E1, E2, and E3 as output */
  _TRISD1 = 0;
  _TRISD2 = 0;
  _TRISD3 = 0;
  /* Set E4 as output */
  _TRISE8 = 0;

  /* Set SDI as input */
  // TRISF |= 0x0004;
  /* Set SD0 as output */
  // TRISF &= 0xFFF7;

  /* Set E0 to High */
  _LATB2 = 1;
  /* Set E1, E2, and E3 to High */
  _LATD1 = 1;
  _LATD2 = 1;
  _LATD3 = 0;
  /* Set E4 to High */
  _LATE8 = 0;

  _PPRE = 0b10;  // Primary   Prescale (Master Mode) FCY/4 = 4 MHz (250 ns)
  _SPRE = 0b000; // Secondary Prescale (Master Mode) FCY/8 = 2 MHz (125 ns)
  _MSTEN = 0b1;  // Master mode enabled
  _CKP = 0b0;    // Clock idle is low
  _SSEN = 0b0;
  _CKE = 0b0;    // Data changes on transition from Idle to active clk state
  _SMP = 0b0;    // SPI Data Input Sample Phase bit
  _MODE16 = 0b1; // 16 bit data transmition
  _DISSDO = 0b0; // Disable SDOx pin bit
  _SPIFSD = 0b0; // Frame sync pulse output (master)
  _FRMEN = 0b0;  // Framed SPI support enabled

  _SPIEN = 0b1;   // Enable SPI module
  _SPISIDL = 0b0; // Continue module operation in idle mode
  _SPIROV = 0b0;  // Receive Overflow Flag bit
}

static void spi_rcv_msg(int16_t *data) {
  while (!_SPIRBF)
    ;
  *data = SPI1BUF;
}

static void spi_snd_msg(int16_t *data) {
  while (_SPITBF)
    ;
  SPI1BUF = *data;
}

static void spi_enable_port(int16_t channel) {
  switch (channel) {
  case 1: {
    SPI_PORT_0_ENABLE;
    break;
  }
  case 2: {
    SPI_PORT_1_ENABLE;
    break;
  }
  case 3: {
    SPI_PORT_2_ENABLE;
    break;
  }
  case 4: {
    SPI_PORT_3_ENABLE;
    break;
  }
  case 5: {
    SPI_PORT_4_ENABLE;
    break;
  }
  }
}

static void spi_disable_port(int16_t channel) {
  switch (channel) {
  case 1: {
    SPI_PORT_0_DISABLE;
    break;
  }
  case 2: {
    SPI_PORT_1_DISABLE;
    break;
  }
  case 3: {
    SPI_PORT_2_DISABLE;
    break;
  }
  case 4: {
    SPI_PORT_3_DISABLE;
    break;
  }
  case 5: {
    SPI_PORT_4_DISABLE;
    break;
  }
  }
}

static void spi_tx_rx_sequence_absolute_encoder(int16_t channel, int16_t data,
                                                int16_t *response) {
  spi_enable_port(channel);
  data |= (spi_calc_even_parity(data) << 15);
  spi_snd_msg(&data);
  spi_rcv_msg(response);
  spi_disable_port(channel);
}

int16_t spi_get_agc_mag_ang_absolute_encoder(int16_t channel,
                                             int16_t *spi_agc_14,
                                             int16_t *spi_mag_14,
                                             int16_t *spi_ang_14,
                                             int16_t *spi_error) {

  int16_t result = 0;

  // Send READ AGC command. Received data is thrown away: this data comes from
  // the previous command (unknown)
  // spi_tx_rx_sequence_absolute_encoder(channel, (SPI_CMD_READ | SPI_REG_AGC),
  // spi_agc_14);
  // spi_tx_rx_sequence_absolute_encoder(channel, (SPI_CMD_READ | SPI_REG_MAG),
  // spi_agc_14);
  spi_tx_rx_sequence_absolute_encoder(channel, (SPI_CMD_READ | SPI_REG_ANG),
                                      spi_mag_14);
  spi_tx_rx_sequence_absolute_encoder(channel, SPI_CMD_NOP, spi_ang_14);

  // if((*spi_agc_14 & 0x4000) || (*spi_mag_14 & 0x4000) || (*spi_ang_14 &
  // 0x4000))
  if (*spi_ang_14 & 0x4000) {
    spi_tx_rx_sequence_absolute_encoder(
        channel, (SPI_CMD_READ | SPI_REG_CLRERR), spi_error);
    spi_tx_rx_sequence_absolute_encoder(channel, SPI_CMD_NOP, spi_error);
    *spi_error = (*spi_error) & 0x0007;
    result = -1;
  }

  *spi_agc_14 = ((*spi_agc_14) & 0x00FF);
  *spi_mag_14 = ((*spi_mag_14) & 0x3FFF);
  *spi_ang_14 = ((*spi_ang_14) & 0x3FFF);

  return result;
}

int16_t spi_set_zero_position_absolute_encoder(int16_t channel,
                                               int16_t zero_position,
                                               int16_t permanent_storage) {
  int16_t old_data = 0;
  int16_t new_data = 0;

  spi_tx_rx_sequence_absolute_encoder(
      channel, (SPI_CMD_WRITE | SPI_REG_ZEROPOS_HI), &old_data);
  spi_tx_rx_sequence_absolute_encoder(channel, 0, &old_data);
  spi_tx_rx_sequence_absolute_encoder(channel, SPI_CMD_NOP, &new_data);

  spi_tx_rx_sequence_absolute_encoder(
      channel, (SPI_CMD_WRITE | SPI_REG_ZEROPOS_LO), &old_data);
  spi_tx_rx_sequence_absolute_encoder(channel, 0, &old_data);
  spi_tx_rx_sequence_absolute_encoder(channel, SPI_CMD_NOP, &new_data);

  spi_tx_rx_sequence_absolute_encoder(
      channel, (SPI_CMD_WRITE | SPI_REG_ZEROPOS_HI), &old_data);
  spi_tx_rx_sequence_absolute_encoder(channel, ((zero_position & 0x3FC0) >> 6),
                                      &old_data);
  spi_tx_rx_sequence_absolute_encoder(channel, SPI_CMD_NOP, &new_data);

  int16_t zero_position_rx = ((new_data & 0x00FF) << 6);

  spi_tx_rx_sequence_absolute_encoder(
      channel, (SPI_CMD_WRITE | SPI_REG_ZEROPOS_LO), &old_data);
  spi_tx_rx_sequence_absolute_encoder(channel, (zero_position & 0x003F),
                                      &old_data);
  spi_tx_rx_sequence_absolute_encoder(channel, SPI_CMD_NOP, &new_data);

  zero_position_rx |= (new_data & 0x003F);

  if (permanent_storage) {
    spi_tx_rx_sequence_absolute_encoder(channel, (SPI_CMD_WRITE | SPI_REG_OTP),
                                        &old_data);
    spi_tx_rx_sequence_absolute_encoder(channel, 0x0001, &old_data);
    spi_tx_rx_sequence_absolute_encoder(channel, SPI_CMD_NOP, &new_data);

    spi_tx_rx_sequence_absolute_encoder(channel, (SPI_CMD_WRITE | SPI_REG_OTP),
                                        &old_data);
    spi_tx_rx_sequence_absolute_encoder(channel, 0x0008, &old_data);
    spi_tx_rx_sequence_absolute_encoder(channel, SPI_CMD_NOP, &new_data);

    spi_tx_rx_sequence_absolute_encoder(channel, (SPI_CMD_READ | SPI_REG_ANG),
                                        &new_data);
    spi_tx_rx_sequence_absolute_encoder(channel, SPI_CMD_NOP, &new_data);

    spi_tx_rx_sequence_absolute_encoder(channel, (SPI_CMD_WRITE | SPI_REG_OTP),
                                        &old_data);
    spi_tx_rx_sequence_absolute_encoder(channel, 0x0040, &old_data);
    spi_tx_rx_sequence_absolute_encoder(channel, SPI_CMD_NOP, &new_data);

    spi_tx_rx_sequence_absolute_encoder(channel, (SPI_CMD_READ | SPI_REG_ANG),
                                        &new_data);
    spi_tx_rx_sequence_absolute_encoder(channel, SPI_CMD_NOP, &new_data);
  }

  return zero_position_rx;
}
