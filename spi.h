/*******************************************************************************
* File: spi.c
* Version: 0.1
* Author: Francisco Rascón

* Driver for the SPI communication of the H2R robot. Contains all functions and
definitions related to SPI communication.
*******************************************************************************/

#ifndef SPI_H
#define SPI_H

#include <stdint.h>

#define SPI_CMD_READ 0x4000       // flag indicating read attempt
#define SPI_CMD_WRITE 0x0000      // flag indicating write attempt
#define SPI_CMD_NOP 0x0000        // flag indicating write attempt
#define SPI_REG_AGC 0x3FFD        // agc register when using SPI
#define SPI_REG_MAG 0x3FFE        // magnitude register when using SPI
#define SPI_REG_ANG 0x3FFF        // data register when using SPI
#define SPI_REG_CLRERR 0x0001     // clear error register when using SPI
#define SPI_REG_ZEROPOS_HI 0x0016 // zero position register high byte
#define SPI_REG_ZEROPOS_LO 0x0017 // zero position register low byte
#define SPI_REG_OTP 0x0003        // data register when using SPI

/*
#define SPI_NOP_CMD        0x0000
#define SPI_ANGLE_READ_CMD 0xFFFF
#define SPI_ERROR_READ_CMD 0x4001
#define SPI_HIGH_ZERO_POSITION_WRITE_CMD 0x8016
#define SPI_LOW_ZERO_POSITION_WRITE_CMD  0X0017
#define SPI_ZERO_POSITION_PERMANENT_STORAGE_CMD 0X0003
 */

/* SPI port SS1 (RB2) */
#define SPI_PORT_0_ENABLE _LATB2 = 0;
#define SPI_PORT_0_DISABLE _LATB2 = 1;
/* SPI port E1 (RD1) */
#define SPI_PORT_1_ENABLE _LATD1 = 0;
#define SPI_PORT_1_DISABLE _LATD1 = 1;
/* SPI port E2 (RD2) */
#define SPI_PORT_2_ENABLE _LATD2 = 0;
#define SPI_PORT_2_DISABLE _LATD2 = 1;
/* SPI port E3 (RD3) */
#define SPI_PORT_3_ENABLE _LATD3 = 0;
#define SPI_PORT_3_DISABLE _LATD3 = 1;
/* SPI port E4 (RE8) */
#define SPI_PORT_4_ENABLE _LATE8 = 0;
#define SPI_PORT_4_DISABLE _LATE8 = 1;

// 14 bits to code an angle, measured with an spi absolute encoder, from 0.0f
// degrees to 360.0f degrees.
// 2^14 (16384) values from 0 to 2^14 - 1 = 16383
#define MAX_COUNTS_IN_SPI_ABS_ENC 16383

enum spi_channel {
  ALPHA_ANGLE_SPI_ABS_ENC_CHANNEL = 2 // for the new driver
};

// Interrupt prototype

void spi_init();
// void spi_get_agc_mag_ang_absolute_encoder(int16_t channel, int16_t*
// spi_filt_agc_14, int16_t* spi_filt_mag_14, int16_t* spi_filt_ang_14, int16_t*
// spi_error);
int16_t spi_get_agc_mag_ang_absolute_encoder(int16_t channel,
                                             int16_t *spi_agc_14,
                                             int16_t *spi_mag_14,
                                             int16_t *spi_ang_14,
                                             int16_t *spi_error);
int16_t spi_set_zero_position_absolute_encoder(int16_t channel,
                                               int16_t zero_position,
                                               int16_t permanent_storage);

#endif
