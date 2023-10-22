/*
 * encoder.c
 *
 *  Created on: Oct 21, 2023
 *      Author: ahmet
 */

/**
 * @file as5047p.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  A library for AMS AS5047P rotary position sensor/magnetic encoder.
 * @copyright MIT License, Copyright (c) 2022 ZiTe
 *
 */


#include <stdint.h>
#include <stdbool.h>
#include "encoder.h"


/* ------------------------------------------------------ */
/*                          DEFINES                       */
/* ------------------------------------------------------ */
#define BIT_MODITY(src, i, val) ((src) ^= (-(val) ^ (src)) & (1UL << (i)))
#define BIT_READ(src, i) (((src) >> (i)&1U))
#define BIT_TOGGLE(src, i) ((src) ^= 1UL << (i))

#define OP_WRITE ((uint8_t)0)
#define OP_READ ((uint8_t)1)

/* ------------------------------------------------------ */
/*                 FUNCTIONS PROTOTYPE                    */
/* ------------------------------------------------------ */

void as5047p_sendData(as5047p_init_t* as5047p, uint16_t address, uint16_t data);

uint16_t as5047p_readData(as5047p_init_t* as5047p, uint16_t address);

static void as5047p_sendCommand(as5047p_init_t* as5047p, uint16_t address, uint8_t op_read_write);

static uint8_t is_evenParity(uint16_t data);

static void as5047p_nop(as5047p_init_t* as5047p_handle);


/* ------------------------------------------------------ */
/*                 PRIVATE FUNCTIONS                      */
/* ------------------------------------------------------ */

/**
 * @brief Sending data to register.
 *
 * @param as5047p_handle AS5047P handle.
 * @param address Register address.
 * @param data Data.
 */
void as5047p_sendData(as5047p_init_t* as5047p, uint16_t address, uint16_t data)
{
  uint16_t frame = data & 0x3FFF;

  /* Data frame bit 14 always low(0). */
  BIT_MODITY(frame, 14, 0);

  /* Parity bit(even) calculated on the lower 15 bits. */
  if (!is_evenParity(frame)){
    BIT_TOGGLE(frame, 15);
  }

  as5047p_sendCommand(as5047p, address, OP_WRITE);
  as5047p->write_reg(frame);
}

/**
 * @brief Reading data from register.
 *
 * @param as5047p_handle AS5047P handle.
 * @param address Register address.
 * @return Data.
 */
uint16_t as5047p_readData(as5047p_init_t* as5047p, uint16_t address)
{
  as5047p_sendCommand(as5047p, address, OP_READ);
  return as5047p->read_reg();
}

/**
 * @brief Sending read or write command to AS5047P.
 *
 * @param as5047p_handle AS5047P handle.
 * @param address Register address.
 * @param op_read_write Read of write opration.
 */
static void as5047p_sendCommand(as5047p_init_t* as5047p, uint16_t address, uint8_t op_read_write)
{
  uint16_t frame = address & 0x3FFF;

  /* R/W: 0 for write, 1 for read. */
  BIT_MODITY(frame, 14, op_read_write);

  /* Parity bit(even) calculated on the lower 15 bits. */
  if (!is_evenParity(frame))
  {
    BIT_TOGGLE(frame, 15);
  }

  as5047p->write_reg(frame);
}

/**
 * @brief No operation instruction.
 *
 * @param as5047p_handle AS5047P handle.
 */
static void as5047p_nop(as5047p_init_t* as5047p_handle)
{
  /* Reading the NOP register is equivalent to a nop (no operation) instruction. */
  as5047p_sendCommand(as5047p_handle, AS5047P_NOP, OP_READ);
}

/**
 * @brief Check data even parity.
 */
static uint8_t is_evenParity(uint16_t data)
{
  uint8_t shift = 1;
  while (shift < (sizeof(data) * 8))
  {
    data ^= (data >> shift);
    shift <<= 1;
  }
  return !(data & 0x1);
}

/* ------------------------------------------------------ */
/*                   GLOBAL FUNCTIONS                     */
/* ------------------------------------------------------ */

void as5047p_config(as5047p_init_t *as5047p, uint8_t settings1, uint8_t settings2)
{
  /* SETTINGS1 bit 0 --> Factory Setting: Pre-Programmed to 1. */
  BIT_MODITY(settings1, 0, 1);

  /* SETTINGS1 bit 1 --> Not Used: Pre-Programmed to 0, must not be overwritten. */
  BIT_MODITY(settings1, 1, 0);

  as5047p_sendData(as5047p, AS5047P_SETTINGS1, (uint16_t)(settings1 & 0x00FF));
  as5047p_sendData(as5047p, AS5047P_SETTINGS2, (uint16_t)(settings2 & 0x00FF));
}

/**
 * @brief Reading error flags.
 *
 * @param as5047p_handle AS5047P handle.
 * @return Error flags. 0 for no error occurred.
 */
uint16_t as5047p_getErrorStatus(as5047p_init_t* as5047p)
{
  return as5047p_readData(as5047p, AS5047P_ERRFL);
}

/**
 * @brief Read current position.
 *
 * @param as5047p_handle AS5047P handle.
 * @param with_daec With or without dynamic angle error compensation (DAEC).
 * @param position Current position raw value.
 * @return Status code.
 *         0: Success.
 *         -1: Error occurred.
 */
int8_t as5047p_getPosition(as5047p_init_t* as5047p, as5047p_daec_t with_daec, uint16_t *position)
{
  uint16_t data;

  if (with_daec){
    /* Measured angle WITH dynamic angle error compensation(DAEC). */
    data = as5047p_readData(as5047p, AS5047P_ANGLECOM);
  }else {
    /* Measured angle WITHOUT dynamic angle error compensation(DAEC). */
    data = as5047p_readData(as5047p, AS5047P_ANGLEUNC);
  }

  if (BIT_READ(data, 14) == 0)
  {
    *position = data & 0x3FFF;
    return 0; /* No error occurred. */
  }
  return -1; /* Error occurred. */
}



/**
 * @brief Read current angle in degree.
 *
 * @param as5047p_handle AS5047P handle
 * @param with_daec With or without dynamic angle error compensation (DAEC).
 * @param angle_degree Current angle in degree.
 * @return Status code.
 *         0: Success.
 *         -1: Error occurred.
 */
int8_t as5047p_getAngle(as5047p_init_t* as5047p, as5047p_daec_t with_daec, float *angle_degree)
{
  uint16_t raw_position;
  int8_t error = as5047p_getPosition(as5047p, with_daec, &raw_position);

  if (error == 0)
  {
    /* Angle in degree = value * ( 360 / 2^14). */
    *angle_degree = raw_position * (360.0 / 0x4000);
  }

  return error;
}



/**
 * @brief Set specify position as zero.
 *
 * @param as5047p_handle AS5047P handle.
 * @param position Position raw value.
 */
void as5047p_setZero(as5047p_init_t* as5047p, uint16_t position)
{
  /* 8 most significant bits of the zero position. */
  as5047p_sendData(as5047p, AS5047P_ZPOSM, ((position >> 6) & 0x00FF));

  /* 6 least significant bits of the zero position. */
  as5047p_sendData(as5047p, AS5047P_ZPOSL, (position & 0x003F));

  as5047p_nop(as5047p);
}

/**
 * @brief Reset.
 *
 * @param as5047p_handle
 */
void as5047p_reset(as5047p_init_t* as5047p)
{
  as5047p_config(as5047p, AS5047P_STEEINGS1_DEFAULT, AS5047P_STEEINGS2_DEFAULT);
  as5047p_setZero(as5047p, 0);
}


