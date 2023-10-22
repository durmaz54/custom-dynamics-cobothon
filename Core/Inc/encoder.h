/*
 * encoder.h
 *
 *  Created on: Oct 21, 2023
 *      Author: ahmet
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>


/* Volatile register address. */
#define AS5047P_NOP ((uint16_t)0x0000)
#define AS5047P_ERRFL ((uint16_t)0x0001)
#define AS5047P_PROG ((uint16_t)0x0003)
#define AS5047P_DIAAGC ((uint16_t)0x3FFC)
#define AS5047P_MAG ((uint16_t)0x3FFD)
#define AS5047P_ANGLEUNC ((uint16_t)0x3FFE)
#define AS5047P_ANGLECOM ((uint16_t)0x3FFF)

/* Non-Volatile register address. */
#define AS5047P_ZPOSM ((uint16_t)0x0016)
#define AS5047P_ZPOSL ((uint16_t)0x0017)
#define AS5047P_SETTINGS1 ((uint16_t)0x0018)
#define AS5047P_SETTINGS2 ((uint16_t)0x0019)

#define AS5047P_STEEINGS1_DEFAULT ((uint8_t)0x01)
#define AS5047P_STEEINGS2_DEFAULT ((uint8_t)0x00)



typedef void (*write_reg_t)(uint16_t data);
typedef uint16_t (*read_reg_t)(void);
//typedef void (*pin_control_t)(bool pin);
typedef void (*delay_t)(void);


typedef struct
{
    write_reg_t     write_reg;
    read_reg_t      read_reg;
    //pin_control_t   pin_control;
    delay_t         delay;
}as5047p_init_t;


typedef enum
{
	without_daec = 0,
	with_daec = !without_daec
}as5047p_daec_t;


void as5047p_sendData(as5047p_init_t* as5047p, uint16_t address, uint16_t data);

uint16_t as5047p_readData(as5047p_init_t* as5047p, uint16_t address);

void as5047p_config(as5047p_init_t *as5047p, uint8_t settings1, uint8_t settings2);

uint16_t as5047p_getErrorStatus(as5047p_init_t* as5047p);

int8_t as5047p_getPosition(as5047p_init_t* as5047p, as5047p_daec_t with_daec, uint16_t *position);

int8_t as5047p_getAngle(as5047p_init_t* as5047p, as5047p_daec_t with_daec, float *angle_degree);

void as5047p_setZero(as5047p_init_t* as5047p, uint16_t position);

void as5047p_reset(as5047p_init_t* as5047p);


#endif /* INC_ENCODER_H_ */
