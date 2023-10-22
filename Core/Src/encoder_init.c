/*
 * encoder_init.c
 *
 *  Created on: Oct 21, 2023
 *      Author: ahmet
 */

#include <stdint.h>

#include "main.h"
#include "encoder.h"

extern SPI_HandleTypeDef hspi3;

void spi_write(uint16_t data)
{
	uint8_t buffer[2];
	buffer[0] = (uint8_t)(data & 0xFF);
	buffer[1] = (uint8_t)((data >> 8) & 0xFF);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_StatusTypeDef ret = HAL_SPI_Transmit(&hspi3, buffer, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	for(int i=0; i<10000; i++);
}

uint16_t spi_read(void)
{
	uint8_t buffer[2];
	uint16_t val;

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_StatusTypeDef ret = HAL_SPI_Receive(&hspi3, buffer, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	for(int i=0; i<10000; i++);
	val = ((uint16_t)buffer[1]<<8) | buffer[0];

	return val;
}

void delay(void)
{
	HAL_Delay(10);
}


void encoder_init(as5047p_init_t *init)
{
	init->write_reg = spi_write;
	init->read_reg = spi_read;
	init->delay = delay;

}
