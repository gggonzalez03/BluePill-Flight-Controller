/*
 * bmp_porting_api.c
 *
 *  Created on: Jul 15, 2020
 *      Author: ginggonzalez
 */

#include "bmp_porting_api.h"

void BMP_delay_ms(uint32_t period_ms)
{
	HAL_Delay(period_ms);
}

int8_t BMP_i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

	while (HAL_I2C_Mem_Write(&hi2c1, i2c_addr, reg_addr, 1, reg_data, length, 100) != HAL_OK)
	{
//		MX_I2C1_Init();
	}

	return (int8_t)HAL_OK;
}

int8_t BMP_i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

	while (HAL_I2C_Mem_Read(&hi2c1, i2c_addr, reg_addr, 1, reg_data, length, 100) != HAL_OK)
	{
//		MX_I2C1_Init();
	}

	return (int8_t)HAL_OK;
}
