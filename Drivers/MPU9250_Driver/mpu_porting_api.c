/*
 * mpu_porting_api.c
 *
 *  Created on: Jul 15, 2020
 *      Author: ginggonzalez
 */

#include <string.h>
#include "mpu_porting_api.h"

HAL_StatusTypeDef MPU_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{
	HAL_StatusTypeDef data_tx_ret;

	uint8_t reg_ptr = (uint8_t)reg_addr;
	slave_addr = slave_addr << 1;

	data_tx_ret = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)slave_addr, (uint16_t)reg_ptr, 1, (uint8_t*)(data), (uint8_t)length, 1);

	while (data_tx_ret != HAL_OK)
	{
//		MX_I2C1_Init();
		data_tx_ret = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)slave_addr, (uint16_t)reg_ptr, 1, (uint8_t*)(data), (uint8_t)length, 1);
	}

	return data_tx_ret;
}

HAL_StatusTypeDef MPU_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
	HAL_StatusTypeDef data_rx_ret;

	uint8_t reg_ptr = (uint8_t)reg_addr;
	slave_addr = slave_addr << 1;

	data_rx_ret = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)slave_addr, (uint16_t)reg_ptr, 1, (uint8_t*)data, (uint16_t)length, 10);

	while(data_rx_ret != HAL_OK)
	{
//		MX_I2C1_Init();
		data_rx_ret = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)slave_addr, (uint16_t)reg_ptr, 1, (uint8_t*)data, (uint16_t)length, 10);
	}

	return data_rx_ret;
}

void MPU_delay_s(unsigned long num_ms)
{
	HAL_Delay((uint32_t)num_ms * 1000);
}

void MPU_delay_ms(unsigned long num_ms)
{
	HAL_Delay((uint32_t)num_ms);
}

void MPU_get_ms(unsigned long *count)
{
	count[0] = HAL_GetTick();
}

void ConsoleLog(char* msg)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}
