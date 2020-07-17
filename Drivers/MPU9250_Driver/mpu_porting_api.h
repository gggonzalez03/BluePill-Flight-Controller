/*
 * mpu_porting_api.h
 *
 *  Created on: Jul 15, 2020
 *      Author: ginggonzalez
 */

#ifndef MPU9250_DRIVER_MPU_PORTING_API_H_
#define MPU9250_DRIVER_MPU_PORTING_API_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef MPU_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);

HAL_StatusTypeDef MPU_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

void MPU_delay_ms(unsigned long num_ms);

void MPU_delay_s(unsigned long num_ms);

void MPU_get_ms(unsigned long *count);

void ConsoleLog(char* msg);

#ifdef __cplusplus
}
#endif

#endif /* MPU9250_DRIVER_MPU_PORTING_API_H_ */
