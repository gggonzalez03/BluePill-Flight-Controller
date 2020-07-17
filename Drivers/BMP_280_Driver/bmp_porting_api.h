/*
 * bmp_porting_api.h
 *
 *  Created on: Jul 15, 2020
 *      Author: ginggonzalez
 */

#ifndef BMP_280_DRIVER_BMP_PORTING_API_H_
#define BMP_280_DRIVER_BMP_PORTING_API_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void BMP_delay_ms(uint32_t period_ms);
int8_t BMP_i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t BMP_i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);


#ifdef __cplusplus
}
#endif


#endif /* BMP_280_DRIVER_BMP_PORTING_API_H_ */
