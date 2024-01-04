/*
 * i2c_func.h
 *
 *  Created on: Jan 4, 2024
 *      Author: gvigelet
 */

#ifndef INC_I2C_FUNC_H_
#define INC_I2C_FUNC_H_

#include "main.h"  // This should contain your HAL includes and other basic includes.
#include "cmsis_os.h"
#include <stdio.h>
#include <stdbool.h>

uint8_t I2C_scan();
uint8_t I2C_get_selected_slave();
void I2C_set_selected_slave(uint8_t slave_addr);

#endif /* INC_I2C_FUNC_H_ */
