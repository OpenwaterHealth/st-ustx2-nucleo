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

void reset_slaves(void);
void I2C_scan(void);
uint8_t I2C_get_selected_slave();
void I2C_set_selected_slave(uint8_t slave_addr);
void I2C_set_next_slave(void);
uint8_t send_buffer_to_slave(uint8_t slave_addr, uint8_t* pBuffer, uint16_t buf_len);

#endif /* INC_I2C_FUNC_H_ */
