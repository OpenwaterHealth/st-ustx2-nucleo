/*
 * uart_comms.h
 *
 *  Created on: Mar 12, 2024
 *      Author: gvigelet
 */

#ifndef INC_UART_COMMS_H_
#define INC_UART_COMMS_H_

#include "main.h"  // This should contain your HAL includes and other basic includes.
#include "cmsis_os.h"
#include "common.h"
#include <stdio.h>
#include <stdbool.h>

void comms_interface_init(void);
void comms_start_task(void);
void comms_handle_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size);
void comms_handle_TxCallback(UART_HandleTypeDef *huart);

#endif /* INC_UART_COMMS_H_ */
