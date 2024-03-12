/*
 * if_commands.h
 *
 *  Created on: Mar 11, 2024
 *      Author: gvigelet
 */

#ifndef INC_IF_COMMANDS_H_
#define INC_IF_COMMANDS_H_

#include "uart_comms.h"
#include "utils.h"

UartPacket process_if_command(UartPacket cmd);

#endif /* INC_IF_COMMANDS_H_ */
