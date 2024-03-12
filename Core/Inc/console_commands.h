/*
 * console_commands.h
 *
 *  Created on: Jan 3, 2024
 *      Author: gvigelet
 */

#ifndef INC_CONSOLE_COMMANDS_H_
#define INC_CONSOLE_COMMANDS_H_

#include <stdint.h>


void print_main_menu(uint8_t slave_address);
void print_prompt(void);
void process_menu_command(char* input, uint8_t selected_slave);

#endif /* INC_CONSOLE_COMMANDS_H_ */
