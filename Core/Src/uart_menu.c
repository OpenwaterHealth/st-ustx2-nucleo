/*
 * uart_comms.c
 *
 *  Created on: Jan 3, 2024
 *      Author: gvigelet
 */

#include <console_commands.h>
#include "uart_menu.h"
#include "utils.h"
#include "i2c_func.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

// Private variables

static void UART_MENU_Task(void *argument);

TaskHandle_t xCommsMenuTask = NULL;


// This is the FreeRTOS task
static void UART_MENU_Task(void *argument) {
    char userInput;
    uint8_t slave_addr = 0xff;

    I2C_scan();
    if(found_address_count>0){
    	I2C_set_selected_slave(found_addresses[0]);
    }
	HAL_UART_AbortReceive(&huart2);

	print_main_menu(slave_addr);

    while(1) {
    	slave_addr = I2C_get_selected_slave();
    	print_prompt();

        // Wait for user input
        HAL_UART_Receive(&huart2, (uint8_t *)&userInput, 1, HAL_MAX_DELAY);
        process_menu_command(&userInput, slave_addr);

    }

}

// Function to start the UART task
void comms_menu_start(void) {

	xTaskCreate(
		UART_MENU_Task,                  /* Task function */
        "commsMenuTask",                /* Name of task */
        128,                        /* Stack size in words (not bytes!) */
        NULL,                       /* Parameter passed into the task */
        tskIDLE_PRIORITY + 4,       /* Priority */
		&xCommsMenuTask                 /* Task handle */
    );
}




