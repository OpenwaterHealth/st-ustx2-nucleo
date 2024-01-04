/*
 * uart_comms.c
 *
 *  Created on: Jan 3, 2024
 *      Author: gvigelet
 */

#include "uart_comms.h"
#include "utils.h"
#include "i2c_func.h"
#include "commands.h"

#include <string.h>

extern UART_HandleTypeDef huart2;

// Private variables

static void UART_Task(void *argument);

TaskHandle_t xCommsTask = NULL;


// This is the FreeRTOS task
static void UART_Task(void *argument) {
    char userInput;
    uint8_t slave_addr = I2C_scan();
	HAL_UART_AbortReceive(&huart2);

	print_main_menu(slave_addr);

    while(1) {
    	slave_addr = I2C_get_selected_slave();
    	print_prompt();

        // Wait for user input
        HAL_UART_Receive(&huart2, (uint8_t *)&userInput, 1, HAL_MAX_DELAY);
        process_command(&userInput, slave_addr);

    }

}

// Function to start the UART task
void comms_module_start(void) {

	xTaskCreate(
        UART_Task,                  /* Task function */
        "commsTask",                /* Name of task */
        128,                        /* Stack size in words (not bytes!) */
        NULL,                       /* Parameter passed into the task */
        tskIDLE_PRIORITY + 4,       /* Priority */
		&xCommsTask                 /* Task handle */
    );
}




