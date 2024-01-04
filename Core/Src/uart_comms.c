/*
 * uart_comms.c
 *
 *  Created on: Jan 3, 2024
 *      Author: gvigelet
 */

#include "uart_comms.h"
#include "utils.h"
#include "commands.h"

#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

// Private variables

static void UART_Task(void *argument);

TaskHandle_t xCommsTask = NULL;


static void I2C_Scan() {
    printf("Scanning I2C bus for devices...\r\n");

    for (uint8_t address = 0x20; address < 0x7f; address++) {
        HAL_StatusTypeDef status;
        status = HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 2, 2); // Address shift left by 1 for read/write bit
        if (status == HAL_OK) {
            printf("Device found at address 0x%02X\r\n", address);
        }
    }

    printf("\r\n");
    fflush(stdout);
}

// This is the FreeRTOS task
static void UART_Task(void *argument) {
    char userInput;
    I2C_Scan();
	HAL_UART_AbortReceive(&huart2);

	print_main_menu();

    while(1) {
    	print_prompt();

        // Wait for user input
        HAL_UART_Receive(&huart2, (uint8_t *)&userInput, 1, HAL_MAX_DELAY);
        process_command(&userInput);

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




