/*
 * uart_comms.c
 *
 *  Created on: Mar 11, 2024
 *      Author: gvigelet
 */

#include "if_commands.h"
#include "main.h"
#include "uart_comms.h"
#include "utils.h"
#include <string.h>

// Private variables
extern uint8_t rxBuffer[COMMAND_MAX_SIZE];
extern uint8_t txBuffer[COMMAND_MAX_SIZE];


static void UART_INTERFACE_Task(void *argument);
volatile uint32_t ptrReceive;

TaskHandle_t xCommsInterfaceTask = NULL;

static void UART_INTERFACE_SendDMA(UartPacket* pResp)
{
	while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
	memset(txBuffer, 0, sizeof(txBuffer));
	int bufferIndex = 0;

	txBuffer[bufferIndex++] = OW_START_BYTE;
	txBuffer[bufferIndex++] = pResp->id >> 8;
	txBuffer[bufferIndex++] = pResp->id & 0xFF;
	txBuffer[bufferIndex++] = pResp->packet_type;
	txBuffer[bufferIndex++] = pResp->command;
	txBuffer[bufferIndex++] = (pResp->data_len) >> 8;
	txBuffer[bufferIndex++] = (pResp->data_len) & 0xFF;
	memcpy(&txBuffer[bufferIndex], pResp->data, pResp->data_len);
	bufferIndex += pResp->data_len;
	uint16_t crc = util_crc16(&txBuffer[1], pResp->data_len + 6);
	txBuffer[bufferIndex++] = crc >> 8;
	txBuffer[bufferIndex++] = crc & 0xFF;

	txBuffer[bufferIndex++] = OW_END_BYTE;
	//hexdump("[DEBUG COMMS] TX Data", txBuffer, data_len+5);

	HAL_UART_Transmit_DMA(&huart1, txBuffer, bufferIndex);
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

// This is the FreeRTOS task
static void UART_INTERFACE_Task(void *argument) {

	memset(rxBuffer, 0, sizeof(rxBuffer));
	ptrReceive = 0;

	HAL_UART_AbortReceive(&huart1);

	UartPacket cmd;
	UartPacket resp;
    uint16_t calculated_crc;

    while(1) {
    	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer, COMMAND_MAX_SIZE);

		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int bufferIndex = 0;

        if(rxBuffer[bufferIndex++] != OW_START_BYTE) {
            // Send NACK doesn't have the correct start byte
        	resp.id = cmd.id;
            resp.data_len = 0;
            resp.packet_type = OW_NAK;
            goto NextDataPacket;
        }

        cmd.id = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
        bufferIndex+=2;
        cmd.packet_type = rxBuffer[bufferIndex++];
        cmd.command = rxBuffer[bufferIndex++];

        // Extract payload length
        cmd.data_len = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
        bufferIndex+=2;

        // Check if data length is valid
        if (cmd.data_len > COMMAND_MAX_SIZE - bufferIndex && rxBuffer[COMMAND_MAX_SIZE-1] != OW_END_BYTE) {
            // Send NACK response due to no end byte
        	// data can exceed buffersize but every buffer must have a start and end packet
        	// command that will send more data than one buffer will follow with data packets to complete the request
        	resp.id = cmd.id;
            resp.data_len = 0;
            resp.packet_type = OW_NAK;
            goto NextDataPacket;
        }

        // Extract data pointer
        cmd.data = &rxBuffer[bufferIndex];
        if (cmd.data_len > COMMAND_MAX_SIZE)
        {
        	bufferIndex=COMMAND_MAX_SIZE-3; // [3 bytes from the end should be the crc for a continuation packet]
        }else{
        	bufferIndex += cmd.data_len; // move pointer to end of data
        }

        // Extract received CRC
        cmd.crc = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
        bufferIndex+=2;

        // Calculate CRC for received data

        if (cmd.data_len > COMMAND_MAX_SIZE)
        {
        	calculated_crc = util_crc16(&rxBuffer[1], COMMAND_MAX_SIZE-3);
        }
        else
        {
        	calculated_crc = util_crc16(&rxBuffer[1], cmd.data_len + 6);
        }

        // Check CRC
        if (cmd.crc != calculated_crc) {
            // Send NACK response due to bad CRC
        	resp.id = cmd.id;
            resp.data_len = 0;
            resp.packet_type = OW_BAD_CRC;
            goto NextDataPacket;
        }

        // Check end byte
        if (rxBuffer[bufferIndex++] != OW_END_BYTE) {
        	resp.id = cmd.id;
            resp.data_len = 0;
            resp.packet_type = OW_NAK;
            goto NextDataPacket;
        }

		resp = process_if_command(cmd);

NextDataPacket:

		UART_INTERFACE_SendDMA(&resp);
		memset(rxBuffer, 0, sizeof(rxBuffer));
		ptrReceive=0;

    }

}

// Callback functions
void comms_handle_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t pos) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == USART1) {
        // Notify the task
    	if (xCommsInterfaceTask != NULL) {
    		vTaskNotifyGiveFromISR(xCommsInterfaceTask, &xHigherPriorityTaskWoken);
    		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    	}
    }
}

void comms_handle_TxCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART1) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		// Notify the task
		if (xCommsInterfaceTask != NULL) {
			vTaskNotifyGiveFromISR(xCommsInterfaceTask, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == USART1) {
        // Handle errors here. Maybe reset DMA reception, etc.
    }
}

// Function to start the UART task
void comms_interface_init(void) {

	xTaskCreate(
		UART_INTERFACE_Task,                  /* Task function */
        "commsInterfaceTask",                /* Name of task */
        128,                        /* Stack size in words (not bytes!) */
        NULL,                       /* Parameter passed into the task */
        tskIDLE_PRIORITY + 4,       /* Priority */
		&xCommsInterfaceTask                 /* Task handle */
    );
}


