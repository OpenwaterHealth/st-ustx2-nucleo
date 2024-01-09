/*
 * i2c_protocol.c
 *
 *  Created on: Jan 8, 2024
 *      Author: gvigelet
 */

#include "main.h"
#include "i2c_protocol.h"
#include "utils.h"
#include "i2c_func.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

static uint8_t tx_buffer[I2C_MAX_TX_BUFFER_SIZE];

#ifdef DEBUG_COMMS
void PrintI2CPacket(uint8_t* packet) {
    printf("\r\nI2C TX PACKET\r\n\r\n");
    printf("Start Byte (sb): 0x%02X\r\n", packet[0]);
    printf("Command (cmd): 0x%02X\r\n", packet[1]);
    uint16_t crc = *((uint16_t *)&packet[2]);
    printf("CRC (crc): 0x%04X\r\n", crc);
    printf("Reserved Byte 1 (reserved): 0x%02X\r\n", packet[4]);
    printf("Reserved Byte 2 (reserved2): 0x%02X\r\n", packet[5]);
    printf("Reserved Byte 3 (reserved3): 0x%02X\r\n", packet[6]);
    printf("End Byte (eb): 0x%02X\r\n", packet[7]);
}
#endif

void SendI2CPacket(uint8_t address, uint8_t command)
{
	uint16_t crc = 0xFFFF;
	memset(tx_buffer, 0, I2C_MAX_TX_BUFFER_SIZE);
	uint8_t* pTx = tx_buffer;

	*pTx = 0xA5;
	pTx++;
	*pTx = command;
	pTx++;
    crc = util_crc16(&tx_buffer[1], 1); // Calculate CRC
    *pTx = crc & 0xFF;
	pTx++;
	*pTx = (crc >> 8) & 0xFF;
	pTx++;
	*pTx = 0x00;
	pTx++;
	*pTx = 0x00;
	pTx++;
	*pTx = 0x00;
	pTx++;
	*pTx = 0x5A;

#ifdef DEBUG_COMMS
	PrintI2CPacket(tx_buffer);
#endif
	send_packet_to_slave(address, tx_buffer, I2C_MAX_TX_BUFFER_SIZE);

}
