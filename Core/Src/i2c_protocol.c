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

static uint8_t tx_buffer[I2C_MAX_TX_BUFFER_SIZE];

void i2c_packet_print(const I2C_TX_Packet* packet) {
    printf("\r\nI2C TX PACKET\r\n\r\n");
    printf("Start Byte (sb): 0x%02X\n", packet->sb);
    printf("ID: 0x%04X\n", packet->id);
    printf("Command (cmd): 0x%02X\n", packet->cmd);
    printf("Status: 0x%02X\n", packet->status);
    printf("Data Length: %d\n", packet->data_len);
    printf("Data: ");
    for (int i = 0; i < packet->data_len; i++) {
        printf("0x%02X ", packet->pData[i]);
    }
    printf("\nCRC: 0x%04X\n", packet->crc);
    printf("End Byte (eb): 0x%02X\n", packet->eb);
}

bool i2c_packet_fromBuffer(const uint8_t* buffer, I2C_TX_Packet* pTX) {
    bool ret = false;
    uint16_t crc = 0xFFFF;
    const uint8_t* pBuff = buffer;

    pTX->sb = *buffer; // Start Byte
    buffer++;
    pTX->id = *(uint16_t*)buffer; // Packet ID
    buffer += 2;
    pTX->cmd = *buffer; // Command ID
    buffer++;
    pTX->status = *buffer; // Status ID
    buffer++;
    pTX->data_len = *(uint16_t*)buffer; // Data Length
    buffer += 2;
    pTX->pData = (uint8_t*)buffer;
    buffer += pTX->data_len;
    pTX->crc = *(uint16_t*)buffer; // CRC
    buffer += 2;
    pTX->eb = *buffer; // End Byte

    // Calculate CRC
    crc = util_crc16(&pBuff[1], pTX->data_len + 6);
    ret = crc == pTX->crc;
    return ret;
}

uint16_t i2c_packet_toBuffer(I2C_TX_Packet* pTX, uint8_t* buffer) {
    uint16_t crc = 0xFFFF;
    int i = 0;
    uint8_t* pBuff = buffer;

    *buffer = 0xA5; // Start Byte
    buffer++;
    *(uint16_t*)buffer = pTX->id; // Packet ID
    buffer += 2;
    *buffer = pTX->cmd; // Command ID
    buffer++;
    *buffer = pTX->status; // Status ID
    buffer++;
    *(uint16_t*)buffer = pTX->data_len; // Data Length
    buffer += 2;
    if (pTX->pData) {
        for (i = 0; i < pTX->data_len; i++) {
            *buffer = pTX->pData[i];
            buffer++;
        }
    }
    // Calculate CRC
    crc = util_crc16(&pBuff[1], pTX->data_len + 6);
    *(uint16_t*)buffer = crc; // CRC
    buffer += 2;
    *buffer = 0x5A; // End Byte
    return crc;
}


void send_i2c_packet(uint8_t address, I2C_TX_Packet *pTX)
{
	uint16_t crc = 0xFFFF;
	int i = 0;
	memset(tx_buffer, 0, I2C_MAX_TX_BUFFER_SIZE);
	uint8_t* pTxBuffer = tx_buffer;

	*pTxBuffer = 0xA5; 						// Start Byte
	pTxBuffer++;
    *pTxBuffer = pTX->id & 0xFF;			// packet id
    pTxBuffer++;
	*pTxBuffer = (pTX->id >> 8) & 0xFF;
	pTxBuffer++;
	*pTxBuffer = pTX->cmd;					// command id
	pTxBuffer++;
	*pTxBuffer = pTX->status;				// status id
	pTxBuffer++;
    *pTxBuffer = pTX->data_len & 0xFF;			// data length
    pTxBuffer++;
	*pTxBuffer = (pTX->data_len >> 8) & 0xFF;
	pTxBuffer++;
	if(pTX->pData){
		for(i=0; i < pTX->data_len; i++)
		{
			*pTxBuffer = pTX->pData[i];
			pTxBuffer++;
		}
	}
	// calculate CRC
    crc = util_crc16(&tx_buffer[1], pTX->data_len + 6); // Calculate CRC
    *pTxBuffer = crc & 0xFF;
    pTxBuffer++;
	*pTxBuffer = (crc >> 8) & 0xFF;
	pTxBuffer++;
	*pTxBuffer = 0x5A;						// End Byte

#ifdef DEBUG_COMMS
	i2c_packet_print(pTX);
#endif
	send_buffer_to_slave(address, tx_buffer, I2C_MAX_TX_BUFFER_SIZE);

}


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

	send_buffer_to_slave(address, tx_buffer, 8); // this is temporary while we get the reworked protocol in place
}
