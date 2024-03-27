/*
 * i2c_protocol.h
 *
 *  Created on: Jan 8, 2024
 *      Author: gvigelet
 */

#ifndef INC_I2C_PROTOCOL_H_
#define INC_I2C_PROTOCOL_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define I2C_MAX_TX_BUFFER_SIZE 1024
#define I2C_TX_PKT_ND_SIZE 10

typedef struct {
	uint8_t sb;
	uint8_t id;
	uint8_t cmd;
	uint8_t status;
	uint16_t data_len;
	uint8_t* pData;
	uint16_t crc;
	uint8_t eb;
} I2C_TX_Packet;

typedef enum {
	CMD_AFE_TOGGLE_LED = 0x01,
	CMD_TURN_OFF_LED = 0x02,
	CMD_TURN_ON_LED = 0x03,
	CMD_HB_LED = 0x04,
	CMD_TX_DEMO = 0x05
} I2C_USTX_AFE_Command;

void i2c_packet_print(const I2C_TX_Packet* packet);
bool i2c_packet_fromBuffer(const uint8_t* buffer, I2C_TX_Packet* pTX);
uint16_t i2c_packet_toBuffer(I2C_TX_Packet* pTX, uint8_t* buffer);
void send_i2c_packet(uint8_t address, I2C_TX_Packet *pTX);
void SendI2CPacket(uint8_t address, uint8_t command);

#endif /* INC_I2C_PROTOCOL_H_ */

