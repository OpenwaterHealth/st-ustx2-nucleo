/*
 * i2c_protocol.h
 *
 *  Created on: Jan 8, 2024
 *      Author: gvigelet
 */

#ifndef INC_I2C_PROTOCOL_H_
#define INC_I2C_PROTOCOL_H_

#define I2C_MAX_TX_BUFFER_SIZE 8

typedef struct  {
	uint8_t sb;
	uint8_t cmd;
	//uint8_t data_len;
	//uint8_t* pData;
	uint16_t crc;
	uint8_t reserved;
	uint8_t reserved2;
	uint8_t reserved3;
	uint8_t eb;
} I2C_TX_Packet;

typedef enum {
	CMD_TOGGLE_LED = 0x01,
	CMD_TURN_OFF_LED = 0x02,
	CMD_TURN_ON_LED = 0x03,
	CMD_HB_LED = 0x04
} I2C_USTX_AFE_Command;

void SendI2CPacket(uint8_t address, uint8_t command);
void PrintI2CPacket(uint8_t* packet);

#endif /* INC_I2C_PROTOCOL_H_ */

