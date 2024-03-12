/*
 * common.h
 *
 *  Created on: Mar 12, 2024
 *      Author: gvigelet
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#define COMMAND_MAX_SIZE 2048


/*
 * Communication Protocol Packet Structure:
 *
 * | Start Byte | ID | Packet Type | Command | Length | Payload | CRC16 | End Byte |
 *
 * Definitions:
 *
 * Start Byte:
 *   - A predefined byte to indicate the beginning of a packet.
 *   - Value: 0xAA (as per USTX_ProtocolTypes)
 *
 * ID:
 *   - ID for transaction used for response or continuation data.
 *
 * Packet Type:
 *   - Indicates the type of the packet being sent or received.
 *   - Possible values:
 *     - OW_ACK: Acknowledgment packet (Value: 0xE0)
 *     - OW_NAK: Negative acknowledgment packet (Value: 0xE1)
 *     - OW_CMD: Command packet (Value: 0xE2)
 *     - OW_RESP: Response packet (Value: 0xE3)
 *     - OW_DATA: Data packet (Value: 0xE4)
 *     - OW_JSON: JSON data packet (Value: 0xE5)
 *     - OW_BAD_CRC: Bad CRC error packet (Value: 0xEE)
 *     - OW_ERROR: General error packet (Value: 0xEF)
 *
 * Command:
 *   - Specifies the command or action to be taken.
 *   - Possible values:
 *     - USTX_NOP: No operation command (Value: 0xB0)
 *     - USTX_PING: Ping command (Value: 0xB1)
 *     - USTX_VERSION: Request for version information (Value: 0xB2)
 *     - USTX_ID: Request for ID information (Value: 0xB3)
 *
 * Length:
 *   - Indicates the length of the payload data.
 *
 * Payload:
 *   - Contains the actual data or information being sent or received.
 *   - Size can vary up to a maximum of COMMAND_MAX_SIZE (2048 bytes).
 *
 * CRC16:
 *   - A 16-bit Cyclic Redundancy Check value for error-checking purposes.
 *   - Helps in detecting errors in the transmitted data.
 *
 * End Byte:
 *   - A predefined byte to indicate the end of a packet.
 *   - Value: 0xDD (as per USTX_ProtocolTypes)
 *
 */



typedef enum {
	OW_START_BYTE = 0xAA,
	OW_END_BYTE = 0xDD,
} USTX_ProtocolTypes;

typedef enum {
	OW_ACK = 0xE0,
	OW_NAK = 0xE1,
	OW_CMD = 0xE2,
	OW_RESP = 0xE3,
	OW_DATA = 0xE4,
	OW_JSON = 0xE5,
	OW_TX7332 = 0xE6,
	OW_BAD_PARSE = 0xEC,
	OW_BAD_CRC = 0xED,
	OW_UNKNOWN = 0xEE,
	OW_ERROR = 0xEF,

} UartPacketTypes;

typedef enum {
	USTX_NOP = 0xB0,
	USTX_PING = 0xB1,
	USTX_PONG = 0xB2,
	USTX_VERSION = 0xB3,
	USTX_ID = 0xB4,
	USTX_ECHO = 0xB5,

} UstxCommands;

typedef enum {
	CMD_READ_ADDR = 0xC0,
	CMD_WRITE_ADDR = 0xC1,
	CMD_WRITE_DEMO = 0xC2,
	CMD_VERIFY_DEMO = 0xC3,
	CMD_WRITE_BLOCK = 0xC4,
	CMD_SET_SWTRIG = 0xCA,
	CMD_GET_SWTRIG = 0xCB,
	CMD_START_SWTRIG = 0xCC,
	CMD_STOP_SWTRIG = 0xCD,
	CMD_STATUS_SWTRIG = 0xCE,

} TX7332Commands;

typedef struct  {
	uint16_t id;
	uint8_t packet_type;
	uint8_t command;
	uint16_t data_len;
	uint16_t crc;
	uint8_t* data;
} UartPacket;

typedef struct {
    uint32_t TriggerFrequencyHz;
    uint32_t TriggerMode;
    uint32_t TriggerPulseCount;
    uint32_t TriggerPulseWidthUsec;
    uint32_t TriggerStatus;
} TimerData;

void updateTimerDataFromPeripheral(TIM_HandleTypeDef* htim, uint32_t channel);


#endif /* INC_COMMON_H_ */
