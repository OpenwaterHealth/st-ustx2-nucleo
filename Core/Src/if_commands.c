/*
 * if_commands.c
 *
 *  Created on: Mar 12, 2024
 *      Author: gvigelet
 */

#include "main.h"
#include "if_commands.h"
#include "common.h"
#include "uart_comms.h"
#include "i2c_master.h"
#include "i2c_protocol.h"
#include "trigger.h"

#include <stdio.h>
#include <string.h>

static char retTriggerJson[256];
static uint8_t FIRMWARE_VERSION_DATA[3] = {0, 1, 1};
static uint32_t id_words[3] = {0};
uint8_t receive_afe_buff[256] = {0};
uint8_t send_afe_buff[256] = {0};

static void process_basic_command(UartPacket *uartResp, UartPacket cmd)
{
	switch (cmd.command)
	{
	case OW_CMD_NOP:
		uartResp->command = OW_CMD_NOP;
		break;
	case OW_CMD_PING:
		uartResp->command = OW_CMD_PONG;
		break;
	case OW_CMD_PONG:
		uartResp->command = OW_CMD_PING;
		break;
	case OW_CMD_VERSION:
		uartResp->command = OW_CMD_VERSION;
		uartResp->data_len = sizeof(FIRMWARE_VERSION_DATA);
		uartResp->data = FIRMWARE_VERSION_DATA;
		break;
	case OW_CMD_HWID:
		uartResp->command = OW_CMD_HWID;
		id_words[0] = HAL_GetUIDw0();
		id_words[1] = HAL_GetUIDw1();
		id_words[2] = HAL_GetUIDw2();
		uartResp->data_len = 16;
		uartResp->data = (uint8_t *)&id_words;
		break;
	case OW_CMD_ECHO:
		// exact copy
		uartResp->id = cmd.id;
		uartResp->packet_type = cmd.packet_type;
		uartResp->command = cmd.command;
		uartResp->data_len = cmd.data_len;
		uartResp->data = cmd.data;
		break;
	case OW_CMD_TOGGLE_LED:
		uartResp->id = cmd.id;
		uartResp->packet_type = cmd.packet_type;
		uartResp->command = cmd.command;
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}
}

static void process_afe_send(UartPacket *uartResp, UartPacket cmd)
{
	uint16_t send_len = 0;
	I2C_TX_Packet send_afe_packet;

	// initialize send packet
	send_afe_packet.id = cmd.id;
	send_afe_packet.cmd = cmd.command;
	send_afe_packet.reserved =0;
	send_afe_packet.data_len = 0;
	send_afe_packet.pData = 0;

	if(found_address_count == 0){
		printf("No AFE's found\r\n");
		uartResp->id = cmd.id;
		uartResp->packet_type = OW_ERROR;
		uartResp->command = cmd.command;
		return;
	}else{

		uartResp->id = cmd.id;
		uartResp->command = cmd.command;
		uartResp->data_len = 0;
	}

	send_len = i2c_packet_toBuffer(&send_afe_packet, send_afe_buff);
	send_buffer_to_slave(0x28, send_afe_buff, send_len);
	uartResp->packet_type = OW_ACK;
#if 0
	switch (cmd.command)
	{
	case OW_CMD_PING:
		// Ping Slave
		printf("Send Ping\r\n");
		//HAL_Delay(1);
		//rx_len = read_status_register_of_slave(0x28, receive_afe_buff, 255);
		//printf("Received %d Bytes \r\n", rx_len);
		//printBuffer(receive_afe_buff, rx_len);
		//i2c_status_packet_fromBuffer(receive_afe_buff, &afe_satus_packet);
		//i2c_status_packet_print(&afe_satus_packet);
		break;
	case OW_CMD_PONG:
		// Pong Slave
		printf("Send Pong\r\n");
		send_len = i2c_packet_toBuffer(&send_afe_packet, send_afe_buff);
		send_buffer_to_slave(0x28, send_afe_buff, send_len);
		uartResp->packet_type = OW_ACK;
		//HAL_Delay(1);
		//rx_len = read_status_register_of_slave(0x28, receive_afe_buff, 255);
		//printf("Received %d Bytes \r\n", rx_len);
		//printBuffer(receive_afe_buff, rx_len);
		//i2c_status_packet_fromBuffer(receive_afe_buff, &afe_satus_packet);
		//i2c_status_packet_print(&afe_satus_packet);
		break;
	case OW_CMD_TOGGLE_LED:
		// Toggle Slave
		printf("Send Toggle LED\r\n");
		send_len = i2c_packet_toBuffer(&send_afe_packet, send_afe_buff);
		send_buffer_to_slave(0x28, send_afe_buff, send_len);
		uartResp->packet_type = OW_ACK;
		HAL_Delay(250);
		printf("Read from slave\r\n");
		rx_len = read_status_register_of_slave(0x28, receive_afe_buff, 1024);
		printf("Received %d Bytes \r\n", rx_len);
		printBuffer(receive_afe_buff, rx_len);
		i2c_status_packet_fromBuffer(receive_afe_buff, &afe_satus_packet);

		i2c_status_packet_print(&afe_satus_packet);

		break;
	default:
		printf("Unknown\r\n");
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}
#endif
}

static void process_afe_read(UartPacket *uartResp, UartPacket cmd)
{
	uint16_t rx_len = 0;
	I2C_STATUS_Packet afe_satus_packet;

	if(found_address_count == 0){
		printf("No AFE's found\r\n");
		uartResp->id = cmd.id;
		uartResp->packet_type = OW_ERROR;
		uartResp->command = cmd.command;
		return;
	}else{
		uartResp->id = cmd.id;
		uartResp->packet_type = cmd.packet_type;
		uartResp->command = cmd.command;
		uartResp->data_len = 0;
	}

	rx_len = read_status_register_of_slave(0x28, receive_afe_buff, 1024);
	printf("Received %d Bytes \r\n", rx_len);
	printBuffer(receive_afe_buff, rx_len);
	i2c_status_packet_fromBuffer(receive_afe_buff, &afe_satus_packet);
	i2c_status_packet_print(&afe_satus_packet);
}

static void CONTROLLER_ProcessCommand(UartPacket *uartResp, UartPacket cmd)
{
	switch (cmd.command)
	{
		case OW_CMD_PING:
			uartResp->command = cmd.command;
			break;
		case OW_CMD_PONG:
			uartResp->command = cmd.command;
			break;
		case OW_CMD_VERSION:
			uartResp->command = cmd.command;
			uartResp->data_len = sizeof(FIRMWARE_VERSION_DATA);
			uartResp->data = FIRMWARE_VERSION_DATA;
			break;
		case OW_CMD_ECHO:
			// exact copy
			uartResp->id = cmd.id;
			uartResp->packet_type = cmd.packet_type;
			uartResp->command = cmd.command;
			uartResp->data_len = cmd.data_len;
			uartResp->data = cmd.data;
			break;
		case OW_CMD_TOGGLE_LED:
			uartResp->id = cmd.id;
			uartResp->packet_type = cmd.packet_type;
			uartResp->command = cmd.command;
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			break;
		case OW_CMD_HWID:
			uartResp->command = OW_CMD_HWID;
			id_words[0] = HAL_GetUIDw0();
			id_words[1] = HAL_GetUIDw1();
			id_words[2] = HAL_GetUIDw2();
			uartResp->data_len = 16;
			uartResp->data = (uint8_t *)&id_words;
			break;
		case OW_CTRL_SCAN_I2C:
			uartResp->id = cmd.id;
			uartResp->packet_type = cmd.packet_type;
			uartResp->command = cmd.command;
			found_address_count = I2C_scan(found_addresses, MAX_FOUND_ADDRESSES, true);
			uartResp->data_len = found_address_count;
			uartResp->data = found_addresses;
			break;
		case OW_CTRL_START_SWTRIG:
			uartResp->command = cmd.command;
			uartResp->data_len = 0;
			start_trigger_pulse();
			break;
		case OW_CTRL_STOP_SWTRIG:
			uartResp->command = cmd.command;
			uartResp->data_len = 0;
			stop_trigger_pulse();
			break;
		case OW_CTRL_SET_SWTRIG:
			uartResp->command = cmd.command;
			uartResp->data_len = 0;
			if(!set_trigger_data((char *)cmd.data, cmd.data_len))
			{
				uartResp->packet_type = OW_ERROR;
			}
			break;
		case OW_CTRL_GET_SWTRIG:
			// refresh state
			get_trigger_data(retTriggerJson, 256);
			uartResp->command = cmd.command;
			uartResp->data_len = strlen(retTriggerJson);
			uartResp->data = (uint8_t *)retTriggerJson;
			break;
		case OW_CMD_RESET:
			uartResp->command = cmd.command;
			uartResp->data_len = 0;
		    // Reset the board
		    NVIC_SystemReset();
			break;
		default:
			uartResp->data_len = 0;
			uartResp->packet_type = OW_UNKNOWN;
			break;
	}

}

static void JSON_ProcessCommand(UartPacket *uartResp, UartPacket cmd)
{
	// json parser
    jsmn_parser parser;
    parser.size = sizeof(parser);
    jsmn_init(&parser, NULL);
    jsmntok_t t[16];
    jsmnerr_t ret = jsmn_parse(&parser, (char *)cmd.data, cmd.data_len, t,
				 sizeof(t) / sizeof(t[0]), NULL);
    printf("Found %d Tokens\r\n", ret);
	switch (cmd.command)
	{
	case OW_CMD_NOP:
		uartResp->command = OW_CMD_NOP;
		break;
	case OW_CMD_ECHO:
		// exact copy
		uartResp->id = cmd.id;
		uartResp->packet_type = cmd.packet_type;
		uartResp->command = cmd.command;
		uartResp->data_len = cmd.data_len;
		uartResp->data = cmd.data;
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		break;
	}
}


static void print_uart_packet(const UartPacket* packet) {
    printf("ID: 0x%04X\r\n", packet->id);
    printf("Packet Type: 0x%02X\r\n", packet->packet_type);
    printf("Command: 0x%02X\r\n", packet->command);
    printf("Data Length: %d\r\n", packet->data_len);
    printf("CRC: 0x%04X\r\n", packet->crc);
    printf("Data: ");
    for (int i = 0; i < packet->data_len; i++) {
        printf("0x%02X ", packet->data[i]);
    }
    printf("\r\n");
}

UartPacket process_if_command(UartPacket cmd)
{
	UartPacket uartResp;
	I2C_TX_Packet i2c_packet;

	uartResp.id = cmd.id;
	uartResp.packet_type = OW_RESP;
	uartResp.data_len = 0;
	uartResp.data = 0;
	switch (cmd.packet_type)
	{
	case OW_JSON:
		JSON_ProcessCommand(&uartResp, cmd);
		break;
	case OW_CONTROLLER:
		// process by the USTX Controller
		CONTROLLER_ProcessCommand(&uartResp, cmd);
		break;
	case OW_CMD:
		process_basic_command(&uartResp, cmd);
		break;
	case OW_AFE_READ:
		process_afe_read(&uartResp, cmd);
		break;
	case OW_AFE_SEND:
		process_afe_send(&uartResp, cmd);
		break;
	case OW_I2C_PASSTHRU:

		print_uart_packet(&cmd);

        printBuffer(cmd.data, 10);
		i2c_packet_fromBuffer(cmd.data, &i2c_packet);
		i2c_tx_packet_print(&i2c_packet);

		HAL_Delay(20);
		send_buffer_to_slave(cmd.command, cmd.data, 10);

		break;
	default:
		uartResp.data_len = 0;
		uartResp.packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}

	return uartResp;

}

