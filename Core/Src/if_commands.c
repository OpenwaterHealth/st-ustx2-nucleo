/*
 * if_commands.c
 *
 *  Created on: Mar 12, 2024
 *      Author: gvigelet
 */

#include "main.h"
#include "if_commands.h"
#include "uart_comms.h"
#include "common.h"
#include "cJSON.h"

#include <stdio.h>
#include <string.h>

static uint8_t FIRMWARE_VERSION_DATA[3] = {0, 1, 1};
static uint32_t id_words[3] = {0};

/* assertion helper macros */
#define assert_has_type(item, item_type) TEST_ASSERT_BITS_MESSAGE(0xFF, item_type, item->type, "Item doesn't have expected type.")
#define assert_has_no_reference(item) TEST_ASSERT_BITS_MESSAGE(cJSON_IsReference, 0, item->type, "Item should not have a string as reference.")
#define assert_has_no_const_string(item) TEST_ASSERT_BITS_MESSAGE(cJSON_StringIsConst, 0, item->type, "Item should not have a const string.")
#define assert_has_valuestring(item) TEST_ASSERT_NOT_NULL_MESSAGE(item->valuestring, "Valuestring is NULL.")
#define assert_has_no_valuestring(item) TEST_ASSERT_NULL_MESSAGE(item->valuestring, "Valuestring is not NULL.")
#define assert_has_string(item) TEST_ASSERT_NOT_NULL_MESSAGE(item->string, "String is NULL")
#define assert_has_no_string(item) TEST_ASSERT_NULL_MESSAGE(item->string, "String is not NULL.")
#define assert_not_in_list(item)                                                   \
	TEST_ASSERT_NULL_MESSAGE(item->next, "Linked list next pointer is not NULL."); \
	TEST_ASSERT_NULL_MESSAGE(item->prev, "Linked list previous pointer is not NULL.")
#define assert_has_child(item) TEST_ASSERT_NOT_NULL_MESSAGE(item->child, "Item doesn't have a child.")
#define assert_has_no_child(item) TEST_ASSERT_NULL_MESSAGE(item->child, "Item has a child.")
#define assert_is_invalid(item)           \
	assert_has_type(item, cJSON_Invalid); \
	assert_not_in_list(item);             \
	assert_has_no_child(item);            \
	assert_has_no_string(item);           \
	assert_has_no_valuestring(item)

static void process_basic_command(UartPacket *uartResp, UartPacket cmd)
{
	switch (cmd.command)
	{
	case USTX_NOP:
		uartResp->command = USTX_NOP;
		break;
	case USTX_PING:
		uartResp->command = USTX_PONG;
		break;
	case USTX_PONG:
		uartResp->command = USTX_PING;
		break;
	case USTX_VERSION:
		uartResp->command = USTX_VERSION;
		uartResp->data_len = sizeof(FIRMWARE_VERSION_DATA);
		uartResp->data = FIRMWARE_VERSION_DATA;
		break;
	case USTX_ID:
		uartResp->command = USTX_ID;
		id_words[0] = HAL_GetUIDw0();
		id_words[1] = HAL_GetUIDw1();
		id_words[2] = HAL_GetUIDw2();
		uartResp->data_len = 16;
		uartResp->data = (uint8_t *)&id_words;
		break;
	case USTX_ECHO:
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
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}
}

static char retTriggerJson[256];
static uint32_t TRIGGER_ProcessCommand(UartPacket *uartResp, UartPacket cmd)
{
	switch (cmd.command)
	{

	case CMD_GET_SWTRIG:
		// refresh state
		get_trigger_data(retTriggerJson, 256);
		uartResp->command = cmd.command;
		uartResp->data_len = strlen(retTriggerJson);
		uartResp->data = (uint8_t *)retTriggerJson;
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		break;
	}

}

static void JSON_ProcessCommand(UartPacket *uartResp, UartPacket cmd, cJSON *root)
{
	switch (cmd.command)
	{
	case USTX_NOP:
		uartResp->command = USTX_NOP;
		break;
	case USTX_ECHO:
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

UartPacket process_if_command(UartPacket cmd)
{
	UartPacket uartResp;
	cJSON *root = NULL;

	uartResp.id = cmd.id;
	uartResp.packet_type = OW_RESP;
	uartResp.data_len = 0;
	uartResp.data = 0;

	switch (cmd.packet_type)
	{
	case OW_JSON:
		// Parse the received data with cJSON
		root = cJSON_Parse((const char *)cmd.data);
		if (root == NULL)
		{
			// Handle parsing error
			uartResp.packet_type = OW_BAD_PARSE;
			return uartResp;
		}
		else
		{
			JSON_ProcessCommand(&uartResp, cmd, root);
		}
		break;
	case OW_TRIGGER:
		// process by the TX7332 Driver
		TRIGGER_ProcessCommand(&uartResp, cmd);
		break;
	case OW_CMD:
		process_basic_command(&uartResp, cmd);
		break;
	default:
		uartResp.data_len = 0;
		uartResp.packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}

	return uartResp;

}

