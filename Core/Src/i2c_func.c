/*
 * i2c_func.c
 *
 *  Created on: Jan 4, 2024
 *      Author: gvigelet
 */
#include "main.h"
#include "i2c_func.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
uint8_t selected_slave = 0xFF;

void reset_slaves()
{
    printf("Scanning I2C bus for devices...\r\n");
    HAL_GPIO_WritePin(nRST_SLV_GPIO_Port, nRST_SLV_Pin, GPIO_PIN_RESET);
    osDelay(10);
    HAL_GPIO_WritePin(nRST_SLV_GPIO_Port, nRST_SLV_Pin, GPIO_PIN_SET);
    osDelay(50);
}

void I2C_scan() {

    // Reset the global array and counter
    memset(found_addresses, 0, MAX_FOUND_ADDRESSES );
    found_address_count = 0;

    for (uint8_t address = 0x0; address < 0x7f; address++) {
        HAL_StatusTypeDef status;
        status = HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 2, 200); // Address shift left by 1 for read/write bit
        if (status == HAL_OK) {
            found_addresses[found_address_count] = address;
            found_address_count++;
        	printf("%2x ", address);
        }else{
        	printf("-- ");
        }
        if (address > 0 && (address + 1) % 16 == 0) printf("\r\n");
    }

    printf("\r\n");
    printf("\r\nFound %d I2C devices:\r\n", found_address_count);
    for (int i = 0; i < found_address_count; i++) {
        printf("Address %d: 0x%02X\r\n", i + 1, found_addresses[i]);
    }
    fflush(stdout);
}

uint8_t send_packet_to_slave(uint8_t slave_addr, uint8_t* pData, uint8_t len)
{
	// Check if the I2C handle is valid
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    	printf("===> ERROR I2C Not in ready state\r\n");
        return 1; // I2C is not in a ready state
    }

    if(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(slave_addr << 1), pData, len, HAL_MAX_DELAY)!= HAL_OK)
	{
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
	}
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    	printf("===> ERROR I2C Not in ready state\r\n");
        return 1; // I2C is not in a ready state
    }

#ifdef DEBUG_COMMS
	printf("Send packet complete\r\n");
#endif

	return 0;
}

uint8_t I2C_get_selected_slave(){
	return selected_slave;
}

void I2C_set_selected_slave(uint8_t slave_addr){
	selected_slave = slave_addr;
}

static int findSelectedSlaveIndex() {
    for (int i = 0; i < found_address_count; i++) {
        if (found_addresses[i] == selected_slave) {
            return i; // Return the index of the selected slave
        }
    }
    return -1; // Selected slave not found in the array
}

void I2C_set_next_slave()
{
	if(found_address_count > 0){
		int selectedIndex = findSelectedSlaveIndex();
		if (selectedIndex == -1) {
			if (found_address_count > 0) {
				selected_slave = found_addresses[0];
			} else {
				selected_slave = 0xFF; // No addresses found, select address 0 as default
			}
		} else {
	        selectedIndex++;
	        if (selectedIndex >= found_address_count) {
	            selectedIndex = 0; // Loop back to the beginning
	        }
	        selected_slave = found_addresses[selectedIndex];
		}
	}else{
		selected_slave = 0xFF;
	}
}
