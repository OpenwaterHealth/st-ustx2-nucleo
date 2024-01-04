/*
 * i2c_func.c
 *
 *  Created on: Jan 4, 2024
 *      Author: gvigelet
 */

#include "i2c_func.h"

extern I2C_HandleTypeDef hi2c1;
uint8_t selected_slave = 0xFF;

uint8_t I2C_scan() {
    printf("Scanning I2C bus for devices...\r\n");
    HAL_GPIO_WritePin(nRST_SLV_GPIO_Port, nRST_SLV_Pin, GPIO_PIN_RESET);
    osDelay(10);
    HAL_GPIO_WritePin(nRST_SLV_GPIO_Port, nRST_SLV_Pin, GPIO_PIN_SET);
    osDelay(15);
    uint8_t bFound = 0;
    for (uint8_t address = 0x0; address < 0x7f; address++) {
        HAL_StatusTypeDef status;
        status = HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 2, 2); // Address shift left by 1 for read/write bit
        if (status == HAL_OK) {
        	printf("%2x ", address);
        	if(bFound == 0){
        		selected_slave = address;
        	}
        }else{
        	printf("-- ");
        }
        if (address > 0 && (address + 1) % 16 == 0) printf("\r\n");
    }

    printf("\r\n");
    fflush(stdout);
    return selected_slave;
}

uint8_t I2C_get_selected_slave(){
	return selected_slave;
}

void I2C_set_selected_slave(uint8_t slave_addr){
	selected_slave = slave_addr;
}
