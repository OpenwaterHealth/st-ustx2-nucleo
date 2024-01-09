/*
 * commands.c
 *
 *  Created on: Jan 3, 2024
 *      Author: gvigelet
 */

#include "commands.h"
#include "i2c_func.h"
#include "i2c_protocol.h"

#include <stdio.h>


void print_main_menu(uint8_t slave_address)
{
    // Print the menu
    printf("\r\nMain Menu (Selected Slave: 0x%02X):\r\n\r\n", slave_address);
    printf("1. Toggle Master\r\n");
    printf("2. Toggle Slave LED\r\n");

    printf("\r\ns. Scan I2C Bus\r\n");
    printf("\r\nn. Select Next Slave\r\n");
    printf("?. This Menu\r\n");
}

void print_prompt(){
    printf("\r\nChoice: ");
    fflush(stdout);
}

void process_command(char* input, uint8_t selected_slave){

    printf("\r\n");
    if(selected_slave == 0xFF && *input != 's'){
        printf("\r\nNo Slave Selected. Please Select Slave first.\r\n\r\n");
        return;
    }
    switch (*input) {
        case '1':
            // Perform Test I2C functionality
            // You should implement the I2C test functionality here
            printf("Toggling Master LED...\r\n");
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            break;

        case '2':
            // Toggle LED
            printf("Toggling Slave 0x%02X LED...\r\n", selected_slave);

            SendI2CPacket(selected_slave, CMD_TOGGLE_LED);
            break;

        case 's':
            // Select Slave
            printf("Scan I2C Bus...\r\n");
            I2C_scan();
            break;

        case 'n':
            // Select Slave
            printf("Select Next Slave Total # %d CURRENT: 0x%02X ", found_address_count, selected_slave);
            I2C_set_next_slave();
            printf("NEW: 0x%02X\r\n", I2C_get_selected_slave());
            break;

        case '?':
            // Show Menu
        	printf("\033c");
        	print_main_menu(selected_slave);
            break;

        default:
            printf("\r\nInvalid choice. Please try again.\r\n\r\n");
            break;
    }
}
