/*
 * commands.c
 *
 *  Created on: Jan 3, 2024
 *      Author: gvigelet
 */

#include "commands.h"
#include "i2c_func.h"
#include <stdio.h>


void print_main_menu(uint8_t slave_address)
{
    // Print the menu
    printf("\r\nMain Menu (Selected Slave: 0x%02X):\r\n\r\n", slave_address);
    printf("1. Test I2C\r\n");
    printf("2. Toggle LED\r\n");

    printf("\r\ns. Select Slave (rescans)\r\n");
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
            printf("Testing I2C...\r\n");
            break;

        case '2':
            // Toggle LED
            printf("Toggling LED...\r\n");
            break;

        case 's':
            // Select Slave
            printf("Select Slave...\r\n");
            I2C_scan();
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
