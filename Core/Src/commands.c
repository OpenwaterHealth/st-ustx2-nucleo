/*
 * commands.c
 *
 *  Created on: Jan 3, 2024
 *      Author: gvigelet
 */

#include "commands.h"
#include <stdio.h>


void print_main_menu()
{
    // Print the menu
    printf("Main Menu:\r\n\r\n");
    printf("1. Test I2C\r\n");
    printf("2. Toggle LED\r\n");

    printf("\r\n?. This Menu\r\n");
}

void print_prompt(){
    printf("\r\nChoice: ");
    fflush(stdout);
}

void process_command(char* input){

    printf("\r\n");
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

        case '?':
            // Toggle LED
        	printf("\033c");
        	print_main_menu();
            break;

        default:
            printf("\r\nInvalid choice. Please try again.\r\n\r\n");
            break;
    }
}
