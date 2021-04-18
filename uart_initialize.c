/*
 * uart_initialize.c
 *
 *  Created on: Mar 24, 2021
 *      Author: Shyama Gandhi
 */


#include "xuartps.h" 										//UART definitions header file
#define UART_DEVICE_ID 		XPAR_XUARTPS_0_DEVICE_ID 		//UART Device ID

XUartPs UART; 				//UART Instance
XUartPs_Config *Config; 	//Pointer to UART


int Initialize_UART(){

	int Status;

	Config = XUartPs_LookupConfig(UART_DEVICE_ID);
	if (NULL == Config)
	{
	return XST_FAILURE;
	xil_printf("UART PS Config failed\n");
	}

	//Initialize UART
	Status = XUartPs_CfgInitialize(&UART, Config, Config->BaseAddress);
	if (Status != XST_SUCCESS)
	{
	return XST_FAILURE;
	xil_printf("UART PS init failed\n");
	}

	return XST_SUCCESS;
}

