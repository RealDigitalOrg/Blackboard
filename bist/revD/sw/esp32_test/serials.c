/*
 * uart_forward.c
 *
 *  Created on: Oct 2, 2018
 *      Author: tingh
 */

#include <xil_types.h>
#include "xparameters.h"
#include "xuartps.h"
#include "xuartps_hw.h"
#include "xuartlite.h"
#include "xuartlite_i.h"
#include "xuartlite_l.h"
#include "serials.h"

XUartPs uartProg;
XUartPs uartEsp32_0;
XUartLite uartEsp32_1;

int UartInit() {
	XUartPs_Config *pConfig_prog;
	XUartPs_Config *pConfig_0;
	XUartLite_Config *pConfig_1;
	int status;

	/* Initialize PS7_UART1 for usb prog port */
	pConfig_prog = XUartPs_LookupConfig(XPAR_PS7_UART_1_DEVICE_ID);
	if (pConfig_0 == NULL) {
		return XST_FAILURE;
	}

	status = XUartPs_CfgInitialize(&uartProg, pConfig_prog, pConfig_prog->BaseAddress);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	XUartPs_SetBaudRate(&uartProg, 115200UL);
	XUartPs_SetOperMode(&uartProg, XUARTPS_OPER_MODE_NORMAL);
	XUartPs_EnableUart(&uartProg);

	/* Initialize PS7_UART0 for ESP32 UART0 */
	pConfig_0 = XUartPs_LookupConfig(XPAR_PS7_UART_0_DEVICE_ID);
	if (pConfig_0 == NULL) {
		return XST_FAILURE;
	}

	status = XUartPs_CfgInitialize(&uartEsp32_0, pConfig_0, pConfig_0->BaseAddress);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XUartPs_SetBaudRate(&uartEsp32_0, 115200UL);
	XUartPs_SetOperMode(&uartEsp32_0, XUARTPS_OPER_MODE_NORMAL);
	XUartPs_EnableUart(&uartEsp32_0);

	/* Initialize PS7_UART1 for ESP32 UART1 */
	status = XUartLite_Initialize(&uartEsp32_1, XPAR_ESP32_UART1_DEVICE_ID);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

int UartIsReceiveData(u8 uart) {
	switch(uart) {
	case ESP_UART0:
		return XUartPs_IsReceiveData(uartEsp32_0.Config.BaseAddress);
	case ESP_UART1:
		return !XUartLite_IsReceiveEmpty(uartEsp32_1.RegBaseAddress);
	case UART_PROG:
		return XUartPs_IsReceiveData(uartProg.Config.BaseAddress);
	default:
		return FALSE;
	}
}

u8 UartRecvByte(u8 uart) {
	switch(uart) {
	case ESP_UART0:
		return XUartPs_RecvByte(uartEsp32_0.Config.BaseAddress);
	case ESP_UART1:
		return XUartLite_RecvByte(uartEsp32_1.RegBaseAddress);
	case UART_PROG:
		return XUartPs_RecvByte(uartProg.Config.BaseAddress);
	default:
		return 0;
	}
}

void UartSendByte(u8 uart, u8 data) {
	switch(uart) {
	case ESP_UART0:
		XUartPs_SendByte(uartEsp32_0.Config.BaseAddress, data);
		break;
	case ESP_UART1:
		XUartLite_SendByte(uartEsp32_1.RegBaseAddress, data);
		break;
	case UART_PROG:
		XUartPs_SendByte(uartProg.Config.BaseAddress, data);
		break;
	default:
		break;
	}
}
