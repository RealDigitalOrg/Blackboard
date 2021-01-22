/*
 * gpio_mio.c
 *
 *  Created on: Oct 2, 2018
 *      Author: tingh
 */

#include "xparameters.h"
#include "xgpiops.h"
#include <xil_exception.h>
#include <xil_printf.h>

static XGpioPs gpioPs;

int GpioPsInit() {
	XGpioPs_Config *pConfig;
	int status;

	/* Initialize GPIO Driver */
	pConfig = XGpioPs_LookupConfig(XPAR_PS7_GPIO_0_DEVICE_ID);
	if(pConfig == NULL) {
		return XST_FAILURE;
	}
	XGpioPs_CfgInitialize(&gpioPs, pConfig, pConfig->BaseAddr);

	/* Optional self test */
	status = XGpioPs_SelfTest(&gpioPs);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Set Direction for specified pin to be input or output */
	// MIO16 LD8_B output
	XGpioPs_SetDirectionPin(&gpioPs, 16, 1);
	XGpioPs_SetOutputEnablePin(&gpioPs, 16, 1);
	XGpioPs_WritePin(&gpioPs, 16, 0);
	// MIO17 LD8_R output
	XGpioPs_SetDirectionPin(&gpioPs, 17, 1);
	XGpioPs_SetOutputEnablePin(&gpioPs, 17, 1);
	XGpioPs_WritePin(&gpioPs, 17, 0);
	// MIO18 LD8_G output
	XGpioPs_SetDirectionPin(&gpioPs, 18, 1);
	XGpioPs_SetOutputEnablePin(&gpioPs, 18, 1);
	XGpioPs_WritePin(&gpioPs, 18, 0);

	// MIO25 ESP32_BOOT0 output
	XGpioPs_SetDirectionPin(&gpioPs, 25, 1);
	XGpioPs_SetOutputEnablePin(&gpioPs, 25, 1);
	XGpioPs_WritePin(&gpioPs, 25, 0);

	// MIO26 ESP32_EN output
	XGpioPs_SetDirectionPin(&gpioPs, 26, 1);
	XGpioPs_SetOutputEnablePin(&gpioPs, 26, 1);
	XGpioPs_WritePin(&gpioPs, 26, 0);

	// MIO50 BTN4 input
	XGpioPs_SetDirectionPin(&gpioPs, 50, 0);

	// MIO51 BTN5 input
	XGpioPs_SetDirectionPin(&gpioPs, 51, 0);

	// MIO24 TEMP_OS input (over temperature shutdown)
	XGpioPs_SetDirectionPin(&gpioPs, 24, 0);

	// MIO9 ESP32_GPIO34 (Used for VSPI Handshake)
	// RevC Temporary Fix: Rewired to EMIO[0] (pinID: 54)
	//	XGpioPs_SetDirectionPin(&gpioPs, 54, 1);
	//	XGpioPs_SetOutputEnablePin(&gpioPs, 54, 1);
	//	XGpioPs_WritePin(&gpioPs, 54, 0);
	XGpioPs_SetDirectionPin(&gpioPs, 54, 0);

	return XST_SUCCESS;
}

void GpioWriteLD8(u32 r, u32 g, u32 b) {
	XGpioPs_WritePin(&gpioPs, 17, r);
	XGpioPs_WritePin(&gpioPs, 18, g);
	XGpioPs_WritePin(&gpioPs, 16, b);
}

void Esp32Enable(u32 bootMode) {
	// Set Boot0 pin value for ESP32
	XGpioPs_WritePin(&gpioPs, 25, bootMode);
	// Set Enable pin to high
	XGpioPs_WritePin(&gpioPs, 26, 1);
}

void Esp32Disable() {
	// Set Enable pin to low
	XGpioPs_WritePin(&gpioPs, 26, 0);
	// Always set Boot0 pin to high (user mode) after shutting down.
	XGpioPs_WritePin(&gpioPs, 25, 0);
}

u32 GpioReadEsp32Hs() {
	return XGpioPs_ReadPin(&gpioPs, 54);
}

u32 GpioReadBtn4() {
	return XGpioPs_ReadPin(&gpioPs, 50);
}

u32 GpioReadBtn5() {
	return XGpioPs_ReadPin(&gpioPs, 51);
}
