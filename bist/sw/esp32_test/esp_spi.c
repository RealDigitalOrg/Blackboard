/*
 * esp_spi.c
 *
 *  Created on: Oct 6, 2018
 *      Author: tingh
 */

#include "xparameters.h"
#include "xspips.h"
#include "esp_spi.h"
#include "gpio_mio.h"

XSpiPs spiEsp32;
u8 pchSpiEsp32SendBuffer[SPI_ESP32_BUFFER_SIZE];
u8 pchSpiEsp32RecvBuffer[SPI_ESP32_BUFFER_SIZE];

int Esp32_SpiInit() {
	XSpiPs_Config *pConfig;
	int status;

	pConfig = XSpiPs_LookupConfig(XPAR_PS7_SPI_1_DEVICE_ID);
	if(pConfig == NULL) {
		return XST_FAILURE;
	}

	status = XSpiPs_CfgInitialize(&spiEsp32, pConfig, pConfig->BaseAddress);
	if(status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XSpiPs_SetOptions(&spiEsp32, XSPIPS_MASTER_OPTION | XSPIPS_MANUAL_START_OPTION | XSPIPS_FORCE_SSELECT_OPTION);
	XSpiPs_SetDelays(&spiEsp32, 1, 1, 1, 1);
	// SPI is clocked by 200MHz clock. By setting the clock prescale to '8', the spi mclk speed is reduced to 25MHz.
	XSpiPs_SetClkPrescaler(&spiEsp32, XSPIPS_CLK_PRESCALE_256);

	return XST_SUCCESS;
}

int Esp32_SpiSend(u8 count) {
	if(GpioReadEsp32Hs() == 0) {
		return XSpiPs_PolledTransfer(&spiEsp32, pchSpiEsp32SendBuffer, pchSpiEsp32RecvBuffer, count);
	} else {
		return XST_FAILURE;
	}
}

int Esp32_SpiReceive() {
	u32 count = 0;
	int status;

	while(GpioReadEsp32Hs() == 0);

	while(GpioReadEsp32Hs() == 1 && count < SPI_ESP32_BUFFER_SIZE) {
		status = XSpiPs_PolledTransfer(&spiEsp32, pchSpiEsp32SendBuffer, pchSpiEsp32RecvBuffer + count, 1);
		if(status == XST_SUCCESS) {
			count += 1;
		}
	}

	return count;
}
