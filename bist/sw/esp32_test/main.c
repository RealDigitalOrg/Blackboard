
#include "xuartps_hw.h"
#include "gpio_mio.h"
#include "serials.h"
#include "esp_spi.h"

void delay(u32 ms) {
	volatile u32 i = 0;
	for(i = 0; i < ms; i++) {
		asm("nop");
	}
}

unsigned int esp32_uart_forward = 2;
unsigned int esp32_boot_mode = 1;
extern u8 pchSpiEsp32SendBuffer[SPI_ESP32_BUFFER_SIZE];
extern u8 pchSpiEsp32RecvBuffer[SPI_ESP32_BUFFER_SIZE];
unsigned int uSpiEsp32SendCount = 0;

int main() {

	u8 chRecv;

	GpioPsInit();
	UartInit();
	Esp32_SpiInit();
	xil_printf("Init System.\r\n");
	Esp32Disable();
	delay(500000000UL);
	xil_printf("ESP32 Uart 0 Forward.\r\n");
	Esp32Enable(esp32_boot_mode);

	while(1) {
		if(UartIsReceiveData(UART_PROG)) {
			chRecv = UartRecvByte(UART_PROG);
			if(esp32_uart_forward == 0) {
				UartSendByte(ESP_UART0, chRecv);
			} else if(esp32_uart_forward == 1) {
				UartSendByte(ESP_UART1, chRecv);
			} else {
				if(chRecv == '\n' && pchSpiEsp32SendBuffer[uSpiEsp32SendCount - 1] == '\r') {
					UartSendByte(UART_PROG, chRecv);
					pchSpiEsp32SendBuffer[uSpiEsp32SendCount] = chRecv;
					uSpiEsp32SendCount += 1;
					Esp32_SpiSend(uSpiEsp32SendCount);
					uSpiEsp32SendCount = 0;
				} else if(uSpiEsp32SendCount < SPI_ESP32_BUFFER_SIZE - 2 ||
						(chRecv == '\r' && uSpiEsp32SendCount < SPI_ESP32_BUFFER_SIZE - 1))  {
					UartSendByte(UART_PROG, chRecv);
					pchSpiEsp32SendBuffer[uSpiEsp32SendCount] = chRecv;
					uSpiEsp32SendCount += 1;
				}
			}
		}

		if(esp32_uart_forward == 0) {
			if(UartIsReceiveData(ESP_UART0)) {
				chRecv = UartRecvByte(ESP_UART0);
				UartSendByte(UART_PROG, chRecv);
			}
		} else if(esp32_uart_forward == 1) {
			if(UartIsReceiveData(ESP_UART0)) {
				chRecv = UartRecvByte(ESP_UART0);
				UartSendByte(UART_PROG, chRecv);
			}
			if(UartIsReceiveData(ESP_UART1)) {
				chRecv = UartRecvByte(ESP_UART1);
				UartSendByte(UART_PROG, chRecv);
			}
		} else {
			if(GpioReadEsp32Hs() != 0) {
				u32 iCharsReceived = Esp32_SpiReceive();
				for(u32 i = 0; i < iCharsReceived; i++) {
					UartSendByte(UART_PROG, pchSpiEsp32RecvBuffer[i]);
				}
			}
			if(UartIsReceiveData(ESP_UART0)) {
				chRecv = UartRecvByte(ESP_UART0);
				UartSendByte(UART_PROG, chRecv);
			}
		}

		if(GpioReadBtn4() != 0) {
			xil_printf("Reset ESP32.\r\n");
			Esp32Disable();
			delay(500000000UL);
			esp32_boot_mode = (esp32_boot_mode) ? 0 : 1;
			xil_printf("Boot ESP32 with mode %d.\r\n", esp32_boot_mode);
			Esp32Enable(esp32_boot_mode);
		}

		if(GpioReadBtn5() != 0) {
			esp32_uart_forward += 1;
			if(esp32_uart_forward == 3) {
				esp32_uart_forward = 0;
			}
			if(esp32_uart_forward == 2) {
				xil_printf("Forwarding UART to ESP32 VSPI...");
				uSpiEsp32SendCount = 0;
			} else {
				xil_printf("Forwarding UART to ESP32 UART %d...", esp32_uart_forward);
			}
			delay(100000000UL);
			xil_printf("done\r\n");
		}
	}
}
