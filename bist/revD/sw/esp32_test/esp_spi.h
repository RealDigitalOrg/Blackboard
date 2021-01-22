/*
 * esp_spi.h
 *
 *  Created on: Oct 6, 2018
 *      Author: tingh
 */

#ifndef SRC_ESP_SPI_H_
#define SRC_ESP_SPI_H_

#define SPI_ESP32_BUFFER_SIZE		128

int Esp32_SpiInit();
int Esp32_SpiSend(u8 count);
int Esp32_SpiReceive();

#endif /* SRC_ESP_SPI_H_ */
