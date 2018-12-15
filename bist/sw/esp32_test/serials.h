/*
 * serials.h
 *
 *  Created on: Oct 2, 2018
 *      Author: tingh
 */

#ifndef SRC_SERIALS_H_
#define SRC_SERIALS_H_

#define ESP_UART0   0
#define ESP_UART1   1
#define UART_PROG   2

int UartInit();
int UartIsReceiveData(u8 uart);
u8 UartRecvByte(u8 uart);
void UartSendByte(u8 uart, u8 data);

#endif /* SRC_SERIALS_H_ */
