/*
 * gpio_mio.h
 *
 *  Created on: Oct 2, 2018
 *      Author: tingh
 */

#ifndef SRC_GPIO_MIO_H_
#define SRC_GPIO_MIO_H_

int GpioPsInit();
void GpioWriteLD8(u32 r, u32 g, u32 b);
void Esp32Enable(u32 bootMode);
void Esp32Disable();
u32 GpioReadEsp32Hs();
u32 GpioReadBtn4();
u32 GpioReadBtn5();

#endif /* SRC_GPIO_MIO_H_ */
