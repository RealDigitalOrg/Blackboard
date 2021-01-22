/*********************************************************************************
 * File: lsm9ds1.h
 * Author: Tinghui Wang
 *
 * Copyright @ 2019 RealDigital.org
 *
 * Description:
 *   Header for LSM9DS1 sensor test routine.
 *
 * History:
 *   03/30/19: Created
 *
 * License: BSD 3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDIN	G NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/
#ifndef SRC_LSM9DS1_H_
#define SRC_LSM9DS1_H_

#include "xparameters.h"
#include <xil_types.h>
#include <xgpiops.h>
#include "lsm9ds1_regs.h"
#include <xscugic.h>

#define LSM9DS1_SPI_DEVICE_ID XPAR_XSPIPS_0_DEVICE_ID

#define LSM9DS1_RDRY_M  87U
#define LSM9DS1_INT1_AG 88U
#define LSM9DS1_INT_M   89U

extern XGpioPs gpioPsInst;
extern XScuGic gicInst;

typedef struct gyroConfig {
	uint16_t scale;
	uint8_t sampleRate;
} GyroConfig_t;

void LSM9DS1_SpiReadBytes(uint8_t address, uint8_t *buffer, uint8_t length);
void LSM9DS1_SpiSendBytes(uint8_t address, uint8_t *buffer, uint8_t length);
uint8_t LSM9DS1_SpiReadByte(uint8_t address);
void LSM9DS1_SpiWriteByte(uint8_t address, uint8_t value);
void LSM9DS1_SpiInit();

void LSM9DS1_SelectAG();
void LSM9DS1_SelectM();
void LSM9DS1_SetAGDen(uint8_t value);

void LSM9DS1_ReadXL(uint16_t *buffer);
void LSM9DS1_ReadG(uint16_t *buffer);
void LSM9DS1_ReadM(uint16_t *buffer);
void LSM9DS1_PrintXL(int16_t *buffer);
void LSM9DS1_PrintG(int16_t *buffer);
void LSM9DS1_PrintM(int16_t *buffer);

void LSM9DS1_EnableINT();
void LSM9DS1_DisableINT();

void LSM9DS1_INT1_AG_Handler(void *data);
void LSM9DS1_RDRY_M_Handler(void *data);

void LSM9DS1_TestStart();
void LSM9DS1_TestStop();
uint32_t LSM9DS1_TestLoop();

#endif /* SRC_LSM9DS1_H_ */
