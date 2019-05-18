/*********************************************************************************
 * File: gpio.h
 * Author: Tinghui Wang
 *
 * Copyright @ 2019 RealDigital.org
 *
 * Description:
 *   Header file for GPIO PS configuration.
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
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/
#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_

#include "xparameters.h"
#include <xgpiops_hw.h>

#define GPIOPS_DEVICE_ID XPAR_XGPIOPS_0_DEVICE_ID

#define LD8_B_OFFSET          16
#define LD8_R_OFFSET          17
#define LD8_G_OFFSET          18

#define ESP32_GPIO25_OFFSET   23
#define ESP32_BOOT0_OFFSET    25
#define ESP32_EN_OFFSET       26
#define ESP32_GPIO26_OFFSET   53

#define BTN4_OFFSET           50
#define BTN5_OFFSET           51

#define EMIO_OFFSET    54
#define JA1_N_OFFSET   (EMIO_OFFSET + 0)
#define JA2_P_OFFSET   (EMIO_OFFSET + 1)
#define JA2_N_OFFSET   (EMIO_OFFSET + 2)
#define JA3_P_OFFSET   (EMIO_OFFSET + 3)
#define JA3_N_OFFSET   (EMIO_OFFSET + 4)
#define JA4_P_OFFSET   (EMIO_OFFSET + 5)
#define JA4_N_OFFSET   (EMIO_OFFSET + 6)

#define JB1_P_OFFSET   (EMIO_OFFSET + 7)
#define JB1_N_OFFSET   (EMIO_OFFSET + 8)
#define JB2_P_OFFSET   (EMIO_OFFSET + 9)
#define JB2_N_OFFSET   (EMIO_OFFSET + 10)
#define JB3_P_OFFSET   (EMIO_OFFSET + 11)
#define JB3_N_OFFSET   (EMIO_OFFSET + 12)
#define JB4_P_OFFSET   (EMIO_OFFSET + 13)
#define JB4_N_OFFSET   (EMIO_OFFSET + 14)

#define JC1_OFFSET     (EMIO_OFFSET + 15)
#define JC2_OFFSET     (EMIO_OFFSET + 16)
#define JC3_OFFSET     (EMIO_OFFSET + 17)
#define JC4_OFFSET     (EMIO_OFFSET + 18)
#define JC7_OFFSET     (EMIO_OFFSET + 19)
#define JC8_OFFSET     (EMIO_OFFSET + 20)
#define JC9_OFFSET     (EMIO_OFFSET + 21)
#define JC10_OFFSET    (EMIO_OFFSET + 22)

#define LSM9DS1_AGDEN_OFFSET (EMIO_OFFSET + 23)

void GpioPs_Init();

#endif /* SRC_GPIO_H_ */
