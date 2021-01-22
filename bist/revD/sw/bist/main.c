/*********************************************************************************
 * File: main.c
 * Author: Tinghui Wang
 *
 * Copyright @ 2019 RealDigital.org
 *
 * Description:
 *   Built-In Self Test Main Entry
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

#include <xil_types.h>
#include <xil_printf.h>
#include <xil_io.h>
#include <stdio.h>
#include <stdlib.h>
#include "gpio.h"
#include "intr.h"
#include "uart.h"
#include "rgb_pwm.h"
#include "seven_seg_display.h"
#include "lsm9ds1.h"

char test = '\0';
char cmd = '\0';
uint32_t loopCount = 0;
uint32_t loopDelay = 0;

void help() {
	xil_printf("\r\nMenu\r\n");
	xil_printf("a - RGB LED Test\r\n");
	xil_printf("b - Seven Segment Test\r\n");
	xil_printf("c - PDM Audio Test\r\n");
	xil_printf("d - Switch/LED Test \r\n");
	xil_printf("e - LSM9DS1 Test \r\n");
	xil_printf("s - Stop Test\r\n");
}

void StartTest(char c) {
	switch(c) {
	case 'a':
		RGBPWM_StartTest();
		break;
	case 'b':
		SSD_StartTest();
		break;
	case 'c':
		Audio_StartTest();
		test = '\0';
		break;
	case 'e':
		LSM9DS1_TestStart();
		break;
	default:
		break;
	}
}

void StopTest(char c) {
	switch(c) {
	case 'a':
		RGBPWM_StopTest();
		break;
	case 'b':
		SSD_StopTest();
		break;
	case 'e':
		LSM9DS1_TestStop();
		break;
	default:
		break;
	}
}

uint32_t LoopTest(char c) {
	uint32_t delay = 0;
	switch(c) {
	case 'a':
		delay = RGBPWM_TestLoop();
		break;
	case 'b':
		delay = SSD_TestLoop();
		break;
	case 'e':
		delay = LSM9DS1_TestLoop();
		break;
	default:
		break;
	}
	return delay;
}

uint32_t ValidateCmd(char c) {
	switch(c) {
	case 'a':
	case 'b':
	case 'c':
	case 'd':
	case 'e':
	case 's':
		return 1;
	default:
		return 0;
	}
}

int main() {
	GicInit();
	Uart_Init();
	GpioPs_Init();
	xil_printf("BlackBoard Rev. D Built-In Self Test Application\r\n");
	xil_printf("\r\n");
	help();

	while(1) {
		if(Uart_IsReceiveData()) {
			cmd = Uart_RecvChar();
			xil_printf("%c\r\n", cmd);
			if(ValidateCmd(cmd)) {
				StopTest(test);
				test = cmd;
				StartTest(test);
				loopCount = 0;
			}
			help();
		}
		if(loopCount == 0) {
			loopDelay = LoopTest(test);
		}
		loopCount = (loopCount == loopDelay) ? 0 : loopCount + 1;
	}

	return 0;
}
