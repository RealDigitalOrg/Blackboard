/*********************************************************************************
 * File: rgb_pwm.c
 * Author: Tinghui Wang
 *
 * Copyright @ 2019 RealDigital.org
 *
 * Description:
 *   Example test for AXI PWM controller. In this example, the AXI PWM controller
 *   outputs are connected to RGB Leds LD10 and LD11 on BlackBoard Rev. D.
 *   - PWM Channel 0 - LD10 Blue
 *   - PWM Channel 1 - LD10 Green
 *   - PWM Channel 2 - LD10 Red
 *   - PWM Channel 3 - LD11 Blue
 *   - PWM Channel 4 - LD11 Green
 *   - PWM Channel 5 - LD11 Red
 *
 * History:
 *   11/13/17: Created
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

#include "rgb_pwm.h"
#include <xil_io.h>

/**
 * Enable PWM Channel
 */
void PWM_EnableChannel(uint8_t channel) {
    Xil_Out32(PWM_BASEADDR + 16 * channel, 0x01UL);
}

/**
 * Disable PWM Channel
 */
void PWM_DisableChannel(uint8_t channel) {
    Xil_Out32(PWM_BASEADDR + 16 * channel, 0x00UL);
}

/**
 * Configure PWM Channel with period and pulse width
 */
void PWM_ConfigureChannel(uint8_t channel, uint32_t period, uint32_t pulse) {
    Xil_Out32(PWM_BASEADDR + 16 * channel + 4, period);
    Xil_Out32(PWM_BASEADDR + 16 * channel + 8, pulse);
}

/**
 * RGB Set Color
 */
void rgbSetColor(
    uint8_t r_channel, uint8_t g_channel, uint8_t b_channel, uint32_t* rgbVal
) {
    PWM_ConfigureChannel(r_channel, PWM_PERIOD, (PWM_PERIOD * rgbVal[0]) / 256);
    PWM_ConfigureChannel(g_channel, PWM_PERIOD, (PWM_PERIOD * rgbVal[1]) / 256);
    PWM_ConfigureChannel(b_channel, PWM_PERIOD, (PWM_PERIOD * rgbVal[2]) / 256);
}

uint32_t rgbColor[3];
int decColor = 0;

/**
 * Setup RGB PWM Test
 */
void RGBPWM_StartTest() {
    int i = 0;

    rgbColor[0] = 64;
    rgbColor[1] = 0;
    rgbColor[2] = 0;

    for(i = 0; i < 6; i++) {
        PWM_ConfigureChannel(i, PWM_PERIOD, PWM_PERIOD/8);
    }
    for(i = 0; i < 6; i++) {
        PWM_EnableChannel(i);
    }
}

/**
 * RGB PWM Loop
 */
uint32_t RGBPWM_TestLoop() {
	if(rgbColor[decColor] == 0) {
		decColor = (decColor == 2) ? 0 : decColor + 1;
	}
	int incColor = (decColor == 2) ? 0 : decColor + 1;
	rgbColor[decColor] -= 1;
	rgbColor[incColor] += 1;

	rgbSetColor(LD10_R, LD10_G, LD10_B, rgbColor);
	rgbSetColor(LD11_R, LD11_G, LD10_B, rgbColor);

	return 500000UL;
}

/**
 * RGB PWM Test Cleanup
 */
void RGBPWM_StopTest() {
	int i = 0;

    for(i = 0; i < 6; i++) {
        PWM_DisableChannel(i);
    }
}
