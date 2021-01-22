/*********************************************************************************
 * File: seven_seg_display.c
 * Author: Tinghui Wang
 *
 * Copyright @ 2019 RealDigital.org
 *
 * Description:
 *   Test for seven segment display
 *
 * History:
 *   03/29/19: Created
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

#include "seven_seg_display.h"
#include <xil_io.h>

uint8_t iDigit = 0;
uint8_t iSegment = 0;

/**
 * Disable cathodes of all digits
 */
void SSD_Clear() {
	for(int i = 0; i < 4; i++) {
		Xil_Out8(SSD_BASEADDR + SSD_DIGITS_OFFSET(i), 0xFF);
	}
}

/**
 * Setup SSD to Segment Mode
 */
void SSD_StartTest() {
	Xil_Out32(SSD_BASEADDR + SSD_MODE_OFFSET, SSD_SEG_MODE | SSD_ENABLE);
}

/**
 * SSD Test Loop
 */
uint32_t SSD_TestLoop() {
	if(iSegment == 7) {
		iSegment = 0;
		iDigit = (iDigit == 3) ? 0 : iDigit + 1;
	} else {
		iSegment++;
	}
	SSD_Clear();
	Xil_Out8(SSD_BASEADDR + SSD_DIGITS_OFFSET(iDigit), ~(0x01 << iSegment));
	return 1000000UL;
}

/**
 * Disable SSD
 */
void SSD_StopTest() {
	SSD_Clear();
	Xil_Out32(SSD_BASEADDR + SSD_MODE_OFFSET, 0x00UL);
}

