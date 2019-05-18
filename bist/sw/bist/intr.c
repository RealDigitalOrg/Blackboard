/*********************************************************************************
 * File: intr.h
 * Author: Tinghui Wang
 *
 * Copyright @ 2019 RealDigital.org
 *
 * Description:
 *   Interrupt handling routine.
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

#include "intr.h"
#include "audio.h"
#include "lsm9ds1.h"
#include <xscugic.h>

extern XExc_VectorTableEntry XExc_VectorTable[];

XScuGic gicInst;

/**
 * Initialize Interrupts
 */
int GicInit() {
	int status;
	XScuGic_Config *config = XScuGic_LookupConfig(XPAR_PS7_SCUGIC_0_DEVICE_ID);
	status = XScuGic_CfgInitialize(&gicInst, config, config->CpuBaseAddress);
	if(status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			(Xil_ExceptionHandler) XScuGic_InterruptHandler, &gicInst);
	Xil_ExceptionEnable();

	XScuGic_Connect(&gicInst, SPKL_INTR, Audio_SpkLIntrHandler, NULL);
	XScuGic_Connect(&gicInst, SPKR_INTR, Audio_SpkRIntrHandler, NULL);
	XScuGic_Connect(&gicInst, MIC_INTR, Audio_MicIntrHandler, NULL);
	XScuGic_Connect(&gicInst, LSM9DS1_INT1_AG, LSM9DS1_INT1_AG_Handler, NULL);
	XScuGic_Connect(&gicInst, LSM9DS1_INT_M, LSM9DS1_RDRY_M_Handler, NULL);

	XScuGic_SetPriorityTriggerType(&gicInst, MIC_INTR, 0xa0, 0x03);
	XScuGic_SetPriorityTriggerType(&gicInst, SPKL_INTR, 0xa0, 0x03);
	XScuGic_SetPriorityTriggerType(&gicInst, SPKR_INTR, 0xa0, 0x03);
	XScuGic_SetPriorityTriggerType(&gicInst, LSM9DS1_INT1_AG, 0xa0, 0x03);
	XScuGic_SetPriorityTriggerType(&gicInst, LSM9DS1_INT_M, 0xa0, 0x03);

	return XST_SUCCESS;
}

