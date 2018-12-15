/*
 * intc.c
 *
 *  Created on: Oct 3, 2018
 *      Author: tingh
 */

#include "xparameters.h"
#include "xscugic.h"
#include "xscugic_hw.h"
#include "xuartps.h"
#include "xuartlite.h"

typedef struct {
	Xil_ExceptionHandler Handler;
	void *Data;
} XExc_VectorTableEntry;

extern XExc_VectorTableEntry XExc_VectorTable[];

XScuGic gicInst;

/**
 * Initialize Interrupts
 */
int GicInit() {
	int status;
	XScuGic_Config *config = XScuGic_LookupConfig(XPAR_PS7_SCUGIC_0_DEVICE_ID);
	if(config == NULL) {
		return XST_FAILURE;
	}

	status = XScuGic_CfgInitialize(&gicInst, config, config->CpuBaseAddress);
	if(status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Connect the interrupt controller interrupt handler to the
	 * hardware interrupt handling logic in the processor.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			(Xil_ExceptionHandler) XScuGic_InterruptHandler, &gicInst);
	Xil_ExceptionEnable();

	/* Register device handler */

	return XST_SUCCESS;
}
