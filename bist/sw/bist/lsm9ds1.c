/*********************************************************************************
 * File: lsm9ds1.c
 * Author: Tinghui Wang
 *
 * Copyright @ 2019 RealDigital.org
 *
 * Description:
 *   LSM9DS1 Test Routine.
 *   Accelerometer Configuration:
 *   - Sample Rate: 952.0 Hz
 *   - Scale: 2G
 *   Gyroscope Configuration
 *   - Sample Rate: 952.0 Hz
 *   - Scale: 2000 degrees/second
 *   Magnetometer Configuration
 *   - Sample Rate: 80.0 Hz (Ultra-High Performance Mode)
 *   - Scale: 4 Gauss
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

#include "lsm9ds1.h"
#include "gpio.h"
#include <xil_types.h>
#include <xgpiops.h>
#include <xspips.h>
#include <xscugic.h>
#include <xil_printf.h>

static XSpiPs spiInst;
uint8_t spiSendBuffer[256];
uint8_t spiRecvBuffer[256];
uint8_t autoCalc = FALSE;

uint16_t biasRawXL[3];
uint16_t biasRawG[3];
uint16_t biasRawM[3];

float xl_res = 2.0 / 32768;
float g_res = 2000.0 / 32768;
float m_res = 3.0 / 32768;

/**
 * Set value for XL/G Data Enable pin
 */
void LSM9DS1_SetAGDen(uint8_t value) {
	XGpioPs_WritePin(&gpioPsInst, LSM9DS1_AGDEN_OFFSET, value);
}

/**
 * Initialize XL/G
 */
void LSM9DS1_InitAG() {
	LSM9DS1_SelectAG();

	/**
	 * CTRL_REG1_G (Default value: 0x00)
	 * [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
	 *
	 * ODR_G[2:0] - Output data rate selection
	 * FS_G[1:0] - Gyroscope full-scale selection
	 * BW_G[1:0] - Gyroscope bandwidth selection
	 */
	LSM9DS1_SpiWriteByte(
		CTRL_REG1_G,
		(GYRO_SAMPLE_RATE_952_0 << GYRO_SAMPLE_RATE_OFFSET) |
		(GYRO_FS_2000_DPS << GYRO_FS_OFFSET)
	);

	/**
	 * CTRL_REG2_G (Default value: 0x00)
	 * [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
	 *
	 * INT_SEL[1:0] - INT selection configuration
	 * OUT_SEL[1:0] - Out selection configuration
	 *
	 * Default to bypass HPF and LPF2
	 */
	LSM9DS1_SpiWriteByte(CTRL_REG2_G, 0x00);

	/**
	 * CTRL_REG3_G (Default value: 0x00)
	 * [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
	 *
	 * LP_mode - Low-power mode enable (0: disabled, 1: enabled)
	 * HP_EN - HPF enable (0:disabled, 1: enabled)
	 * HPCF_G[3:0] - HPF cutoff frequency
	 *
	 * As HPF is bypassed, we disable LP mode and disable HPF
	 */
	LSM9DS1_SpiWriteByte(CTRL_REG3_G, 0x00);

	/**
	 * CTRL_REG4 (Default value: 0x38)
	 * [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
	 *
	 * Zen_G - Z-axis output enable (0:disable, 1:enable)
	 * Yen_G - Y-axis output enable (0:disable, 1:enable)
	 * Xen_G - X-axis output enable (0:disable, 1:enable)
	 * LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
	 * 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
	 */
	LSM9DS1_SpiWriteByte(CTRL_REG4, 0x38);

	/**
	 * ORIENT_CFG_G (Default value: 0x00)
	 * [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
	 *
	 * SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
	 * SignY_G - Pitch axis (Y) angular rate sign (0: positive, 1: negative)
	 * SignZ_G - Pitch axis (Z) angular rate sign (0: positive, 1: negative)
	 * Orient [2:0] - Directional user orientation selection
	 */
	LSM9DS1_SpiWriteByte(ORIENT_CFG_G, 0x00);

	/**
	 * CTRL_REG5_XL (Default value: 0x38)
	 * [DEC_1][DEC_0][Zen_XL][Yen_XL][Xen_XL][0][0][0]
	 *
	 * DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
	 *     00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
	 * Zen_XL - Z-axis output enabled
	 * Yen_XL - Y-axis output enabled
	 * Xen_XL - X-axis output enabled
	 *
	 * Enable all axes without decimation.
	 */
	LSM9DS1_SpiWriteByte(CTRL_REG5_XL, 0x38);

	/**
	 * CTRL_REG6_XL (Default value: 0x00)
	 * [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
	 *
	 * ODR_XL[2:0] - Output data rate & power mode selection
	 * FS_XL[1:0] - Full-scale selection
	 * BW_SCAL_ODR - Bandwidth selection
	 * BW_XL[1:0] - Anti-aliasing filter bandwidth selection
	 */
	LSM9DS1_SpiWriteByte(
		CTRL_REG6_XL,
		(XL_SAMPLE_RATE_952_0 << XL_SAMPLE_RATE_OFFSET) |
		(XL_FS_2G << XL_FS_OFFSET)
	);

	/**
	 * CTRL_REG7_XL (Default value: 0x00)
	 * [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
	 *
	 * HR - High resolution mode (0: disable, 1: enable)
	 * DCF[1:0] - Digital filter cutoff frequency
	 * FDS - Filtered data selection
	 * HPIS1 - HPF enabled for interrupt function
	 */
	LSM9DS1_SpiWriteByte(CTRL_REG7_XL, 0x00);
}

/**
 * Initialize Magnetometer
 */
void LSM9DS1_InitM() {
	LSM9DS1_SelectM();

	/**
	 * CTRL_REG1_M (Default value: 0x10)
	 * [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][FAST_ODR][ST]
	 *
	 * TEMP_COMP - Temperature compensation
	 * OM[1:0] - X & Y axes op mode selection
	 *   00:low-power, 01:medium performance
	 *   10: high performance, 11:ultra-high performance
	 * DO[2:0] - Output data rate selection
	 * FAST_ODR - Enables data rate higher than 80Hz
	 * ST - Self-test enable
	 */
	LSM9DS1_SpiWriteByte(
		CTRL_REG1_M,
		(M_OM_ULTRAHIGH << M_XY_OM_OFFSET) |
		(M_DO_80_0 << M_DO_OFFSET)
	);

	/**
	 * CTRL_REG2_M (Default value 0x00)
	 * [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
	 *
	 * FS[1:0] - Full-scale configuration
	 * REBOOT - Reboot memory content (0:normal, 1:reboot)
	 * SOFT_RST - Reset config and user registers (0:default, 1:reset)
	 */
	LSM9DS1_SpiWriteByte(
		CTRL_REG2_M,
		M_FS_4_GAUSS << M_FS_OFFSET
	);

	/**
	 * CTRL_REG3_M (Default value: 0x03)
	 * [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
	 *
	 * I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
	 * LP - Low-power mode cofiguration (1:enable)
	 * SIM - SPI mode selection (0:write-only, 1:read/write enable)
	 * MD[1:0] - Operating mode
	 *   00:continuous conversion, 01:single-conversion,
	 *   10,11: Power-down
	 */
	LSM9DS1_SpiWriteByte(
		CTRL_REG3_M, M_MD_CONTINUOUS << M_MD_OFFSET
	);

	/**
	 * CTRL_REG4_M (Default value: 0x00)
	 * [0][0][0][0][OMZ1][OMZ0][BLE][0]
	 *
	 * OMZ[1:0] - Z-axis operative mode selection
	 * BLE - Big/little endian data
	 */
	LSM9DS1_SpiWriteByte(
		CTRL_REG4_M, M_OM_ULTRAHIGH << M_Z_OM_OFFSET
	);

	/**
	 * CTRL_REG5_M (Default value: 0x00)
	 * [0][BDU][0][0][0][0][0][0]
	 *
	 * BDU - Block data update for magnetic data
	 *   0:continuous, 1:not updated until MSB/LSB are read
	 */
	LSM9DS1_SpiWriteByte(CTRL_REG5_M, 0x00);
}

/**
 * Initialize XL/G Interrupt Registers
 */
void LSM9DS1_InitAG_Int() {
	/**
	 * INT1_CTRL (Default value: 0x00)
	 * [INT1_IG_G][INT_IG_XL][INT_FSS5][INT_OVR][INT_FTH][INT_Boot][INT_DRDY_G][INT_DRDY_XL]
	 *
	 * INT1_IG_G: Gyroscope interrupt enable on INT1_A/G pin.
	 * INT_IG_XL: Accelerometer interrupt generator on INT1_A/G pin.
	 * INT_FSS5: FSS5 interrupt enable on INT1_A/G pin.
	 * INT_OVR: Overrun interrupt on INT1_A/G pin.
	 * INT_FTH: FIFO threshold interrupt on INT1_A/G pin.
	 * INT_Boot: Boot status available on INT1_A/G pin.
	 * INT_DRDY_G: Gyroscope data ready on INT1_A/G pin.
	 * INT_DRDY_XL: Accelerometer data ready on INT1_A/G pin.
	 */
	LSM9DS1_SpiWriteByte(INT1_CTRL, XLG_INT1_DRDY_XL | XLG_INT1_DRDY_G);

	/**
	 * INT2_CTRL (Default value: 0x00)
	 * [INT2_IG_G][0][INT_FSS5][INT_OVR][INT_FTH][INT2_DRDY_TEMP][INT_DRDY_G][INT_DRDY_XL]
	 *
	 * INT1_IG_G: Gyroscope interrupt enable on INT2_A/G pin.
	 * INT_FSS5: FSS5 interrupt enable on INT2_A/G pin.
	 * INT_OVR: Overrun interrupt on INT2_A/G pin.
	 * INT_FTH: FIFO threshold interrupt on INT2_A/G pin.
	 * INT2_DRDY_TEMP: Temperature data ready on INT2_A/G pin.
	 * INT_DRDY_G: Gyroscope data ready on INT2_A/G pin.
	 * INT_DRDY_XL: Accelerometer data ready on INT2_A/G pin.
	 */
	// LSM9DS1_SpiWriteByte(INT2_CTRL, XLG_INT2_DRDY_TEMP | XLG_INT2_DRDY_G);
}

/**
 * Initialize Magnetometer Interrupt Register
 */
void LSM9DS1_InitM_int() {
	/**
	 * INT_CFG_M (Default value: 0x00)
	 * [XIEN][YIEN][ZIEN][0][0][IEA][IEL][IEN]
	 *
	 * XIEN: Enable interrupt generation on X-axis.
	 * YIEN: Enable interrupt generation on Y-axis.
	 * ZIEN: Enable interrupt generation on Z-axis.
	 * IEA: Interrupt active configuration on INT_MAG.
	 * IEL: Latch interrupt request.
	 * IEN: Interrupt enable on the INT_M pin.
	 */
	LSM9DS1_SpiWriteByte(INT_CFG_M, M_XIEN | M_YIEN | M_ZIEN | M_IEA | M_IEN);
}

uint8_t LSM9DS1_XL_DataReady() {
	return LSM9DS1_SpiReadByte(STATUS_REG) & XLG_XLDA;
}

uint8_t LSM9DS1_G_DataReady() {
	return LSM9DS1_SpiReadByte(STATUS_REG) & XLG_GDA;
}

uint8_t LSM9DS1_Temp_DataReady() {
	return LSM9DS1_SpiReadByte(STATUS_REG) & XLG_TDA;
}

uint8_t LSM9DS1_M_DataReady() {
	return LSM9DS1_SpiReadByte(STATUS_REG_M) & M_ZYXDA;
}

void LSM9DS1_INT1_AG_Handler(void *data) {
	uint16_t dataAg[3];
	if(LSM9DS1_XL_DataReady()) {
		LSM9DS1_ReadXL(dataAg);
		LSM9DS1_PrintXL((int16_t *) dataAg);
	}
	if(LSM9DS1_G_DataReady()) {
		LSM9DS1_ReadG(dataAg);
		LSM9DS1_PrintG((int16_t *) dataAg);
	}
}

void LSM9DS1_RDRY_M_Handler(void *data) {
	uint16_t dataM[3];
	if(LSM9DS1_M_DataReady()) {
		LSM9DS1_ReadM(dataM);
		LSM9DS1_PrintM((int16_t *) dataM);
	}
}

void LSM9DS1_ReadXL(uint16_t *buffer) {
	uint8_t *u8Buffer = (uint8_t *) buffer;
	LSM9DS1_SpiReadBytes(OUT_X_L_XL, u8Buffer, 6);
	buffer[0] = (u8Buffer[1] << 8) | u8Buffer[0]; // x-axis
	buffer[1] = (u8Buffer[3] << 8) | u8Buffer[2]; // y-axis
	buffer[2] = (u8Buffer[5] << 8) | u8Buffer[4]; // z-axis
	if(autoCalc) {
		buffer[0] -= biasRawXL[0];
		buffer[1] -= biasRawXL[1];
		buffer[2] -= biasRawXL[2];
	}
}

void LSM9DS1_ReadG(uint16_t *buffer) {
	uint8_t *u8Buffer = (uint8_t *) buffer;
	LSM9DS1_SpiReadBytes(OUT_X_L_G, u8Buffer, 6);
	buffer[0] = (u8Buffer[1] << 8) | u8Buffer[0]; // x-axis
	buffer[1] = (u8Buffer[3] << 8) | u8Buffer[2]; // y-axis
	buffer[2] = (u8Buffer[5] << 8) | u8Buffer[4]; // z-axis
	if(autoCalc) {
		buffer[0] -= biasRawG[0];
		buffer[1] -= biasRawG[1];
		buffer[2] -= biasRawG[2];
	}
}

void LSM9DS1_ReadM(uint16_t *buffer) {
	uint8_t *u8Buffer = (uint8_t *) buffer;
	LSM9DS1_SpiReadBytes(OUT_X_H_M, u8Buffer, 6);
	buffer[0] = (u8Buffer[1] << 8) | u8Buffer[0]; // x-axis
	buffer[1] = (u8Buffer[3] << 8) | u8Buffer[2]; // y-axis
	buffer[2] = (u8Buffer[5] << 8) | u8Buffer[4]; // z-axis
	if(autoCalc) {
		buffer[0] -= biasRawM[0];
		buffer[1] -= biasRawM[1];
		buffer[2] -= biasRawM[2];
	}
}

/**
 * Accelerometer
 */
void LSM9DS1_PrintXL(int16_t *buffer) {
	xil_printf(
		"Accelerometer (X, Y, Z) [g]: %.4f, %.4f, %.4f \r\n",
		buffer[0] * xl_res,
		buffer[1] * xl_res,
		buffer[2] * xl_res
	);
}

void LSM9DS1_PrintG(int16_t *buffer) {
	xil_printf(
		"Gyroscope (X, Y, Z) [dps]: %.4f, %.4f, %.4f \r\n",
		buffer[0] * g_res,
		buffer[1] * g_res,
		buffer[2] * g_res
	);
}

void LSM9DS1_PrintM(int16_t *buffer) {
	xil_printf(
		"Magnetometer (X, Y, Z) [guass]: %.4f, %.4f, %.4f \r\n",
		buffer[0] * m_res,
		buffer[1] * m_res,
		buffer[2] * m_res
	);
}

void LSM9DS1_EnableINT() {
	XScuGic_Enable(&gicInst, LSM9DS1_INT1_AG);
	XScuGic_Enable(&gicInst, LSM9DS1_INT_M);
}

void LSM9DS1_DisableINT() {
	XScuGic_Disable(&gicInst, LSM9DS1_INT1_AG);
	XScuGic_Disable(&gicInst, LSM9DS1_INT_M);
}

void LSM9DS1_TestStart() {
	uint8_t regval;

	XGpioPs_SetDirectionPin(&gpioPsInst, LSM9DS1_AGDEN_OFFSET, 1);
	XGpioPs_SetOutputEnablePin(&gpioPsInst, LSM9DS1_AGDEN_OFFSET, 1);
	LSM9DS1_SetAGDen(1);

	LSM9DS1_SpiInit();
	LSM9DS1_SelectAG();
	regval = LSM9DS1_SpiReadByte(WHO_AM_I_XLG);
	if(regval == WHO_AM_I_XLG_VAL) {
		xil_printf("[PASS] LSM9DS1 Accelerometer/Gyroscope WhoAmI Register Test.\r\n");
	} else {
		xil_printf("[FAIL] LSM9DS1 Accelerometer/Gyroscope WhoAmI Register Test.\r\n");
		xil_printf("\tError: Revceived 0x%x instead of 0x%x.\r\n", regval, WHO_AM_I_XLG_VAL);
	}
	LSM9DS1_InitAG();
	LSM9DS1_InitAG_Int();

	LSM9DS1_SelectM();
	regval = LSM9DS1_SpiReadByte(WHO_AM_I_M);
	if(regval == WHO_AM_I_M_VAL) {
		xil_printf("[PASS] LSM9DS1 Magnetic Sensor WhoAmI Register Test.\r\n");
	} else {
		xil_printf("[FAIL] LSM9DS1  Magnetic Sensor WhoAmI Register Test.\r\n");
		xil_printf("\tError: Revceived 0x%x instead of 0x%x.\r\n", regval, WHO_AM_I_M_VAL);
	}
	LSM9DS1_InitM();
	LSM9DS1_InitM_int();
	LSM9DS1_EnableINT();
}

void LSM9DS1_TestStop() {
	LSM9DS1_SetAGDen(0);
	LSM9DS1_DisableINT();
}

uint32_t LSM9DS1_TestLoop() {
	uint16_t dataAg[3];
	LSM9DS1_SelectAG();
	if(LSM9DS1_XL_DataReady()) {
		LSM9DS1_ReadXL(dataAg);
		LSM9DS1_PrintXL((int16_t *) dataAg);
	}
	if(LSM9DS1_G_DataReady()) {
		LSM9DS1_ReadG(dataAg);
		LSM9DS1_PrintG((int16_t *) dataAg);
	}
	uint16_t dataM[3];
	LSM9DS1_SelectM();
	if(LSM9DS1_M_DataReady()) {
		LSM9DS1_ReadM(dataM);
		LSM9DS1_PrintM((int16_t *) dataM);
	}
	return 1000000;
}

/**
 * SPI Read Bytes
 */
void LSM9DS1_SpiReadBytes(uint8_t address, uint8_t *buffer, uint8_t length) {
	spiSendBuffer[0] = 0x80 | (address & 0x7F);
	XSpiPs_PolledTransfer(&spiInst, spiSendBuffer, spiRecvBuffer, length);
	for(int i = 0; i < length; i++) {
		buffer[i] = spiRecvBuffer[i + 1];
	}
}

/**
 * SPI Send Bytes
 */
void LSM9DS1_SpiSendBytes(uint8_t address, uint8_t *buffer, uint8_t length) {
	spiSendBuffer[0] = 0x80 | (address & 0x7F);
	for(int i = 0; i < length; i++) {
		spiSendBuffer[i + 1] = buffer[i];
	}
	XSpiPs_PolledTransfer(&spiInst, spiSendBuffer, spiRecvBuffer, length);
}

/**
 * SPI Read Byte
 */
uint8_t LSM9DS1_SpiReadByte(uint8_t address) {
	spiSendBuffer[0] = 0x80 | address;
	spiSendBuffer[1] = 0x00;
	XSpiPs_PolledTransfer(&spiInst, spiSendBuffer, spiRecvBuffer, 2);
	return spiRecvBuffer[1];
}

/**
 * SPI Write Byte
 */
void LSM9DS1_SpiWriteByte(uint8_t address, uint8_t value) {
	spiSendBuffer[0] = 0x80 | address;
	spiSendBuffer[1] = value;
	XSpiPs_PolledTransfer(&spiInst, spiSendBuffer, spiRecvBuffer, 2);
}

/**
 * Initialize PS SPI0 controller
 */
void LSM9DS1_SpiInit() {
	XSpiPs_Config *spiConfig;
	spiConfig = XSpiPs_LookupConfig(LSM9DS1_SPI_DEVICE_ID);
	XSpiPs_CfgInitialize(&spiInst, spiConfig, spiConfig->BaseAddress);
	XSpiPs_SetOptions(&spiInst, XSPIPS_MASTER_OPTION | XSPIPS_CLK_ACTIVE_LOW_OPTION | XSPIPS_CLK_PHASE_1_OPTION | XSPIPS_FORCE_SSELECT_OPTION);
	XSpiPs_SetClkPrescaler(&spiInst, XSPIPS_CLK_PRESCALE_128);
}

/**
 * Enable XL/G SPI Interface
 */
void LSM9DS1_SelectAG() {
	XSpiPs_SetSlaveSelect(&spiInst, 0);
}

/**
 * Enable Magnetometer SPI Interface
 */
void LSM9DS1_SelectM() {
	XSpiPs_SetSlaveSelect(&spiInst, 1);
}

