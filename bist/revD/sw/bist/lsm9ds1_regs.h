/*********************************************************************************
 * File: lsm9ds1_regs.h
 * Author: Tinghui Wang
 *
 * Copyright @ 2019 RealDigital.org
 *
 * Description:
 *   Register definition of LSM9DS1 Module
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
#ifndef SRC_LSM9DS1_REGS_H_
#define SRC_LSM9DS1_REGS_H_

/**
 * LSM9DS1 Accelerometer and gyroscope register description
 */
#define ACT_THS             0x04
#define ACT_DUR             0x05
#define INT_GEN_CFG_XL      0x06
#define INT_GEN_THS_X_XL    0x07
#define INT_GEN_THS_Y_XL    0x08
#define INT_GEN_THS_Z_XL    0x09
#define INT_GEN_DUR_XL      0x0A
#define REFERENCE_G         0x0B
#define INT1_CTRL           0x0C
#define INT2_CTRL           0x0D
#define WHO_AM_I_XLG        0x0F
#define CTRL_REG1_G         0x10
#define CTRL_REG2_G         0x11
#define CTRL_REG3_G         0x12
#define ORIENT_CFG_G        0x13
#define INT_GEN_SRC_G       0x14
#define OUT_TEMP_L          0x15
#define OUT_TEMP_H          0x16
#define STATUS_REG          0x17
#define OUT_X_L_G           0x18
#define OUT_X_H_G           0x19
#define OUT_Y_L_G           0x1A
#define OUT_Y_H_G           0x1B
#define OUT_Z_L_G           0x1C
#define OUT_Z_H_G           0x1D
#define CTRL_REG4           0x1E
#define CTRL_REG5_XL        0x1F
#define CTRL_REG6_XL        0x20
#define CTRL_REG7_XL        0x21
#define CTRL_REG8           0x22
#define CTRL_REG9           0x23
#define CTRL_REG10          0x24
#define INT_GEN_SRC_XL      0x26
#define STATUS_REG_1        0x27
#define OUT_X_L_XL          0x28
#define OUT_X_H_XL          0x29
#define OUT_Y_L_XL          0x2A
#define OUT_Y_H_XL          0x2B
#define OUT_Z_L_XL          0x2C
#define OUT_Z_H_XL          0x2D
#define FIFO_CTRL           0x2E
#define FIFO_SRC            0x2F
#define INT_GEN_CFG_G       0x30
#define INT_GEN_THS_XH_G    0x31
#define INT_GEN_THS_XL_G    0x32
#define INT_GEN_THS_YH_G    0x33
#define INT_GEN_THS_YL_G    0x34
#define INT_GEN_THS_ZH_G    0x35
#define INT_GEN_THS_ZL_G    0x36
#define INT_GEN_DUR_G       0x37

/**
 * LSM9DS1 Magnetic Sensor register description
 */
#define OFFSET_X_REG_L_M    0x05
#define OFFSET_X_REG_H_M    0x06
#define OFFSET_Y_REG_L_M    0x07
#define OFFSET_Y_REG_H_M    0x08
#define OFFSET_Z_REG_L_M    0x09
#define OFFSET_Z_REG_H_M    0x0A
#define WHO_AM_I_M          0x0F
#define CTRL_REG1_M         0x20
#define CTRL_REG2_M         0x21
#define CTRL_REG3_M         0x22
#define CTRL_REG4_M         0x23
#define CTRL_REG5_M         0x24
#define STATUS_REG_M        0x27
#define OUT_X_L_M           0x28
#define OUT_X_H_M           0x29
#define OUT_Y_L_M           0x2A
#define OUT_Y_H_M           0x2B
#define OUT_Z_L_M           0x2C
#define OUT_Z_H_M           0x2D
#define INT_CFG_M           0x30
#define INT_SRC_M           0x30
#define INT_THS_L_M         0x32
#define INT_THS_H_M         0x33

/**
 * WHO_AM_I Default Response
 */
#define WHO_AM_I_XLG_VAL    0x68
#define WHO_AM_I_M_VAL      0x3D

/**
 * GYRO Sample Rate Constants
 *
 * Sample Rate | ODR (Hz)     | Cutoff (Hz)  |
 * ------------|--------------|--------------|
 * 0x00        | Power-down   | N/A          |
 * 0x01        | 14.9         | 5            |
 * 0x02        | 59.5         | 19           |
 * 0x03        | 119          | 38           |
 * 0x04        | 238          | 76           |
 * 0x05        | 476          | 100          |
 * 0x06        | 952          | 100          |
 * 0x07        | N/A          | N/A          |
 */
#define GYRO_SAMPLE_RATE_OFF        0x00
#define GYRO_SAMPLE_RATE_14_9       0x01
#define GYRO_SAMPLE_RATE_59_5       0x02
#define GYRO_SAMPLE_RATE_119_0      0x03
#define GYRO_SAMPLE_RATE_238_0      0x04
#define GYRO_SAMPLE_RATE_476_0      0x05
#define GYRO_SAMPLE_RATE_952_0      0x06
#define GYRO_SAMPLE_RATE_OFFSET     0x05

/**
 * GYRO Full-scale Selection
 */
#define GYRO_FS_245_DPS             0x00
#define GYRO_FS_500_DPS             0x01
#define GYRO_FS_2000_DPS            0x03
#define GYRO_FS_OFFSET              0x03

/**
 * Accelerometer (XL) Sample Rate Constants
 *
 * Sample Rate | ODR (Hz)     |
 * ------------|--------------|
 * 0x00        | Power-down   |
 * 0x01        | 10           |
 * 0x02        | 50           |
 * 0x03        | 119          |
 * 0x04        | 238          |
 * 0x05        | 476          |
 * 0x06        | 952          |
 * 0x07        | N/A          |
 */
#define XL_SAMPLE_RATE_OFF        0x00
#define XL_SAMPLE_RATE_10_0       0x01
#define XL_SAMPLE_RATE_50_0       0x02
#define XL_SAMPLE_RATE_119_0      0x03
#define XL_SAMPLE_RATE_238_0      0x04
#define XL_SAMPLE_RATE_476_0      0x05
#define XL_SAMPLE_RATE_952_0      0x06
#define XL_SAMPLE_RATE_OFFSET     0x05

/**
 * Accelerometer (XL) Full-scale Selection
 */
#define XL_FS_2G      0x00
#define XL_FS_16G     0x01
#define XL_FS_4G      0x02
#define XL_FS_8G      0x03
#define XL_FS_OFFSET  0x03

/**
 * Accelerometer (XL) Bandwidth Selection
 */
#define XL_BW_SCALE_ODR_OFFSET     0x02
#define XL_BW_408_HZ               0x00
#define XL_BW_211_HZ               0x01
#define XL_BW_105_HZ               0x02
#define XL_BW_50_HZ                0x03

/**
 * Magnetometer Temperature Compensation
 */
#define M_TEMP_COMP_MASK           0x10

/**
 * Magnetometer Operation Mode
 */
#define M_OM_LOWPOWER              0x00
#define M_OM_MEDIUM                0x01
#define M_OM_HIGH                  0x02
#define M_OM_ULTRAHIGH             0x03
#define M_XY_OM_OFFSET             0x05
#define M_XY_OM_MASK               0x60
#define M_Z_OM_OFFSET              0x02
#define M_Z_OM_MASK                0x0C

/**
 * Magnetometer Output Data Rate
 */
#define M_DO_0_625                 0x00
#define M_DO_1_25                  0x01
#define M_DO_2_5                   0x02
#define M_DO_5_0                   0x03
#define M_DO_10_0                  0x04
#define M_DO_20_0                  0x05
#define M_DO_40_0                  0x06
#define M_DO_80_0                  0x07
#define M_DO_OFFSET                0x02

/**
 * Magnetometer FAST ODR
 */
#define M_FAST_ODR_MASK            0x02

/**
 * Magnetometer Full-scale Selection
 */
#define M_FS_4_GAUSS               0x00
#define M_FS_8_GAUSS               0x01
#define M_FS_12_GAUSS              0x02
#define M_FS_16_GAUSS              0x03
#define M_FS_OFFSET                0x05
#define M_FS_MASK                  0x60
#define M_REBOOT_MASK              0x08
#define M_SOFT_RST_MASK            0x04

/**
 * Magnetometer Operating Mode
 */
#define M_MD_CONTINUOUS            0x00
#define M_MD_SINGLE                0x01
#define M_MD_POWERDOWN             0x02
#define M_MD_OFFSET                0x00
#define M_MD_MASK                  0x03
#define M_SIM_MASK                 0x04
#define M_LP_MASK                  0x20
#define M_I2C_DISABLE_MASK         0x80

/**
 * Interrupt Register
 */
#define XLG_INT1_IG_G              0x80
#define XLG_INT1_IG_XL             0x40
#define XLG_INT1_FSS5              0x20
#define XLG_INT1_OVR               0x10
#define XLG_INT1_FTH               0x08
#define XLG_INT1_BOOT              0x04
#define XLG_INT1_DRDY_G            0x02
#define XLG_INT1_DRDY_XL           0x01

#define XLG_INT2_IG_G              0x80
#define XLG_INT2_FSS5              0x20
#define XLG_INT2_OVR               0x10
#define XLG_INT2_FTH               0x08
#define XLG_INT2_DRDY_TEMP         0x04
#define XLG_INT2_DRDY_G            0x02
#define XLG_INT2_DRDY_XL           0x01

/**
 * XL_G Status Register
 */
#define XLG_IG_XL                  0x40
#define XLG_IG_G                   0x20
#define XLG_INACT                  0x10
#define XLG_BOOT_STATUS            0x08
#define XLG_TDA                    0x04
#define XLG_GDA                    0x02
#define XLG_XLDA                   0x01

/**
 * Magnetometer Interrupt Register
 */
#define M_XIEN                     0x80
#define M_YIEN                     0x40
#define M_ZIEN                     0x20
#define M_IEA                      0x04
#define M_IEL                      0x02
#define M_IEN                      0x01

/**
 * Magnetometer Status Register
 */
#define M_ZYXOR                    0x80
#define M_ZOR                      0x40
#define M_YOR                      0x20
#define M_XOR                      0x10
#define M_ZYXDA                    0x08
#define M_ZDA                      0x04
#define M_YDA                      0x02
#define M_XDA                      0x01

#endif /* SRC_LSM9DS1_REGS_H_ */
