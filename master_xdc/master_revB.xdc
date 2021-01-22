###############################################################################
# File: system_revB.xdc
# Author: Tinghui Wang
#
# Copyright (c) 2018-2019, RealDigital.org
#
# Description:
#   Blackboard Rev. B master contraint file (include location and I/O 
#   constraints).
#
# History:
#   01/24/20: Initial release
#
# License: BSD 3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
# POSSIBILITY OF SUCH DAMAGE.
#
###############################################################################

# 100MHz system clock
set_property -dict {PACKAGE_PIN H16 IOSTANDARD LVCMOS33} [get_ports SYSCLK]; #IO_L13P_T2_MRCC_35 Schematic=SYSCLK
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 4} [get_ports SYSCLK];

# LEDS
set_property -dict {PACKAGE_PIN N20 IOSTANDARD LVCMOS33} [get_ports {led[0]}]; #IO_L14P_T2_SRCC_34 Schematic=LD0
set_property -dict {PACKAGE_PIN P20 IOSTANDARD LVCMOS33} [get_ports {led[1]}]; #IO_L14N_T2_SRCC_34 Schematic=LD1
set_property -dict {PACKAGE_PIN R19 IOSTANDARD LVCMOS33} [get_ports {led[2]}]; #IO_0_34 Schematic=LD2
set_property -dict {PACKAGE_PIN T20 IOSTANDARD LVCMOS33} [get_ports {led[3]}]; #IO_L15P_T2_DQS_34 Schematic=LD3

# RGB_LEDS
set_property -dict {PACKAGE_PIN U13 IOSTANDARD LVCMOS33} [get_ports {RGB_led_0[2]}]; #IO_L3P_T0_DWS_PUDC_B_34 Schematic=LD4_R
set_property -dict {PACKAGE_PIN T19 IOSTANDARD LVCMOS33} [get_ports {RGB_led_0[1]}]; #IO_25_34 Schematic=LD4_G
set_property -dict {PACKAGE_PIN W20 IOSTANDARD LVCMOS33} [get_ports {RGB_led_0[0]}]; #IO_L16N_T2_34 Schematic=LD4_B

set_property -dict {PACKAGE_PIN Y19 IOSTANDARD LVCMOS33} [get_ports {RGB_led_1[2]}]; #IO_L17N_T2_34  Schematic=LD5_R
set_property -dict {PACKAGE_PIN V20 IOSTANDARD LVCMOS33} [get_ports {RGB_led_1[1]}]; #IO_L16P_T2_34 Schematic=LD5_G
set_property -dict {PACKAGE_PIN W19 IOSTANDARD LVCMOS33} [get_ports {RGB_led_1[0]}]; #IO_L22N_T3_34 Schematic=LD5_B

set_property -dict {PACKAGE_PIN W18 IOSTANDARD LVCMOS33} [get_ports {RGB_led_2[2]}]; #IO_L22P_T3_34  Schematic=LD6_R
set_property -dict {PACKAGE_PIN W16 IOSTANDARD LVCMOS33} [get_ports {RGB_led_2[1]}]; #IO_L18N_T2_34 Schematic=LD6_G
set_property -dict {PACKAGE_PIN Y18 IOSTANDARD LVCMOS33} [get_ports {RGB_led_2[0]}]; #IO_L17P_T2_34 Schematic=LD6_B

set_property -dict {PACKAGE_PIN Y14 IOSTANDARD LVCMOS33} [get_ports {RGB_led_3[2]}]; #IO_L8N_T1_34 Schematic=LD7_R
set_property -dict {PACKAGE_PIN Y16 IOSTANDARD LVCMOS33} [get_ports {RGB_led_3[1]}]; #IO_L7P_T1_34 Schematic=LD7_G
set_property -dict {PACKAGE_PIN Y17 IOSTANDARD LVCMOS33} [get_ports {RGB_led_3[0]}]; #IO_L7N_T1_34 Schematic=LD7_B

# Switches
set_property -dict {PACKAGE_PIN R17 IOSTANDARD LVCMOS33} [get_ports {sw[0]}]; #IO_L19N_T3_VREF_34 Schematic=SW0
set_property -dict {PACKAGE_PIN U20 IOSTANDARD LVCMOS33} [get_ports {sw[1]}]; #IO_L15N_T2_DQS_34 Schematic=SW1
set_property -dict {PACKAGE_PIN R16 IOSTANDARD LVCMOS33} [get_ports {sw[2]}]; #IO_L19P_T3_34 Schematic=SW2
set_property -dict {PACKAGE_PIN N16 IOSTANDARD LVCMOS33} [get_ports {sw[3]}]; #IO_L21N_T3_DQS_AD14N_35 Schematic=SW3
set_property -dict {PACKAGE_PIN R14 IOSTANDARD LVCMOS33} [get_ports {sw[4]}]; #IO_L6N_T0_VREF_34 Schematic=SW4
set_property -dict {PACKAGE_PIN P14 IOSTANDARD LVCMOS33} [get_ports {sw[5]}]; #IO_L6P_T0_34 Schematic=SW5
set_property -dict {PACKAGE_PIN L15 IOSTANDARD LVCMOS33} [get_ports {sw[6]}]; #IO_L22N_T3_AD7N_35 Schematic=SW6
set_property -dict {PACKAGE_PIN M15 IOSTANDARD LVCMOS33} [get_ports {sw[7]}]; #IO_L23N_T3_35 Schematic=SW7

# Push Buttons
set_property -dict {PACKAGE_PIN W14 IOSTANDARD LVCMOS33} [get_ports {btn[0]}]; #IO_L8P_T1_34 Schematic=BTN0
set_property -dict {PACKAGE_PIN W13 IOSTANDARD LVCMOS33} [get_ports {btn[1]}]; #IO_L4N_T0_34 Schematic=BTN1
set_property -dict {PACKAGE_PIN P15 IOSTANDARD LVCMOS33} [get_ports {btn[2]}]; #IO_L24P_T3_34 Schematic=BTN2
set_property -dict {PACKAGE_PIN M14 IOSTANDARD LVCMOS33} [get_ports {btn[3]}]; #IO_L23P_T3_35 Schematic=BTN3

# Seven Segment Display
# common anodes
set_property -dict {PACKAGE_PIN K19 IOSTANDARD LVCMOS33} [get_ports {ssd_an[0]}]; #IO_L10P_T1_AD11P_35 Schematic=SSEG_AN0
set_property -dict {PACKAGE_PIN H17 IOSTANDARD LVCMOS33} [get_ports {ssd_an[1]}]; #IO_L13N_T2_MRCC_35 Schematic=SSEG_AN1
set_property -dict {PACKAGE_PIN M18 IOSTANDARD LVCMOS33} [get_ports {ssd_an[2]}]; #IO_L8N_T1_AD10N_35 Schematic=SSEG_AN2
set_property -dict {PACKAGE_PIN L16 IOSTANDARD LVCMOS33} [get_ports {ssd_an[3]}]; #IO_L11P_T1_SRCC_35 Schematic=SSEG_AN3

# segment cathodes
set_property -dict {PACKAGE_PIN K14 IOSTANDARD LVCMOS33} [get_ports {ssd_seg[0]}]; #IO_L20P_T3_AD6P_35 Schematic=SSEG_CA
set_property -dict {PACKAGE_PIN H15 IOSTANDARD LVCMOS33} [get_ports {ssd_seg[1]}]; #IO_L19P_T3_35 Schematic=SSEG_CB
set_property -dict {PACKAGE_PIN J18 IOSTANDARD LVCMOS33} [get_ports {ssd_seg[2]}]; #IO_L14P_T2_AD4P_SRCC_35 Schematic=SSEG_CC
set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS33} [get_ports {ssd_seg[3]}]; #IO_25_35 Schematic=SSEG_CD
set_property -dict {PACKAGE_PIN M17 IOSTANDARD LVCMOS33} [get_ports {ssd_seg[4]}]; #IO_L8P_T1_AD10P_35 Schematic=SSEG_CE
set_property -dict {PACKAGE_PIN J16 IOSTANDARD LVCMOS33} [get_ports {ssd_seg[5]}]; #IO_L24N_T3_AD15N_35 Schematic=SSEG_CF
set_property -dict {PACKAGE_PIN H18 IOSTANDARD LVCMOS33} [get_ports {ssd_seg[6]}]; #IO_L8P_T1_AD10P_35 Schematic=SSEG_CG
set_property -dict {PACKAGE_PIN K18 IOSTANDARD LVCMOS33} [get_ports {ssd_dp}]; #IO_L12N_T1_MRCC_35 Schematic=SSEG_DP

# 9-axis inertial module: LSM9DS1
# 3D Accelerometer, 3D Gyroscope and 3D Magnetometer
set_property -dict {PACKAGE_PIN H20 IOSTANDARD LVCMOS33} [get_ports {GYRO_SCK}]; #IO_L17N_T2_AD5N_35 Schematic=GYRO_SCL
set_property -dict {PACKAGE_PIN J19 IOSTANDARD LVCMOS33} [get_ports {GYRO_MOSI}]; #IO_L10N_T1_AD11N_35 Schematic=GYRO_SDA
set_property -dict {PACKAGE_PIN J20 IOSTANDARD LVCMOS33} [get_ports {GYRO_MISO_AG}]; #IO_L17P_T2_AD5P_35 Schematic=GYRO_SDO_A/G
set_property -dict {PACKAGE_PIN L17 IOSTANDARD LVCMOS33} [get_ports {GYRO_MISO_M}]; #IO_L11N_T1_SRCC_35 Schematic=GYRO_SDO_M
set_property -dict {PACKAGE_PIN K17 IOSTANDARD LVCMOS33} [get_ports {GYRO_SS_AG}]; #IO_L12P_T1_MRCC_35 Schematic=GYRO_CS_A/G
set_property -dict {PACKAGE_PIN K16 IOSTANDARD LVCMOS33} [get_ports {GYRO_SS_M}]; #IO_L24P_T3_AD15P_35 Schematic=GYRO_CS_M
set_property -dict {PACKAGE_PIN J14 IOSTANDARD LVCMOS33} [get_ports {GYRO_DEN_AG}]; #IO_L20N_T3_AD6N_35 Schematic=GYRO_DEN_A/G
set_property -dict {PACKAGE_PIN L20 IOSTANDARD LVCMOS33} [get_ports {GYRO_DRDY_M}]; #IO_L9N_T1_DQS_AD3N_35 Schematic=GYRO_DRDY_M
set_property -dict {PACKAGE_PIN M20 IOSTANDARD LVCMOS33} [get_ports {GYRO_INT_AG}]; #IO_L7N_T1_AD2N_35 Schematic=GYRO_INT_A/G
set_property -dict {PACKAGE_PIN L19 IOSTANDARD LVCMOS33} [get_ports {GYRO_INT_M}]; #IO_L9P_T1_DQS_AD3P_35 Schematic=GYRO_INT_M

# MIC
set_property -dict {PACKAGE_PIN L14 IOSTANDARD LVCMOS33} [get_ports {PDM_MIC_CLK}]; #IO_L22P_T3_AD7P_35 Schematic=M_CLK
set_property -dict {PACKAGE_PIN N15 IOSTANDARD LVCMOS33} [get_ports {PDM_MIC_DATA}]; #IO_L21P_T3_DQS_AD14P_35 Schematic=M_DATA

# Speaker
set_property -dict {PACKAGE_PIN G18 IOSTANDARD LVCMOS33} [get_ports {audio}]; #IO_L16N_T2_35 Schematic=AUDIO

# VGA
set_property -dict {PACKAGE_PIN V15 IOSTANDARD LVCMOS33} [get_ports {VGA_R[0]}]; #IO_L10P_T1_34 Sch=VGA_R4_CON
set_property -dict {PACKAGE_PIN W15 IOSTANDARD LVCMOS33} [get_ports {VGA_R[1]}]; #IO_L10N_T1_34 Sch=VGA_R5_CON
set_property -dict {PACKAGE_PIN V16 IOSTANDARD LVCMOS33} [get_ports {VGA_R[2]}]; #IO_L18P_T2_34 Sch=VGA_R6_CON
set_property -dict {PACKAGE_PIN T16 IOSTANDARD LVCMOS33} [get_ports {VGA_R[3]}]; #IO_L18N_T2_AD13N_35 Sch=VGA_R7_CON
set_property -dict {PACKAGE_PIN T15 IOSTANDARD LVCMOS33} [get_ports {VGA_G[0]}]; #IO_L5N_T0_34 Sch=VGA_G4_CON
set_property -dict {PACKAGE_PIN V13 IOSTANDARD LVCMOS33} [get_ports {VGA_G[1]}]; #IO_L3N_T0_DQS_34 Sch=VGA_G5_CON
set_property -dict {PACKAGE_PIN U14 IOSTANDARD LVCMOS33} [get_ports {VGA_G[2]}]; #IO_L11P_T1_SRCC_34 Sch=VGA_G6_CON
set_property -dict {PACKAGE_PIN U15 IOSTANDARD LVCMOS33} [get_ports {VGA_G[3]}]; #IO_L11N_T1_SRCC_34 Sch=VGA_G7_CON
set_property -dict {PACKAGE_PIN T11 IOSTANDARD LVCMOS33} [get_ports {VGA_B[0]}]; #IO_L1P_T0_34 Sch=VGA_B4_CON
set_property -dict {PACKAGE_PIN T14 IOSTANDARD LVCMOS33} [get_ports {VGA_B[1]}]; #IO_L5P_T0_34 Sch=VGA_B5_CON
set_property -dict {PACKAGE_PIN U12 IOSTANDARD LVCMOS33} [get_ports {VGA_B[2]}]; #IO_L2N_T0_34 Sch=VGA_B6_CON
set_property -dict {PACKAGE_PIN V12 IOSTANDARD LVCMOS33} [get_ports {VGA_B[3]}]; #IO_L4P_T0_34 Sch=VGA_B7_CON
set_property -dict {PACKAGE_PIN T12 IOSTANDARD LVCMOS33} [get_ports {VGA_HS}]; #IO_L2P_T0_34 Sch=VGA_HS
set_property -dict {PACKAGE_PIN T10 IOSTANDARD LVCMOS33} [get_ports {VGA_VS}]; #IO_L1N_T0_34 Sch=VGA_VS

# HDMI Signals
set_property -dict {PACKAGE_PIN U19 IOSTANDARD TMDS_33} [get_ports hdmi_tx_clk_n]; #IO_L12N_T1_MRCC_34 Sch=HDMI_TX_CLK_N
set_property -dict {PACKAGE_PIN U18 IOSTANDARD TMDS_33} [get_ports hdmi_tx_clk_p]; #IO_L12P_T1_MRCC_34 Sch=HDMI_TX_CLK_P
set_property -dict {PACKAGE_PIN V18 IOSTANDARD TMDS_33} [get_ports {hdmi_tx_n[0]}]; #IO_L21N_T3_DQS_34 Sch=HDMI_TX0_N
set_property -dict {PACKAGE_PIN V17 IOSTANDARD TMDS_33} [get_ports {hdmi_tx_p[0]}]; #IO_L21P_T3_DQS_34 Sch=HDMI_TX0_P
set_property -dict {PACKAGE_PIN P18 IOSTANDARD TMDS_33} [get_ports {hdmi_tx_n[1]}]; #IO_L23N_T3_34 Sch=HDMI_TX1_N
set_property -dict {PACKAGE_PIN N15 IOSTANDARD TMDS_33} [get_ports {hdmi_tx_p[1]}]; #IO_L23P_T3_34 Sch=HDMI_TX1_P
set_property -dict {PACKAGE_PIN P19 IOSTANDARD TMDS_33} [get_ports {hdmi_tx_n[2]}]; #IO_L13N_T2_MRCC_34 Sch=HDMI_TX2_N
set_property -dict {PACKAGE_PIN N18 IOSTANDARD TMDS_33} [get_ports {hdmi_tx_p[2]}]; #IO_L13P_T2_MRCC_34 Sch=HDMI_TX2_P
set_property -dict {PACKAGE_PIN U17 IOSTANDARD LVCMOS33} [get_ports hdmi_tx_cec]; #IO_L9N_T1_DQS_34 Sch=HDMI_TX_CEC
set_property -dict {PACKAGE_PIN P16 IOSTANDARD LVCMOS33} [get_ports hdmi_tx_hpd]; #IO_L24N_T3_34 Sch=HDMI_TX_HPD
set_property -dict {PACKAGE_PIN T17 IOSTANDARD LVCMOS33} [get_ports hdmi_tx_scl]; #IO_L20P_T3_34 Sch=HDMI_TX_SCL
set_property -dict {PACKAGE_PIN R18 IOSTANDARD LVCMOS33} [get_ports hdmi_tx_sda]; #IO_L20N_T3_34 Sch=HDMI_TX_SDA

# PmodA
set_property -dict {PACKAGE_PIN F16 IOSTANDARD LVCMOS33} [get_ports {JA_p[0]}]; #IO_L6P_T0_35 Sch=JA1_P
set_property -dict {PACKAGE_PIN F17 IOSTANDARD LVCMOS33} [get_ports {JA_n[0]}]; #IO_L6N_T0_VREF_35 Sch=JA1_N
set_property -dict {PACKAGE_PIN G19 IOSTANDARD LVCMOS33} [get_ports {JA_p[1]}]; #IO_L18P_T2_AD13P_35 Sch=JA2_P
set_property -dict {PACKAGE_PIN G20 IOSTANDARD LVCMOS33} [get_ports {JA_n[1]}]; #IO_L18N_T2_AD13N_35 Sch=JA2_N
set_property -dict {PACKAGE_PIN E18 IOSTANDARD LVCMOS33} [get_ports {JA_p[2]}]; #IO_L5P_T0_AD9P_35 Sch=JA3_P
set_property -dict {PACKAGE_PIN E19 IOSTANDARD LVCMOS33} [get_ports {JA_n[2]}]; #IO_L5N_T0_AD9N_35 Sch=JA3_N
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS33} [get_ports {JA_p[3]}]; #IO_L3P_T0_DQS_AD1P_35 Sch=JA4_P
set_property -dict {PACKAGE_PIN D18 IOSTANDARD LVCMOS33} [get_ports {JA_n[3]}]; #IO_L3N_T0_DQS_AD1N_35 Sch=JA4_N

# PmodB
set_property -dict {PACKAGE_PIN D19 IOSTANDARD LVCMOS33} [get_ports {JB_p[0]}]; #IO_L4P_T0_35 Sch=JB1_p
set_property -dict {PACKAGE_PIN D20 IOSTANDARD LVCMOS33} [get_ports {JB_n[0]}]; #IO_L4N_T0_35 Sch=JB1_N
set_property -dict {PACKAGE_PIN F19 IOSTANDARD LVCMOS33} [get_ports {JB_p[1]}]; #IO_L15P_T2_DQS_AD12P_35 Sch=JB2_P
set_property -dict {PACKAGE_PIN F20 IOSTANDARD LVCMOS33} [get_ports {JB_n[1]}]; #IO_L15N_T2_DQS_AD12N_35 Sch=JB2_N
set_property -dict {PACKAGE_PIN C20 IOSTANDARD LVCMOS33} [get_ports {JB_p[2]}]; #IO_L1P_T0_AD0P_35 Sch=JB3_P
set_property -dict {PACKAGE_PIN B20 IOSTANDARD LVCMOS33} [get_ports {JB_n[2]}]; #IO_L1N_T0_AD0N_35 Sch=JB3_N
set_property -dict {PACKAGE_PIN B19 IOSTANDARD LVCMOS33} [get_ports {JB_p[3]}]; #IO_L2P_T0_AD8P_35 Sch=JB4_P
set_property -dict {PACKAGE_PIN A20 IOSTANDARD LVCMOS33} [get_ports {JB_n[3]}]; #IO_L2N_T0_AD8N_35 Sch=JB4_N

# Servos
set_property -dict {PACKAGE_PIN G17 IOSTANDARD LVCMOS33} [get_ports servo[0]]; #IO_L16P_T2_35 Sch=SERVO1
set_property -dict {PACKAGE_PIN G15 IOSTANDARD LVCMOS33} [get_ports servo[1]]; #IO_L19N_T3_VREF_35 Sch=SERVO2
set_property -dict {PACKAGE_PIN G14 IOSTANDARD LVCMOS33} [get_ports servo[2]]; #IO_0_35 Sch=SERVO3
set_property -dict {PACKAGE_PIN M19 IOSTANDARD LVCMOS33} [get_ports servo[3]]; #IO_L7P_T1_AD2P_35 Sch=SERVO4
