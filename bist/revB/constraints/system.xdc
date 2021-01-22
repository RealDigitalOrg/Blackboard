###############################################################################
# File: system.xdc
# Author: Tinghui Wang
#
# Copyright (c) 2018-2019, RealDigital.org
#
# Description:
#   Blackboard Rev. B pin assignments and constraints.
#
# History:
#   10/13/18: Initial release
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

# clk100_in is from the 100 MHz oscillator on Blackboard
# create_clock -period 10.000 -name gclk [get_ports clk100_in]
set_property -dict {PACKAGE_PIN H16 IOSTANDARD LVCMOS33} [get_ports clk100_in]

# Set false path from pxl_clk to AXI 100 MHz clock
set fclk0 [get_clocks -of_objects [get_pins "*/processing_system7_0/FCLK_CLK0"]]
set pxl_clk [get_clocks -of_objects [get_pins "*/clk_wiz/clk_out1"]]
set_clock_groups -name pxl_clk_async -asynchronous -group [get_clocks $fclk0] -group [get_clocks $pxl_clk]

## Pmod Header JA (XADC)
## Note that JA1P is rewired to left channel of PDM speaker
# set_property -dict {PACKAGE_PIN F16 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[0]}]
set_property -dict {PACKAGE_PIN F17 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[0]}]
set_property -dict {PACKAGE_PIN G19 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[1]}]
set_property -dict {PACKAGE_PIN G20 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[2]}]
set_property -dict {PACKAGE_PIN E18 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[3]}]
set_property -dict {PACKAGE_PIN E19 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[4]}]
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[5]}]
set_property -dict {PACKAGE_PIN D18 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[6]}]

## Pmod Header JB
set_property -dict {PACKAGE_PIN D19 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[7]}]
set_property -dict {PACKAGE_PIN D20 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[8]}]
set_property -dict {PACKAGE_PIN F19 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[9]}]
set_property -dict {PACKAGE_PIN F20 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[10]}]
set_property -dict {PACKAGE_PIN C20 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[11]}]
set_property -dict {PACKAGE_PIN B20 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[12]}]
set_property -dict {PACKAGE_PIN B19 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[13]}]
set_property -dict {PACKAGE_PIN A20 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[14]}]

# speaker pin
set_property PACKAGE_PIN G18 [get_ports {pdm_speaker_l}]
set_property IOSTANDARD LVCMOS33 [get_ports {pdm_speaker_l}]

# JA1P (PmodA 1) rewired to pdm_speaker_r
set_property PACKAGE_PIN F16 [get_ports {pdm_speaker_r}]
set_property IOSTANDARD LVCMOS33 [get_ports {pdm_speaker_r}]

# MIC pin
set_property PACKAGE_PIN L14 [get_ports {pdm_mic}]
set_property IOSTANDARD LVCMOS33 [get_ports {pdm_mic}]

set_property PACKAGE_PIN N15 [get_ports {pdm_mic_mclk}]
set_property IOSTANDARD LVCMOS33 [get_ports {pdm_mic_mclk}]

# 9-axis inertial module: LSM9DS1
# 3D Accelerometer, 3D gyroscope, 3D magnetometer
set_property -dict {PACKAGE_PIN H20 IOSTANDARD LVCMOS33} [get_ports {GYRO_SPC}];
set_property -dict {PACKAGE_PIN J19 IOSTANDARD LVCMOS33} [get_ports {GYRO_SDI}];
set_property -dict {PACKAGE_PIN J20 IOSTANDARD LVCMOS33} [get_ports {GYRO_SDO_AG}];
set_property -dict {PACKAGE_PIN L17 IOSTANDARD LVCMOS33} [get_ports {GYRO_SDO_M}];
set_property -dict {PACKAGE_PIN K17 IOSTANDARD LVCMOS33} [get_ports {GYRO_SS_AG}];
set_property -dict {PACKAGE_PIN K16 IOSTANDARD LVCMOS33} [get_ports {GYRO_SS_M}];
## Interrupts
set_property -dict {PACKAGE_PIN L20 IOSTANDARD LVCMOS33} [get_ports {GYRO_DRDY_M}];
set_property -dict {PACKAGE_PIN M20 IOSTANDARD LVCMOS33} [get_ports {GYRO_INT1_AG}];
set_property -dict {PACKAGE_PIN L19 IOSTANDARD LVCMOS33} [get_ports {GYRO_INT_M}];
## GYRO_DEN_AG is connected to PS_GPIO pin 23
set_property -dict {PACKAGE_PIN J14 IOSTANDARD LVCMOS33} [get_ports {PS_GPIO_tri_io[15]}];

# Switches
set_property PACKAGE_PIN R17 [get_ports sws_8bits_tri_i[0]]
set_property PACKAGE_PIN U20 [get_ports sws_8bits_tri_i[1]]
set_property PACKAGE_PIN R16 [get_ports sws_8bits_tri_i[2]]
set_property PACKAGE_PIN N16 [get_ports sws_8bits_tri_i[3]]
set_property PACKAGE_PIN R14 [get_ports sws_8bits_tri_i[4]]
set_property PACKAGE_PIN P14 [get_ports sws_8bits_tri_i[5]]
set_property PACKAGE_PIN L15 [get_ports sws_8bits_tri_i[6]]
set_property PACKAGE_PIN M15 [get_ports sws_8bits_tri_i[7]]
set_property IOSTANDARD LVCMOS33 [get_ports sws_8bits_tri_i[*]]

# Buttons
set_property PACKAGE_PIN W14 [get_ports btns_4bits_tri_i[0]]
set_property PACKAGE_PIN W13 [get_ports btns_4bits_tri_i[1]]
set_property PACKAGE_PIN P15 [get_ports btns_4bits_tri_i[2]]
set_property PACKAGE_PIN M14 [get_ports btns_4bits_tri_i[3]]
set_property IOSTANDARD LVCMOS33 [get_ports btns_4bits_tri_i[*]]

# LEDs
set_property PACKAGE_PIN N20 [get_ports leds_4bits_tri_o[0]]
set_property PACKAGE_PIN P20 [get_ports leds_4bits_tri_o[1]]
set_property PACKAGE_PIN R19 [get_ports leds_4bits_tri_o[2]]
set_property PACKAGE_PIN T20 [get_ports leds_4bits_tri_o[3]]
set_property IOSTANDARD LVCMOS33 [get_ports leds_4bits_tri_o[*]]

# RGB LEDs
set_property -dict {PACKAGE_PIN Y14 IOSTANDARD LVCMOS33} [get_ports {rgb_pwm_2[5]}]; #LD7_R
set_property -dict {PACKAGE_PIN Y16 IOSTANDARD LVCMOS33} [get_ports {rgb_pwm_2[4]}]; #LD7_G
set_property -dict {PACKAGE_PIN Y17 IOSTANDARD LVCMOS33} [get_ports {rgb_pwm_2[3]}]; #LD7_B
set_property -dict {PACKAGE_PIN W18 IOSTANDARD LVCMOS33} [get_ports {rgb_pwm_2[2]}]; #LD6_R
set_property -dict {PACKAGE_PIN W16 IOSTANDARD LVCMOS33} [get_ports {rgb_pwm_2[1]}]; #LD6_G
set_property -dict {PACKAGE_PIN Y18 IOSTANDARD LVCMOS33} [get_ports {rgb_pwm_2[0]}]; #LD6_B
set_property -dict {PACKAGE_PIN Y19 IOSTANDARD LVCMOS33} [get_ports {rgb_pwm_1[5]}]; #LD5_R
set_property -dict {PACKAGE_PIN V20 IOSTANDARD LVCMOS33} [get_ports {rgb_pwm_1[4]}]; #LD5_G
set_property -dict {PACKAGE_PIN W19 IOSTANDARD LVCMOS33} [get_ports {rgb_pwm_1[3]}]; #LD5_B
set_property -dict {PACKAGE_PIN U13 IOSTANDARD LVCMOS33} [get_ports {rgb_pwm_1[2]}]; #LD4_R
set_property -dict {PACKAGE_PIN T19 IOSTANDARD LVCMOS33} [get_ports {rgb_pwm_1[1]}]; #LD4_G
set_property -dict {PACKAGE_PIN W20 IOSTANDARD LVCMOS33} [get_ports {rgb_pwm_1[0]}]; #LD4_B

# Seven Segment Display
set_property -dict {PACKAGE_PIN K19 IOSTANDARD LVCMOS33} [get_ports {ssd_an[0]}]; #SSEG_AN0
set_property -dict {PACKAGE_PIN H17 IOSTANDARD LVCMOS33} [get_ports {ssd_an[1]}]; #SSEG_AN1
set_property -dict {PACKAGE_PIN M18 IOSTANDARD LVCMOS33} [get_ports {ssd_an[2]}]; #SSEG_AN2
set_property -dict {PACKAGE_PIN L16 IOSTANDARD LVCMOS33} [get_ports {ssd_an[3]}]; #SSEG_AN3

set_property -dict {PACKAGE_PIN K14 IOSTANDARD LVCMOS33} [get_ports {ssd_cathode[0]}]; #SSEG_CA
set_property -dict {PACKAGE_PIN H15 IOSTANDARD LVCMOS33} [get_ports {ssd_cathode[1]}]; #SSEG_CB
set_property -dict {PACKAGE_PIN J18 IOSTANDARD LVCMOS33} [get_ports {ssd_cathode[2]}]; #SSEG_CC
set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS33} [get_ports {ssd_cathode[3]}]; #SSEG_CD
set_property -dict {PACKAGE_PIN M17 IOSTANDARD LVCMOS33} [get_ports {ssd_cathode[4]}]; #SSEG_CE
set_property -dict {PACKAGE_PIN J16 IOSTANDARD LVCMOS33} [get_ports {ssd_cathode[5]}]; #SSEG_CF
set_property -dict {PACKAGE_PIN H18 IOSTANDARD LVCMOS33} [get_ports {ssd_cathode[6]}]; #SSEG_CG
set_property -dict {PACKAGE_PIN K18 IOSTANDARD LVCMOS33} [get_ports {ssd_dp}]; #SSEG_DP

# HDMI (DVI) outputs
set_property -dict {PACKAGE_PIN U19 IOSTANDARD TMDS_33} [get_ports hdmi_out_tmds_clk_n]
set_property -dict {PACKAGE_PIN U18 IOSTANDARD TMDS_33} [get_ports hdmi_out_tmds_clk_p]
set_property -dict {PACKAGE_PIN V18 IOSTANDARD TMDS_33} [get_ports {hdmi_out_tmds_data_n[0]}]
set_property -dict {PACKAGE_PIN V17 IOSTANDARD TMDS_33} [get_ports {hdmi_out_tmds_data_p[0]}]
set_property -dict {PACKAGE_PIN P18 IOSTANDARD TMDS_33} [get_ports {hdmi_out_tmds_data_n[1]}]
set_property -dict {PACKAGE_PIN N17 IOSTANDARD TMDS_33} [get_ports {hdmi_out_tmds_data_p[1]}]
set_property -dict {PACKAGE_PIN P19 IOSTANDARD TMDS_33} [get_ports {hdmi_out_tmds_data_n[2]}]
set_property -dict {PACKAGE_PIN N18 IOSTANDARD TMDS_33} [get_ports {hdmi_out_tmds_data_p[2]}]
set_property -dict {PACKAGE_PIN P16 IOSTANDARD LVCMOS33} [get_ports hdmi_hpd_tri_i[0]]

# HDMI I2C interface
set_property -dict {PACKAGE_PIN T17 IOSTANDARD LVCMOS33} [get_ports hdmi_ddc_scl_io]
set_property -dict {PACKAGE_PIN R18 IOSTANDARD LVCMOS33} [get_ports hdmi_ddc_sda_io]

# XADC Vp, Vn
set_property -dict {PACKAGE_PIN K9 IOSTANDARD LVCMOS33} [get_ports {vp_vn_v_p}]; 
set_property -dict {PACKAGE_PIN L10 IOSTANDARD LVCMOS33} [get_ports {vp_vn_v_n}]; 

# Servos
set_property -dict {PACKAGE_PIN G17 IOSTANDARD LVCMOS33} [get_ports servo_1];
set_property -dict {PACKAGE_PIN G15 IOSTANDARD LVCMOS33} [get_ports servo_2];
set_property -dict {PACKAGE_PIN G14 IOSTANDARD LVCMOS33} [get_ports servo_3];
set_property -dict {PACKAGE_PIN M19 IOSTANDARD LVCMOS33} [get_ports servo_4];

# Bitstream Compression
set_property BITSTREAM.GENERAL.COMPRESS true [current_design]
# Temporary fix for pulldown...
set_property BITSTREAM.CONFIG.UNUSEDPIN PULLUP [current_design]
