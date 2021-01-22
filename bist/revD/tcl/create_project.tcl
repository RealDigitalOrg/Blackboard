###############################################################################
# File: create_project.tcl
# Author: Tinghui Wang
#
# Copyright (c) 2018-2019, RealDigital.org
#
# Description:
#   Vivado tcl script to generate BIST project for BlackBoard.
#
# History:
#   10/13/18: Initial release
#   04/20/19: Update system design for BlackBoard Rev. D.
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

namespace eval _tcl {
    proc get_script_folder {} {
	    set script_path [file normalize [info script]]
	    set script_folder [file dirname $script_path]
        return $script_folder
    }
}
variable script_folder
set script_folder [_tcl::get_script_folder]
puts $script_folder

variable script_file
set script_file "create_project.tcl"

################################################################
# START
################################################################

set ip_repo_path        $script_folder/../../../ip_repo
set bd_name             {system}
set device              {xc7z007sclg400-1}
set project_name        {bist}
set project_dir         {bist_proj}

# Help information for this script
proc help {} {
  global script_file
  puts "\nDescription:"
  puts "Create Vivado BIST project for Blackboard rev. D."
  puts "Syntax:"
  puts "$script_file -tclargs \[--project_name <name>\]"
  puts "$script_file -tclargs \[--project_dir <path\]"
  puts "$script_file -tclargs \[--help\]"
  puts "Usage:"
  puts "Name                   Description"
  puts "------------------------------------------------------------------"
  puts "\[--project_name <name>\] Create project with the specified name."
  puts "                       Default name is \"bist\"."
  puts "\[--project_dir <path>\]  Determine the project paths wrt to \".\"."
  puts "                       Default project path value is \"./bist_proj\"."
  puts "\[--help\]               Print help information for this script"
  puts "------------------------------------------------------------------\n" 
  exit 0
}

if { $::argc > 0 } {
  for {set i 0} {$i < [llength $::argc]} {incr i} {
    set option [string trim [lindex $::argv $i]]
    switch -regexp -- $option {
      "--project_dir"   { incr i; set project_dir [lindex $::argv $i] }
      "--project_name" { incr i; set project_name [lindex $::argv $i] }
      "--help"         { help }
      default {
        if { [regexp {^-} $option] } {
          puts "ERROR: Unknown option '$option' specified, please type '$script_file -tclargs --help' for usage info.\n"
          return 1
        }
      }
    }
  }
}

# Create project
set list_projs [get_projects -quiet]
if { $list_projs eq "" } {
    create_project $project_name ./$project_dir -part $device
}

# Set IP Repository
puts ${ip_repo_path}
set_property ip_repo_paths ${ip_repo_path} [current_fileset]
update_ip_catalog -rebuild

# Change design name
variable design_name
set design_name "system"

# Create block design
set current_bd [create_bd_design $design_name]
set parentCell [get_bd_cells /]

proc apply_ps_presets { cell presetFile } {
    source $presetFile
    set presets [apply_preset 0]
    foreach {k v} $presets {
        if { ![info exists preset_list] } {
            set preset_list [dict create $k $v]
        } else {
            dict set preset_list $k $v
        }
    }

    set_property -dict $preset_list $cell 
}

proc create_ps { parentCell cellName } {
    global script_folder
    # Add Zynq processing system
    set ps_cell [ create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7 $cellName ]
    apply_ps_presets $ps_cell $script_folder/../../../presets/BlackBoard_revD_ps_presets.tcl
    set_property -dict [list \
        CONFIG.PCW_USE_S_AXI_HP0 {1} \
        CONFIG.PCW_GPIO_EMIO_GPIO_ENABLE {1} \
        CONFIG.PCW_GPIO_EMIO_GPIO_IO {24} \
        CONFIG.PCW_SPI0_PERIPHERAL_ENABLE {1} \
        CONFIG.PCW_TTC0_PERIPHERAL_ENABLE {1} \
    ] $ps_cell
    apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 -config {make_external "FIXED_IO, DDR" Master "Disable" Slave "Disable" } $ps_cell 
    set PS_GPIO [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:gpio_rtl:1.0 PS_GPIO ]
    connect_bd_intf_net -intf_net processing_system7_0_GPIO_0 [get_bd_intf_ports PS_GPIO] [get_bd_intf_pins processing_system7_0/GPIO_0]
}

# Create processing system
create_ps $parentCell "processing_system7_0"

# Servo connections
set servo_1 [ create_bd_port -dir O servo_1 ]
set servo_2 [ create_bd_port -dir O servo_2 ]
set servo_3 [ create_bd_port -dir O servo_3 ]
connect_bd_net -net ttc0_wave0_out_servo_1 $servo_1 [ get_bd_pins processing_system7_0/TTC0_WAVE0_OUT ]
connect_bd_net -net ttc0_wave1_out_servo_2 $servo_2 [ get_bd_pins processing_system7_0/TTC0_WAVE1_OUT ]
connect_bd_net -net ttc0_wave2_out_servo_3 $servo_3 [ get_bd_pins processing_system7_0/TTC0_WAVE2_OUT ]

# Create instance: pwm_servo_4 and set properties
set pwm_servo_4 [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_timer pwm_servo_4 ]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/processing_system7_0/M_AXI_GP0} } [get_bd_intf_pins pwm_servo_4/S_AXI] 
set servo_4 [ create_bd_port -dir O servo_4 ]
connect_bd_net -net pwm_servo_4_pwm0 $servo_4 [ get_bd_pins pwm_servo_4/pwm0 ]


# Create instance: LSM9DS1_SPI_Bridge and set properties
set lsm9ds1_spi_bridge [ create_bd_cell -type ip -vlnv realdigital.org:realdigital:LSM9DS1_SPI_Bridge:1.0 LSM9DS1_SPI_Bridge ]
connect_bd_intf_net [get_bd_intf_pins processing_system7_0/SPI_0] [get_bd_intf_pins LSM9DS1_SPI_Bridge/SPI]
set GYRO_SPC [ create_bd_port -dir IO GYRO_SPC ]
set GYRO_SDI [ create_bd_port -dir IO GYRO_SDI ]
set GYRO_SDO_AG [ create_bd_port -dir I GYRO_SDO_AG ]
set GYRO_SDO_M [ create_bd_port -dir I GYRO_SDO_M ]
set GYRO_SS_AG [ create_bd_port -dir IO GYRO_SS_AG ]
set GYRO_SS_M [ create_bd_port -dir IO GYRO_SS_M ]
set GYRO_DRDY_M [ create_bd_port -type intr -dir I GYRO_DRDY_M ]
set GYRO_INT1_AG [ create_bd_port -type intr -dir I GYRO_INT1_AG ]
set GYRO_INT_M [ create_bd_port -type intr -dir I GYRO_INT_M ]
connect_bd_net -net LSM9DS1_SPI_Bridge_SPC [get_bd_pins LSM9DS1_SPI_Bridge/SCK_AGM] $GYRO_SPC
connect_bd_net -net LSM9DS1_SPI_Bridge_SDI [get_bd_pins LSM9DS1_SPI_Bridge/MOSI_AGM] $GYRO_SDI
connect_bd_net -net LSM9DS1_SPI_Bridge_SDO_AG [get_bd_pins LSM9DS1_SPI_Bridge/MISO_AG] $GYRO_SDO_AG
connect_bd_net -net LSM9DS1_SPI_Bridge_SDO_M [get_bd_pins LSM9DS1_SPI_Bridge/MISO_M] $GYRO_SDO_M
connect_bd_net -net LSM9DS1_SPI_Bridge_SS_AG [get_bd_pins LSM9DS1_SPI_Bridge/SS_AG] $GYRO_SS_AG
connect_bd_net -net LSM9DS1_SPI_Bridge_SS_M [get_bd_pins LSM9DS1_SPI_Bridge/SS_M] $GYRO_SS_M

# Create instance: gpio_btn and set properties
set gpio_btn [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio gpio_btn ]
set_property -dict [ list \
    CONFIG.C_ALL_INPUTS {1} \
    CONFIG.C_GPIO_WIDTH {4} \
    CONFIG.C_INTERRUPT_PRESENT {1} \
] $gpio_btn
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/processing_system7_0/M_AXI_GP0} } [get_bd_intf_pins gpio_btn/S_AXI] 
set btns_4bits [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:gpio_rtl:1.0 btns_4bits ]
connect_bd_intf_net -intf_net gpio_btn_GPIO [get_bd_intf_ports btns_4bits] [get_bd_intf_pins gpio_btn/GPIO]

# Create instance: gpio_led and set properties  
set gpio_led [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio gpio_led ]
set_property -dict [ list \
    CONFIG.C_ALL_OUTPUTS {1} \
    CONFIG.C_GPIO_WIDTH {10} \
    CONFIG.C_INTERRUPT_PRESENT {1} \
] $gpio_led
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/processing_system7_0/M_AXI_GP0} } [get_bd_intf_pins gpio_led/S_AXI] 
set leds_10bits [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:gpio_rtl:1.0 leds_10bits ]
connect_bd_intf_net -intf_net gpio_led_GPIO [get_bd_intf_ports leds_10bits] [get_bd_intf_pins gpio_led/GPIO]

# Create instance: gpio_sw and set properties
set gpio_sw [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio gpio_sw ]
set_property -dict [ list \
    CONFIG.C_ALL_INPUTS {1} \
    CONFIG.C_GPIO_WIDTH {12} \
    CONFIG.C_INTERRUPT_PRESENT {1} \
] $gpio_sw
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/processing_system7_0/M_AXI_GP0} } [get_bd_intf_pins gpio_sw/S_AXI] 
set sws_12bits [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:gpio_rtl:1.0 sws_12bits ]
connect_bd_intf_net -intf_net gpio_sw_btn_GPIO [get_bd_intf_ports sws_12bits] [get_bd_intf_pins gpio_sw/GPIO]

# Create instance: rgb_pwm, and set properties
set rgb_pwm [ create_bd_cell -type ip -vlnv realdigital.org:realdigital:axi_pwm rgb_pwm ]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/processing_system7_0/M_AXI_GP0} } [get_bd_intf_pins rgb_pwm/S_AXI]
set rgb_pwm_pins [ create_bd_port -from 5 -to 0 -dir O rgb_pwm ]
connect_bd_net -net net_rgb_pwm $rgb_pwm_pins [get_bd_pins rgb_pwm/pwm] 

# Create instance: seven_seg_display, and set properties
set seven_seg_display [create_bd_cell -type ip -vlnv realdigital.org:realdigital:seven_seg_display:1.0 seven_seg_display]
set_property -dict [ list \
    CONFIG.C_ANODE_POLARITY {ACTIVE_LOW} \
] $seven_seg_display
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/processing_system7_0/M_AXI_GP0} } [get_bd_intf_pins seven_seg_display/S_AXI]
set ssd_an [ create_bd_port -from 3 -to 0 -dir O ssd_an ]
set ssd_cathode [ create_bd_port -from 6 -to 0 -dir O ssd_cathode ]
set ssd_dp [ create_bd_port -dir O ssd_dp ]
connect_bd_net -net net_ssd_an $ssd_an [get_bd_pins seven_seg_display/anode]
connect_bd_net -net net_ssd_cathode $ssd_cathode [get_bd_pins seven_seg_display/cathode]
connect_bd_net -net net_ssd_dp $ssd_dp [get_bd_pins seven_seg_display/dp]

# Create instance: esp32_uart1, and set properties
set esp32_uart1 [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_uartlite esp32_uart1 ]
set_property -dict [ list \
    CONFIG.C_BAUDRATE {115200} \
] $esp32_uart1
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/processing_system7_0/M_AXI_GP0} } [get_bd_intf_pins esp32_uart1/S_AXI] 
set esp32_uart1 [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:uart_rtl:1.0 esp32_uart1 ]
connect_bd_intf_net -intf_net esp32_uart1_UART [get_bd_intf_ports esp32_uart1] [get_bd_intf_pins esp32_uart1/UART]

# Create instance: pdm_audio, and set properties
set pdm_audio [ create_bd_cell -type ip -vlnv realdigital.org:realdigital:pdm_audio:1.0 pdm_audio ]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/processing_system7_0/M_AXI_GP0} } [get_bd_intf_pins pdm_audio/S_AXI] 
set pdm_mic [ create_bd_port -dir I pdm_mic ]
set pdm_mic_mclk [ create_bd_port -dir O pdm_mic_mclk ]
set pdm_speaker_l [ create_bd_port -dir IO pdm_speaker_l ]
set pdm_speaker_r [ create_bd_port -dir IO pdm_speaker_r ]
connect_bd_net -net net_pdm_speaker_r [get_bd_ports pdm_speaker_r] [get_bd_pins pdm_audio/pdm_speaker_r]
connect_bd_net -net net_pdm_speaker_l [get_bd_ports pdm_speaker_l] [get_bd_pins pdm_audio/pdm_speaker_l]
connect_bd_net -net net_pdm_mic_mclk [get_bd_ports pdm_mic_mclk] [get_bd_pins pdm_audio/pdm_mic_mclk]
connect_bd_net -net net_pdm_mic_data [get_bd_ports pdm_mic] [get_bd_pins pdm_audio/pdm_mic]
connect_bd_net -net processing_system7_0_FCLK_CLK1 [get_bd_pins pdm_audio/pdm_mclk] [get_bd_pins processing_system7_0/FCLK_CLK1]

# HDMI Video Subsystem
# Create instance: gpio_hdmi, and set properties
set gpio_hdmi [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio gpio_hdmi ]
set_property -dict [ list \
    CONFIG.C_GPIO_WIDTH {1} \
    CONFIG.C_ALL_INPUTS {1} \
    CONFIG.C_INTERRUPT_PRESENT {1} \
] $gpio_hdmi
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/processing_system7_0/M_AXI_GP0} } [get_bd_intf_pins gpio_hdmi/S_AXI] 
set hdmi_hpd [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:gpio_rtl:1.0 hdmi_hpd ]
connect_bd_intf_net -intf_net gpio_hdmi_GPIO [get_bd_intf_ports hdmi_hpd] [get_bd_intf_pins gpio_hdmi/GPIO]

# Create instance: iic_hdmi, and set properties
set iic_hdmi [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_iic iic_hdmi ]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/processing_system7_0/M_AXI_GP0} } [get_bd_intf_pins iic_hdmi/S_AXI] 
set hdmi_ddc [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:iic_rtl:1.0 hdmi_ddc ]
connect_bd_intf_net -intf_net iic_hdmi_IIC [get_bd_intf_ports hdmi_ddc] [get_bd_intf_pins iic_hdmi/IIC]

# Create instance: hdmi_vdma, and set properties
set hdmi_vdma [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_vdma hdmi_vdma ]
set_property -dict [ list \
   CONFIG.c_include_mm2s_dre {0} \
   CONFIG.c_include_s2mm {0} \
   CONFIG.c_mm2s_genlock_mode {0} \
   CONFIG.c_mm2s_linebuffer_depth {2048} \
   CONFIG.c_mm2s_max_burst_length {16} \
   CONFIG.c_num_fstores {1} \
   CONFIG.c_s2mm_genlock_mode {0} \
] $hdmi_vdma
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/processing_system7_0/M_AXI_GP0} } [get_bd_intf_pins hdmi_vdma/S_AXI_LITE] 
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/hdmi_vdma/M_AXI_MM2S} Slave {/processing_system7_0/S_AXI_HP0} intc_ip {Auto} master_apm {0}}  [get_bd_intf_pins processing_system7_0/S_AXI_HP0]
connect_bd_net [get_bd_pins hdmi_vdma/m_axis_mm2s_aclk] [get_bd_pins processing_system7_0/FCLK_CLK0]

# Create instance: hdmi_axi4s_vid_out, and set properties 
set hdmi_axi4s_vid_out [ create_bd_cell -type ip -vlnv xilinx.com:ip:v_axi4s_vid_out hdmi_axi4s_vid_out ]
set_property -dict [ list \
   CONFIG.C_ADDR_WIDTH {12} \
   CONFIG.C_HAS_ASYNC_CLK {1} \
   CONFIG.C_S_AXIS_VIDEO_FORMAT {6} \
   CONFIG.C_VTG_MASTER_SLAVE {1} \
] $hdmi_axi4s_vid_out
connect_bd_intf_net -intf_net axi_vdma_0_M_AXIS_MM2S [get_bd_intf_pins hdmi_vdma/M_AXIS_MM2S] [get_bd_intf_pins hdmi_axi4s_vid_out/video_in]
connect_bd_net [get_bd_pins hdmi_axi4s_vid_out/aclk] [get_bd_pins processing_system7_0/FCLK_CLK0]

# Create instance: hdmi_v_tc, and set properties
set hdmi_v_tc [ create_bd_cell -type ip -vlnv xilinx.com:ip:v_tc hdmi_v_tc ]
set_property -dict [ list \
   CONFIG.enable_detection {false} \
] $hdmi_v_tc
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/processing_system7_0/M_AXI_GP0} } [get_bd_intf_pins hdmi_v_tc/ctrl] 
connect_bd_intf_net -intf_net hdmi_v_tc_vtiming_out [get_bd_intf_pins hdmi_axi4s_vid_out/vtiming_in] [get_bd_intf_pins hdmi_v_tc/vtiming_out]
connect_bd_net -net hdmi_axi4s_vid_out_vtg_ce [get_bd_pins hdmi_axi4s_vid_out/vtg_ce] [get_bd_pins hdmi_v_tc/gen_clken]

# Create instance: hdmi_tx, and set properties
set hdmi_tx [ create_bd_cell -type ip -vlnv realdigital.org:realdigital:hdmi_tx hdmi_tx ]
set hdmi_out [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:hdmi_rtl:2.0 hdmi_out ]
connect_bd_intf_net -intf_net hdmi_tx_0_hdmi_tx [get_bd_intf_ports hdmi_out] [get_bd_intf_pins hdmi_tx/hdmi_tx]
connect_bd_intf_net -intf_net v_axi4s_vid_out_0_vid_io_out [get_bd_intf_pins hdmi_tx/RGB] [get_bd_intf_pins hdmi_axi4s_vid_out/vid_io_out]

# Create instance: clk_wiz, and set properties
set clk_wiz [ create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz clk_wiz ]
set_property -dict [ list \
   CONFIG.AXI_DRP {false} \
   CONFIG.CLKOUT1_DRIVES {BUFG} \
   CONFIG.CLKOUT1_JITTER {337.616} \
   CONFIG.CLKOUT1_PHASE_ERROR {322.999} \
   CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {74.25} \
   CONFIG.CLKOUT2_DRIVES {BUFG} \
   CONFIG.CLKOUT2_JITTER {258.703} \
   CONFIG.CLKOUT2_PHASE_ERROR {322.999} \
   CONFIG.CLKOUT2_REQUESTED_OUT_FREQ {371.25} \
   CONFIG.CLKOUT2_USED {true} \
   CONFIG.CLKOUT3_DRIVES {BUFG} \
   CONFIG.CLKOUT4_DRIVES {BUFG} \
   CONFIG.CLKOUT5_DRIVES {BUFG} \
   CONFIG.CLKOUT6_DRIVES {BUFG} \
   CONFIG.CLKOUT7_DRIVES {BUFG} \
   CONFIG.CLK_IN1_BOARD_INTERFACE {Custom} \
   CONFIG.MMCM_CLKFBOUT_MULT_F {37.125} \
   CONFIG.MMCM_CLKIN1_PERIOD {10.0} \
   CONFIG.MMCM_CLKIN2_PERIOD {10.0} \
   CONFIG.MMCM_CLKOUT0_DIVIDE_F {10.000} \
   CONFIG.MMCM_CLKOUT1_DIVIDE {2.00} \
   CONFIG.MMCM_DIVCLK_DIVIDE {5} \
   CONFIG.NUM_OUT_CLKS {2} \
   CONFIG.PHASE_DUTY_CONFIG {false} \
   CONFIG.SECONDARY_SOURCE {Single_ended_clock_capable_pin} \
   CONFIG.USE_BOARD_FLOW {true} \
   CONFIG.USE_DYN_RECONFIG {true} \
   CONFIG.USE_PHASE_ALIGNMENT {false} \
] $clk_wiz
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/processing_system7_0/M_AXI_GP0} } [get_bd_intf_pins clk_wiz/s_axi_lite]
connect_bd_net -net pxl_clk [get_bd_pins clk_wiz/clk_out1] [get_bd_pins hdmi_tx/pix_clk] [get_bd_pins hdmi_axi4s_vid_out/vid_io_out_clk] [get_bd_pins hdmi_v_tc/clk]
connect_bd_net -net pxl_clk5 [get_bd_pins clk_wiz/clk_out2] [get_bd_pins hdmi_tx/pix_clkx5]
connect_bd_net -net clk_wiz_locked [get_bd_pins clk_wiz/locked] [get_bd_pins hdmi_tx/pix_clk_locked]
set clk100_in [ create_bd_port -dir I -type clk clk100_in ]
connect_bd_net [get_bd_pins clk_wiz/clk_in1] $clk100_in 

# Create instance: xadc
set xadc_wiz [ create_bd_cell -type ip -vlnv xilinx.com:ip:xadc_wiz xadc_wiz ]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/processing_system7_0/M_AXI_GP0} } [get_bd_intf_pins xadc_wiz/s_axi_lite]
set vp_vn [ create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_analog_io_rtl:1.0 vp_vn ]
connect_bd_intf_net -intf_net xadc_wiz_vp_vn [get_bd_intf_pins xadc_wiz/Vp_Vn] $vp_vn

# Create instance: intr_concat, and set properties
set intr_concat [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat intr_concat ]
set_property -dict [ list \
    CONFIG.NUM_PORTS {16} \
] $intr_concat
connect_bd_net -net pdm_audio_0_spkLIntr [get_bd_pins intr_concat/In0] [get_bd_pins pdm_audio/spkLIntr]
connect_bd_net -net pdm_audio_0_spkRIntr [get_bd_pins intr_concat/In1] [get_bd_pins pdm_audio/spkRIntr]
connect_bd_net -net pdm_audio_0_micIntr [get_bd_pins intr_concat/In2] [get_bd_pins pdm_audio/micIntr]
connect_bd_net -net esp32_uart1_irpt [get_bd_pins esp32_uart1/interrupt] [get_bd_pins intr_concat/In3]
connect_bd_net -net gpio_sw_ip2intc_irpt [get_bd_pins gpio_sw/ip2intc_irpt] [get_bd_pins intr_concat/In4]
connect_bd_net -net gpio_led_ip2intc_irpt [get_bd_pins gpio_led/ip2intc_irpt] [get_bd_pins intr_concat/In5]
connect_bd_net -net gpio_btn_ip2intc_irpt [get_bd_pins gpio_btn/ip2intc_irpt] [get_bd_pins intr_concat/In6]
connect_bd_net -net gpio_hdmi_ip2intc_irpt [get_bd_pins gpio_hdmi/ip2intc_irpt] [get_bd_pins intr_concat/In7]
connect_bd_net -net iic_hdmi_iic2intc_irpt [get_bd_pins iic_hdmi/iic2intc_irpt] [get_bd_pins intr_concat/In8]
connect_bd_net -net hdmi_vdma_mm2s_introut [get_bd_pins hdmi_vdma/mm2s_introut] [get_bd_pins intr_concat/In9]
connect_bd_net -net hdmi_v_tc_irq [get_bd_pins hdmi_v_tc/irq] [get_bd_pins intr_concat/In10]
connect_bd_net -net gyro_drdy_m_irq $GYRO_DRDY_M [get_bd_pins intr_concat/In11]
connect_bd_net -net gyro_int1_ag_irq $GYRO_INT1_AG [get_bd_pins intr_concat/In12]
connect_bd_net -net gyro_int_m_irq $GYRO_INT_M [get_bd_pins intr_concat/In13]
connect_bd_net -net xadc_wiz_ip2intc_irpt [get_bd_pins xadc_wiz/ip2intc_irpt] [get_bd_pins intr_concat/In14]
connect_bd_net -net pwm_servo_4_irpt [ get_bd_pins pwm_servo_4/interrupt ] [get_bd_pins intr_concat/In15]
connect_bd_net -net xlconcat_0_dout [get_bd_pins intr_concat/dout] [get_bd_pins processing_system7_0/IRQ_F2P]

# Save block design
save_bd_design
set bd_filename [get_property FILE_NAME [current_bd_design]]

# Close block design
close_bd_design $current_bd

# Disable OOC
set_property synth_checkpoint_mode None [get_files $bd_filename]

# Create HDL wrapper
set wrapper_file [make_wrapper -files [get_files $bd_filename] -top]

# Create source set
if {[string equal [get_filesets -quiet sources_1] ""]} {
    create_fileset -srcset sources_1
}
add_files -fileset sources_1 $wrapper_file

# Add constraints
if {[string equal [get_filesets -quiet constrs_1] ""]} {
    create_fileset -constrset constrs_1
}
set obj [get_filesets constrs_1]
set file "[file normalize "$script_folder/../constraints/system.xdc"]"
set file_imported [import_files -fileset constrs_1 [list $file]]

puts ""
puts "############################################"
puts "# Implementation Starts"
puts "############################################"
puts ""

# Launch implementation and bitstream generation
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

puts ""
puts "############################################"
puts "# Implementation Done"
puts "############################################"
puts ""


puts ""
puts "############################################"
puts "# Exporting Hardware"
puts "############################################"
puts ""
write_hw_platform -fixed -force  -include_bit -file $project_dir/$project_name.xsa
