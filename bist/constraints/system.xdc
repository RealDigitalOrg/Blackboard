# speaker pin
set_property PACKAGE_PIN G18 [get_ports pdm_speaker_l]
set_property IOSTANDARD LVCMOS33 [get_ports pdm_speaker_l]

# JA1P (PmodA 1) rewired to pdm_speaker_r
set_property PACKAGE_PIN F16 [get_ports pdm_speaker_r]
set_property IOSTANDARD LVCMOS33 [get_ports pdm_speaker_r]

# MIC pin
set_property PACKAGE_PIN L14 [get_ports pdm_mic]
set_property IOSTANDARD LVCMOS33 [get_ports pdm_mic]

set_property PACKAGE_PIN N15 [get_ports pdm_mic_mclk]
set_property IOSTANDARD LVCMOS33 [get_ports pdm_mic_mclk]

# ESP32 UART1
set_property PACKAGE_PIN U14 [get_ports esp32_uart1_rxd]
set_property IOSTANDARD LVCMOS33 [get_ports esp32_uart1_rxd]

set_property PACKAGE_PIN U15 [get_ports esp32_uart1_txd]
set_property IOSTANDARD LVCMOS33 [get_ports esp32_uart1_txd]

# Switches
set_property PACKAGE_PIN R17 [get_ports sws_12bits_tri_i[0]]
set_property PACKAGE_PIN U20 [get_ports sws_12bits_tri_i[1]]
set_property PACKAGE_PIN R16 [get_ports sws_12bits_tri_i[2]]
set_property PACKAGE_PIN N16 [get_ports sws_12bits_tri_i[3]]
set_property PACKAGE_PIN R14 [get_ports sws_12bits_tri_i[4]]
set_property PACKAGE_PIN P14 [get_ports sws_12bits_tri_i[5]]
set_property PACKAGE_PIN L15 [get_ports sws_12bits_tri_i[6]]
set_property PACKAGE_PIN M15 [get_ports sws_12bits_tri_i[7]]
set_property PACKAGE_PIN T10 [get_ports sws_12bits_tri_i[8]]
set_property PACKAGE_PIN T12 [get_ports sws_12bits_tri_i[9]]
set_property PACKAGE_PIN T11 [get_ports sws_12bits_tri_i[10]]
set_property PACKAGE_PIN T14 [get_ports sws_12bits_tri_i[11]]
set_property IOSTANDARD LVCMOS33 [get_ports sws_12bits_tri_i[*]]

# Buttons
set_property PACKAGE_PIN W14 [get_ports btns_4bits_tri_i[0]]
set_property PACKAGE_PIN W13 [get_ports btns_4bits_tri_i[1]]
set_property PACKAGE_PIN P15 [get_ports btns_4bits_tri_i[2]]
set_property PACKAGE_PIN M14 [get_ports btns_4bits_tri_i[3]]
set_property IOSTANDARD LVCMOS33 [get_ports btns_4bits_tri_i[*]]

# LEDs
set_property PACKAGE_PIN N20 [get_ports leds_10bits_tri_o[0]]
set_property PACKAGE_PIN P20 [get_ports leds_10bits_tri_o[1]]
set_property PACKAGE_PIN R19 [get_ports leds_10bits_tri_o[2]]
set_property PACKAGE_PIN T20 [get_ports leds_10bits_tri_o[3]]
set_property PACKAGE_PIN T19 [get_ports leds_10bits_tri_o[4]]
set_property PACKAGE_PIN U13 [get_ports leds_10bits_tri_o[5]]
set_property PACKAGE_PIN V20 [get_ports leds_10bits_tri_o[6]]
set_property PACKAGE_PIN W20 [get_ports leds_10bits_tri_o[7]]
set_property PACKAGE_PIN W19 [get_ports leds_10bits_tri_o[8]]
set_property PACKAGE_PIN Y19 [get_ports leds_10bits_tri_o[9]]
set_property IOSTANDARD LVCMOS33 [get_ports leds_10bits_tri_o[*]]

# Bitstream Compression
set_property BITSTREAM.GENERAL.COMPRESS true [current_design]
# Temporary fix for pulldown...
set_property BITSTREAM.CONFIG.UNUSEDPIN PULLUP [current_design]
