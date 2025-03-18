# *************************************************************************
#
# Copyright 2020 Xilinx, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# *************************************************************************
create_clock -period 10.000 -name pcie_refclk [get_ports pcie_refclk_p]

set_false_path -through [get_ports pcie_rstn]

foreach axis_aclk [get_clocks -of_object [get_nets axis_aclk*]] {
    foreach cmac_clk [get_clocks -of_object [get_nets cmac_clk*]] {
        set_max_delay -datapath_only -from $axis_aclk -to $cmac_clk 4.000
        set_max_delay -datapath_only -from $cmac_clk -to $axis_aclk 3.103 
    }
}

create_pblock pblock_packet_adapter_tx
add_cells_to_pblock [get_pblocks pblock_packet_adapter_tx] [get_cells -quiet {cmac_port*.packet_adapter_inst/tx_inst}]
resize_pblock [get_pblocks pblock_packet_adapter_tx] -add {CLOCKREGION_X1Y8:CLOCKREGION_X2Y8}

create_pblock pblock_packet_adapter_rx
add_cells_to_pblock [get_pblocks pblock_packet_adapter_rx] [get_cells -quiet {cmac_port*.packet_adapter_inst/rx_inst}]
resize_pblock [get_pblocks pblock_packet_adapter_rx] -add {CLOCKREGION_X5Y8:CLOCKREGION_X6Y8}

# DDR Controller
# System clocks
# 300 MHz (DDR 0)
set_property -dict {LOC AY37 IOSTANDARD LVDS} [get_ports clk_in1_p]
set_property -dict {LOC AY38 IOSTANDARD LVDS} [get_ports clk_in1_n]

create_clock -period 3.333 -name clk_in1 [get_ports clk_in1_p]

# CDC datapath - constrain maximum delay.
set ddr4clk [ get_clocks -of_object [ get_pins box_250mhz_inst/dummy_reg_inst/loop_registers_inst/l_ddr4_reg_rdata_reg[187]/C ] ]
set boxclk [ get_clocks -of_object [ get_pins box_250mhz_inst/dummy_reg_inst/loop_registers_inst/sync_ddr4_reg_rdata_reg[187]/C ] ]
set_max_delay -datapath_only -from $ddr4clk -to $boxclk 8.000
set_max_delay -datapath_only -from $boxclk -to $ddr4clk 8.000
 
# 300 MHz (DDR 1)
#set_property -dict {LOC AW20 IOSTANDARD LVDS} [get_ports clk_300mhz_1_p]
#set_property -dict {LOC AW19 IOSTANDARD LVDS} [get_ports clk_300mhz_1_n]
#create_clock -period 3.333 -name clk_300mhz_1 [get_ports clk_300mhz_1_p]

# 300 MHz (DDR 2)
#set_property -dict {LOC F32  IOSTANDARD LVDS} [get_ports clk_300mhz_2_p]
#set_property -dict {LOC E32  IOSTANDARD LVDS} [get_ports clk_300mhz_2_n]
#create_clock -period 3.333 -name clk_300mhz_2 [get_ports clk_300mhz_2_p]

# 300 MHz (DDR 3)
#set_property -dict {LOC J16  IOSTANDARD LVDS} [get_ports clk_300mhz_3_p]
#set_property -dict {LOC H16  IOSTANDARD LVDS} [get_ports clk_300mhz_3_n]
#create_clock -period 3.333 -name clk_300mhz_3 [get_ports clk_300mhz_3_p]