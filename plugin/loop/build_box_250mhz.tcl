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
if {$num_qdma > 1} {
    source box_250mhz/box_250mhz_axis_switch.tcl
}
read_verilog -quiet -sv { p2p_250mhz.sv loop_lite_slave.sv loop_registers.sv loop_ddr_ctrl.sv }
# read_xdc box_250mhz_to_ddr4.xdc
