// Copyright (c) 2026 EAGLE Pi 6E Project
// SPDX-License-Identifier: BSD-3-Clause
//Timing Constraints file
//Tool Version: V1.9.12
// 27MHz 输入时钟
create_clock -name I_sys_clk -period 37.037 -waveform {0 18.518} [get_ports {I_sys_clk}]

// PPI 总线为异步输入，内部同步
set_false_path -from [get_ports {I_sram_cs_n I_sram_adv I_sram_wr_n I_sram_oe_n I_sram_lb_n I_sram_ub_n IO_sram_data[*]}]
set_false_path -to [get_ports {IO_sram_data[*]}]

// 按键复位
set_false_path -from [get_regs {sys_rst_n*}]

// MCU 复位输入
set_false_path -from [get_ports {I_mcu_rst_n}]
set_false_path -from [get_regs {mcu_rst_sync*}]

// 生成时钟：clk_psram 108MHz，clk_dma 54MHz，clk_tmds 185.625MHz，clk_pixel 37.125MHz
create_generated_clock -name clk_psram -source [get_ports {I_sys_clk}] -multiply_by 4 [get_pins {gowin_rpll_psram_inst/rpll_inst/CLKOUT}]
create_generated_clock -name clk_dma -source [get_pins {gowin_rpll_psram_inst/rpll_inst/CLKOUT}] -divide_by 2 [get_pins {PSRAM_Memory_Interface_HS_Top_inst/u_psram_top/clkdiv/CLKOUT}]
create_generated_clock -name clk_tmds -source [get_ports {I_sys_clk}] -multiply_by 55 -divide_by 8 [get_pins {u_tmds_pll/rpll_inst/CLKOUT}]
create_generated_clock -name clk_pixel -source [get_pins {u_tmds_pll/rpll_inst/CLKOUT}] -divide_by 5 [get_pins {u_clkdiv/CLKOUT}]

// 三组时钟之间经异步 FIFO 或同步器交互
set_clock_groups -asynchronous -group [get_clocks {I_sys_clk}] -group [get_clocks {clk_psram clk_dma}] -group [get_clocks {clk_tmds clk_pixel}]
