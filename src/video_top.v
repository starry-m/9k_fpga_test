// Copyright (c) 2026 EAGLE Pi 6E Project
// SPDX-License-Identifier: BSD-3-Clause
// ============================================================================
// video_top：EAGLE Pi 6E FPGA 顶层（GW1NR-LV9QN88PC6/I5）
// 接收 HPM6E80 经 PPI 推送的 RGB565 图像，经 PSRAM 帧缓冲后输出 HDMI。
// 与 Web APP 配套，PPI 协议版本 0x1000，默认 1280x720 @ 30Hz。
//
// 数据流：ppi_video_in / testpattern → Video_Frame_Buffer（PSRAM）→ DVI_TX → HDMI
// 时钟：27MHz 输入；PSRAM 108MHz；TMDS 串行时钟为像素时钟的 5 倍
// 图像源：上电为 testpattern，收到 MCU 的首个 SOF 后切到 PPI；按键在两者间切换
// ============================================================================

// 与 IP 配置一致
`define WR_VIDEO_WIDTH_16        // 16/24/32
`define DEF_WR_VIDEO_WIDTH 16    // 16/24/32

`define RD_VIDEO_WIDTH_16        // 16/24/32
`define DEF_RD_VIDEO_WIDTH 16    // 16/24/32

`define USE_THREE_FRAME_BUFFER

`define DEF_ADDR_WIDTH 21
`define DEF_SRAM_DATA_WIDTH 64

// ============================================================================
// 视频分辨率，只定义一项。修改后需同步调整 Gowin_rPLL_TMDS 的输出频率
// （像素时钟 × 5）和 SDC 中 clk_tmds 的倍频系数。
//   1080P@30 : 像素 74.25MHz， TMDS 371.25MHz（27 × 55/4）
//   720P@30  : 像素 37.125MHz，TMDS 185.625MHz（27 × 55/8）
//   480P@60  : 像素 25.175MHz，TMDS 125.875MHz
//   360P@60  : 像素 18MHz，    TMDS 90MHz（27 × 10/3）
// ============================================================================
//`define VIDEO_PROFILE_1080P30
`define VIDEO_PROFILE_720P30
//`define VIDEO_PROFILE_480P60
//`define VIDEO_PROFILE_360P60

`ifndef VIDEO_PROFILE_1080P30
`ifndef VIDEO_PROFILE_720P30
`ifndef VIDEO_PROFILE_480P60
`ifndef VIDEO_PROFILE_360P60
`define VIDEO_PROFILE_1080P30   // 未选择任何档时默认 1080P@30
`endif
`endif
`endif
`endif

`ifdef VIDEO_PROFILE_1080P30
    // 1920×1080 @30Hz（CEA 时序，与 1080p60 相同总像素，像素时钟减半）
    // 像素时钟 74.25MHz；五倍时钟(PLL clkout，÷5 前) 371.25MHz
    `define VP_H_TOTAL   2200
    `define VP_H_SYNC    44
    `define VP_H_BPORCH  148
    `define VP_H_RES     1920
    `define VP_V_TOTAL   1125
    `define VP_V_SYNC    5
    `define VP_V_BPORCH  36
    `define VP_V_RES     1080
    `define VP_RD_HRES   1920
    `define VP_RD_VRES   1080
    `define VP_HS_POL    1'b1
    `define VP_VS_POL    1'b1
`elsif VIDEO_PROFILE_720P30
    // 1280×720 @30Hz（CEA 720p 结构，像素时钟减半）
    // 像素时钟 37.125MHz；五倍时钟 185.625MHz
    `define VP_H_TOTAL   1650
    `define VP_H_SYNC    40
    `define VP_H_BPORCH  220
    `define VP_H_RES     1280
    `define VP_V_TOTAL   750
    `define VP_V_SYNC    5
    `define VP_V_BPORCH  20
    `define VP_V_RES     720
    `define VP_RD_HRES   1280
    `define VP_RD_VRES   720
    `define VP_HS_POL    1'b1
    `define VP_VS_POL    1'b1
`elsif VIDEO_PROFILE_480P60
    // 640×480 @60Hz（VESA DMT ID 4，负同步）
    // 像素时钟 25.175MHz；五倍时钟 125.875MHz
    `define VP_H_TOTAL   800
    `define VP_H_SYNC    96
    `define VP_H_BPORCH  48
    `define VP_H_RES     640
    `define VP_V_TOTAL   525
    `define VP_V_SYNC    2
    `define VP_V_BPORCH  33
    `define VP_V_RES     480
    `define VP_RD_HRES   640
    `define VP_RD_VRES   480
    `define VP_HS_POL    1'b0
    `define VP_VS_POL    1'b0
`elsif VIDEO_PROFILE_360P60
    // 640×360 @60Hz（简化行场，便于由 27MHz 派生 18MHz 像素时钟）
    // 像素时钟 18MHz；五倍时钟 90MHz
    `define VP_H_TOTAL   800
    `define VP_H_SYNC    96
    `define VP_H_BPORCH  48
    `define VP_H_RES     640
    `define VP_V_TOTAL   375
    `define VP_V_SYNC    3
    `define VP_V_BPORCH  11
    `define VP_V_RES     360
    `define VP_RD_HRES   640
    `define VP_RD_VRES   360
    `define VP_HS_POL    1'b0
    `define VP_VS_POL    1'b0
`endif

// ============================================================================
// 模块定义
// ============================================================================
module video_top
(
    // 系统时钟和复位
    input             I_sys_clk,          // 27MHz 系统时钟
    input             I_sys_rst_n,        // 系统复位，低有效
    input             I_mcu_rst_n,        // MCU 复位线（CTR6=PE02 → 50 脚），低有效

    // 按键输入
    input             I_key_n,            // 按键：切换图像源（PPI ↔ testpattern）

    // PPI 异步 SRAM 从机接口，地址 / 数据复用 16 位
    input             I_sram_cs_n,        // 片选，低有效
    input             I_sram_adv,         // 地址有效，高有效
    input             I_sram_wr_n,        // 写使能，低有效
    input             I_sram_oe_n,        // 读使能，低有效
    input             I_sram_lb_n,        // 字节使能（未用）
    input             I_sram_ub_n,        // 字节使能（未用）
    inout      [15:0] IO_sram_data,       // AD 复用总线
    
    // LED状态指示
    output     [7:0]  O_led,              // LED输出

    // 测试信号
    output      reg   O_test,

    // PSRAM接口信号
    output     [1:0]  O_psram_ck,         // PSRAM时钟
    output     [1:0]  O_psram_ck_n,       // PSRAM时钟（反相）
    inout      [1:0]  IO_psram_rwds,      // PSRAM读写数据选通
    inout      [15:0] IO_psram_dq,        // PSRAM数据总线
    output     [1:0]  O_psram_reset_n,    // PSRAM复位
    output     [1:0]  O_psram_cs_n,       // PSRAM片选
    
    // HDMI/TMDS输出信号
    output            O_tmds_clk_p,       // TMDS时钟正
    output            O_tmds_clk_n,       // TMDS时钟负
    output     [2:0]  O_tmds_data_p,      // TMDS数据正 {r,g,b}
    output     [2:0]  O_tmds_data_n       // TMDS数据负 {r,g,b}
);

// ============================================================================
// 参数定义
// ============================================================================
parameter ADDR_WIDTH       = `DEF_ADDR_WIDTH;        // 地址位宽
parameter DATA_WIDTH       = `DEF_SRAM_DATA_WIDTH;   // PSRAM IP 用户数据位宽
parameter WR_VIDEO_WIDTH   = `DEF_WR_VIDEO_WIDTH;     // 写入视频数据位宽
parameter RD_VIDEO_WIDTH   = `DEF_RD_VIDEO_WIDTH;     // 读出视频数据位宽

// ============================================================================
// 信号声明 - 系统控制
// ============================================================================
reg         sys_rst_n = 1'd1; // 系统复位（按键翻转）
reg  [1:0]  mcu_rst_sync = 2'b11; // MCU 复位线两级同步（27MHz 域）
wire        rst_all_n;        // 按键复位 与 MCU 复位 合并
reg  [31:0] run_cnt;          // 运行计数器
wire        running;          // 运行状态标志

// ============================================================================
// 信号声明 - 时钟和复位
// ============================================================================
wire        memory_clk;       // PSRAM工作时钟 (108MHz)
wire        mem_pll_lock;     // PSRAM PLL锁定信号
wire        dma_clk;          // DMA工作时钟 (54MHz)
wire        hdmi_serial_clk;  // TMDS串行时钟（≈像素×5，随VIDEO_PROFILE变）
wire        hdmi_pll_lock;    // TMDS PLL锁定信号
wire        hdmi_pixel_clk;   // 像素时钟（由所选 VIDEO_PROFILE 决定，见文件头部宏说明）
wire        hdmi_rst_n;       // HDMI复位信号

// ============================================================================
// 信号声明 - 视频帧缓冲输入
// ============================================================================
wire                   ch0_vfb_clk_in;             // 视频帧缓冲输入时钟
wire                   ch0_vfb_vs_in;              // 视频帧缓冲垂直同步（负极性）
wire                   ch0_vfb_de_in;              // 视频帧缓冲数据使能
wire [WR_VIDEO_WIDTH-1:0] ch0_vfb_data_in;         // 视频帧缓冲输入数据

// ============================================================================
// 信号声明 - 视频帧缓冲输出
// ============================================================================
wire                   off0_syn_de;                 // 输出FIFO数据使能
wire [RD_VIDEO_WIDTH-1:0] off0_syn_data;            // 输出FIFO数据

// ============================================================================
// 信号声明 - 同步信号生成
// ============================================================================
wire                   syn_off0_re;                 // 输出FIFO读使能信号
wire                   syn_off0_vs;                 // 同步输出垂直同步
wire                   syn_off0_hs;                 // 同步输出水平同步
wire                   out_de;                      // 同步输出数据使能

localparam N = 5;                                   // 延迟时钟数
reg  [N-1:0]  Pout_hs_dn;                           // 水平同步延迟链
reg  [N-1:0]  Pout_vs_dn;                           // 垂直同步延迟链
reg  [N-1:0]  Pout_de_dn;                           // 数据使能延迟链

// ============================================================================
// 信号声明 - PSRAM内存接口
// ============================================================================
wire                    cmd;                        // 命令信号 (0:读, 1:写)
wire                    cmd_en;                     // 命令使能
wire [ADDR_WIDTH-1:0]   addr;                       // 地址信号
wire [DATA_WIDTH-1:0]   wr_data;                    // 写数据
wire [DATA_WIDTH/8-1:0] data_mask;                  // 数据掩码
wire                    rd_data_valid;              // 读数据有效
wire [DATA_WIDTH-1:0]   rd_data;                    // 读数据
wire                    init_calib;                 // 初始化校准完成

// ============================================================================
// 信号声明 - RGB数据
// ============================================================================
wire        rgb_vs;                                  // RGB垂直同步
wire        rgb_hs;                                  // RGB水平同步
wire        rgb_de;                                  // RGB数据使能
wire [23:0] rgb_data;                                // RGB数据 {r[7:0], g[7:0], b[7:0]}

// ============================================================================
// LED状态指示（低电平点亮）
// ============================================================================
// [7]=PSRAM 校准完成，[6]=当前源（亮=PPI）
// [5:0]：收到 SOF 前为 PPI 信号活动标志 [0]CS [1]ADV [2]WE [3]OE [4]寄存器区 [5]读驱动，
//        收到 SOF 后为帧计数低 6 位
assign O_led[7]   = ~init_calib;
assign O_led[6]   = ~source_is_ppi;
assign O_led[5:0] = ppi_active ? ~ppi_frame_cnt[5:0] : ~ppi_dbg_stat;

// ============================================================================
// 时钟生成 - PSRAM时钟域
// ============================================================================
//  27MHz ──▶ [PLL ×4] ──▶ 108MHz (memory_clk)
//
Gowin_rPLL_PSRAM gowin_rpll_psram_inst
(
    .clkin    (I_sys_clk),                            // 输入时钟 27MHz
    .clkout   (memory_clk),                           // 输出时钟 108MHz
    .reset_p  (~rst_all_n),
    .lock     (mem_pll_lock)                          // PLL锁定信号
);

// ============================================================================
// 时钟生成 - TMDS/HDMI时钟域
// ============================================================================
//  27MHz ──▶ [PLL] ──▶ hdmi_serial_clk（≈ 像素时钟×5）──▶ [CLKDIV ÷5] ──▶ hdmi_pixel_clk
//
Gowin_rPLL_TMDS u_tmds_pll
(
    .clkin    (I_sys_clk),                            // 输入时钟 27MHz
    .clkout   (hdmi_serial_clk),                      // 输出时钟 ≈ 5×像素时钟（随 PROFILE 变）
    .reset_p  (~rst_all_n),
    .lock     (hdmi_pll_lock)                         // PLL锁定信号
);

// HDMI复位信号生成
assign hdmi_rst_n = rst_all_n & hdmi_pll_lock;

// 像素时钟分频 (hdmi_serial_clk / 5 = 当前模式的像素时钟)
CLKDIV u_clkdiv
(
    .RESETN (hdmi_rst_n),
    .HCLKIN (hdmi_serial_clk),                      // 高频输入时钟（PLL 输出，≈5×像素）
    .CLKOUT (hdmi_pixel_clk),                       // 输出时钟 x1（像素时钟）
    .CALIB  (1'b1)
);

defparam u_clkdiv.DIV_MODE = "5";
defparam u_clkdiv.GSREN    = "false";

// ============================================================================
// 测试图像生成模块（testpattern）
// ============================================================================
wire        testpattern_de;              // 测试图像数据使能
wire        testpattern_hs;              // 测试图像水平同步
wire        testpattern_vs;              // 测试图像垂直同步
wire [7:0]  testpattern_data_r;          // 测试图像红色数据
wire [7:0]  testpattern_data_g;          // 测试图像绿色数据
wire [7:0]  testpattern_data_b;          // 测试图像蓝色数据

testpattern testpattern_inst
(
    .I_pxl_clk   (I_sys_clk),                       // 像素时钟 27MHz
    .I_rst_n     (hdmi_rst_n),                      // 复位信号，低有效
    .I_mode      ({1'b0,cnt_vs[6:5]} ),             // 测试模式: 000=color bar, 001=net grid, 010=gray, 011=black-white square, 111=single color
    .I_sqr_width (16'd64),                          // 棋盘格宽度
    .I_single_r  (8'd0),                            // 单色模式红色值
    .I_single_g  (8'd0),                            // 单色模式绿色值
    .I_single_b  (8'd0),                            // 单色模式蓝色值
    // 水平/垂直时序（由 VIDEO_PROFILE_xxx 宏定义）
    .I_h_total   (16'd`VP_H_TOTAL),
    .I_h_sync    (16'd`VP_H_SYNC),
    .I_h_bporch  (16'd`VP_H_BPORCH),
    .I_h_res     (16'd`VP_H_RES),
    .I_v_total   (16'd`VP_V_TOTAL),
    .I_v_sync    (16'd`VP_V_SYNC),
    .I_v_bporch  (16'd`VP_V_BPORCH),
    .I_v_res     (16'd`VP_V_RES),
    // 同步极性
    .I_hs_pol    (`VP_HS_POL),
    .I_vs_pol    (`VP_VS_POL),
    // 输出信号
    .O_de        (testpattern_de),                  // 数据使能
    .O_hs        (testpattern_hs),                  // 水平同步
    .O_vs        (testpattern_vs),                  // 垂直同步
    .O_data_r    (testpattern_data_r),              // 红色数据
    .O_data_g    (testpattern_data_g),              // 绿色数据
    .O_data_b    (testpattern_data_b)               // 蓝色数据
);

reg         vs_r;
reg  [9:0]  cnt_vs;

always@(posedge I_sys_clk)
begin
    vs_r <= testpattern_vs;
end

always@(posedge I_sys_clk or negedge hdmi_rst_n)
begin
    if(!hdmi_rst_n)
        cnt_vs <= 10'd0;
    else if(vs_r && !testpattern_vs) //vs24 falling edge
        cnt_vs <= cnt_vs+1'b1;
end 

// ============================================================================
// PPI 图像接收模块（MCU → 像素流）
// ============================================================================
//  PPI 采样使用 memory_clk（108MHz），输出 27MHz 视频流
//
wire        ppi_vs_n;                    // PPI 源帧起始负脉冲
wire        ppi_de;                      // PPI 源像素有效
wire [15:0] ppi_data;                    // PPI 源 RGB565
wire        ppi_active;                  // 收到过 SOF
wire [7:0]  ppi_frame_cnt;               // PPI 收帧计数
wire        vfb_vin_fifo_full;           // VFB 输入 FIFO 满，反压组帧器
wire [5:0]  ppi_dbg_stat;                // PPI 信号活动标志（LED）

ppi_video_in #(
    .FRAME_PIXELS (`VP_H_RES * `VP_V_RES),
    .H_RES        (`VP_H_RES),               // 供 MCU 读回的输出分辨率，跟随 profile
    .V_RES        (`VP_V_RES)
) ppi_video_in_inst
(
    // 108MHz PPI 采样
    .I_clk_ppi    (memory_clk),
    .I_rst_ppi_n  (rst_all_n & mem_pll_lock),
    .I_cs_n       (I_sram_cs_n),
    .I_adv        (I_sram_adv),
    .I_we_n       (I_sram_wr_n),
    .I_oe_n       (I_sram_oe_n),
    .I_lb_n       (I_sram_lb_n),
    .I_ub_n       (I_sram_ub_n),
    .IO_sram_data (IO_sram_data),
    // 27MHz 组帧输出
    .I_vin_clk       (I_sys_clk),
    .I_vin_rst_n     (hdmi_rst_n),
    .I_vfb_fifo_full (vfb_vin_fifo_full),
    .O_vs_n       (ppi_vs_n),
    .O_de         (ppi_de),
    .O_data       (ppi_data),
    .O_active     (ppi_active),
    .O_frame_cnt  (ppi_frame_cnt),
    .O_dbg_stat   (ppi_dbg_stat)
);

// ============================================================================
// 图像源切换：按键在 testpattern 与 PPI 之间切换；PPI 停止推送时保持最后一帧
// ============================================================================
reg  [19:0] key_dbn_cnt;                 // 消抖计数（约 20ms @27MHz）
reg         key_stable;                  // 消抖后电平（高=未按下）
reg         key_stable_r;
reg         force_testpattern;           // 按键强制 testpattern 模式
wire        source_is_ppi = ppi_active && !force_testpattern;

always @(posedge I_sys_clk or negedge hdmi_rst_n)
begin
    if(!hdmi_rst_n)
    begin
        key_dbn_cnt       <= 20'd0;
        key_stable        <= 1'b1;
        key_stable_r      <= 1'b1;
        force_testpattern <= 1'b0;
    end
    else
    begin
        if(I_key_n == key_stable)
            key_dbn_cnt <= 20'd0;
        else if(key_dbn_cnt == 20'd540_000)   // 20ms 持续反电平才翻转
        begin
            key_dbn_cnt <= 20'd0;
            key_stable  <= I_key_n;
        end
        else
            key_dbn_cnt <= key_dbn_cnt + 1'b1;

        key_stable_r <= key_stable;
        if(key_stable_r && !key_stable)       // 按下沿：强制/自动互切
            force_testpattern <= ~force_testpattern;
    end
end

// ============================================================================
// 视频数据格式转换 - 写入帧缓冲
// ============================================================================
//  源 mux：PPI 像素流（RGB565 直通）/ testpattern（RGB888 → RGB565）
//
`ifdef WR_VIDEO_WIDTH_16
    assign ch0_vfb_clk_in  = I_sys_clk;              // 两源同为 27MHz 域
    assign ch0_vfb_vs_in   = source_is_ppi ? ppi_vs_n : ~testpattern_vs;
    assign ch0_vfb_de_in   = source_is_ppi ? ppi_de   : testpattern_de;
    assign ch0_vfb_data_in = source_is_ppi ? ppi_data :
                             {testpattern_data_r[7:3], testpattern_data_g[7:2], testpattern_data_b[7:3]}; // RGB565
`endif

`ifdef WR_VIDEO_WIDTH_24
    assign ch0_vfb_clk_in  = I_sys_clk;              // 使用系统时钟
    assign ch0_vfb_vs_in   = ~testpattern_vs;        // 负极性，取反
    assign ch0_vfb_de_in   = testpattern_de;         // 数据使能，直接连接
    assign ch0_vfb_data_in = {testpattern_data_r, testpattern_data_g, testpattern_data_b}; // RGB888 {R,G,B}
`endif

`ifdef WR_VIDEO_WIDTH_32
    assign ch0_vfb_clk_in  = I_sys_clk;              // 使用系统时钟
    assign ch0_vfb_vs_in   = ~testpattern_vs;        // 负极性，取反
    assign ch0_vfb_de_in   = testpattern_de;         // 数据使能，直接连接
    assign ch0_vfb_data_in = {8'd0, testpattern_data_r, testpattern_data_g, testpattern_data_b}; // RGB888 + Alpha {A,R,G,B}
`endif

// ============================================================================
// 视频帧缓冲模块
// ============================================================================
//  写入侧(27MHz) ──▶ [异步FIFO] ──▶ DMA(54MHz) ──▶ PSRAM
//                                                  │
//  读出侧(像素时钟) ◀── [异步FIFO] ◀── DMA(54MHz) ◀──┘
//
Video_Frame_Buffer_Top Video_Frame_Buffer_Top_inst
(
    // 复位和时钟
    .I_rst_n     (init_calib && rst_all_n),         // 复位信号（等待PSRAM初始化完成）
    .I_dma_clk   (dma_clk),                         // DMA时钟 (54MHz)
    
    // 帧缓冲控制
`ifdef USE_THREE_FRAME_BUFFER
    .I_wr_halt   (1'd0),                            // 写入暂停: 1=暂停, 0=不暂停
    .I_rd_halt   (1'd0),                            // 读出暂停: 1=暂停, 0=不暂停
`endif
`ifndef USE_THREE_FRAME_BUFFER
    .I_wr_halt   (1'd0),                            // 写入暂停: 1=暂停, 0=不暂停
    .I_rd_halt   (1'd0),                            // 读出暂停: 1=暂停, 0=不暂停
`endif
    
    // 视频数据输入 (27MHz时钟域)
    .I_vin0_clk       (ch0_vfb_clk_in),              // 输入时钟
    .I_vin0_vs_n      (ch0_vfb_vs_in),               // 垂直同步（负极性）
    .I_vin0_de        (ch0_vfb_de_in),               // 数据使能
    .I_vin0_data      (ch0_vfb_data_in),             // 输入数据
    .O_vin0_fifo_full (vfb_vin_fifo_full),          // FIFO满标志，反压 PPI 组帧器
    
    // 视频数据输出 (hdmi_pixel_clk 时钟域)
    .I_vout0_clk      (hdmi_pixel_clk),               // 输出时钟
    .I_vout0_vs_n     (~syn_off0_vs),                 // 垂直同步（负极性）
    .I_vout0_de       (syn_off0_re),                  // 读使能
    .O_vout0_den      (off0_syn_de),                  // 数据使能输出
    .O_vout0_data     (off0_syn_data),                // 输出数据
    .O_vout0_fifo_empty(),                            // FIFO空标志（未使用）
    
    // 内存接口 (54MHz时钟域)
    .O_cmd            (cmd),                          // 命令: 0=读, 1=写
    .O_cmd_en         (cmd_en),                       // 命令使能
    .O_addr           (addr),                         // 地址
    .O_wr_data        (wr_data),                      // 写数据
    .O_data_mask      (data_mask),                    // 数据掩码
    .I_rd_data_valid  (rd_data_valid),                // 读数据有效
    .I_rd_data        (rd_data),                      // 读数据
    .I_init_calib     (init_calib)                    // 初始化校准完成
);

// ============================================================================
// PSRAM内存接口模块
// ============================================================================
//  Video_Frame_Buffer ──▶ [PSRAM接口] ──▶ PSRAM物理接口 (108MHz)
//         │                    │
//         │                    └──▶ dma_clk输出
//         └──▶ init_calib (初始化完成信号)
//
PSRAM_Memory_Interface_HS_Top PSRAM_Memory_Interface_HS_Top_inst
(
    // 时钟和复位
    .clk        (I_sys_clk),                         // 系统时钟 27MHz
    .memory_clk (memory_clk),                        // PSRAM工作时钟 108MHz
    .pll_lock   (mem_pll_lock),                      // PLL锁定信号
    .rst_n      (sys_rst_n),                         // 复位信号
    
    // PSRAM物理接口
    .O_psram_ck      (O_psram_ck),                   // PSRAM时钟
    .O_psram_ck_n    (O_psram_ck_n),                 // PSRAM时钟（反相）
    .IO_psram_rwds   (IO_psram_rwds),                // 读写数据选通
    .IO_psram_dq     (IO_psram_dq),                  // 数据总线
    .O_psram_reset_n (O_psram_reset_n),              // 复位信号
    .O_psram_cs_n    (O_psram_cs_n),                 // 片选信号
    
    // 内存接口信号
    .wr_data         (wr_data),                      // 写数据
    .rd_data         (rd_data),                      // 读数据
    .rd_data_valid   (rd_data_valid),                // 读数据有效
    .addr            (addr),                         // 地址
    .cmd             (cmd),                          // 命令
    .cmd_en          (cmd_en),                       // 命令使能
    .data_mask       (data_mask),                    // 数据掩码
    .clk_out         (dma_clk),                      // DMA时钟输出
    .init_calib      (init_calib)                    // 初始化校准完成
);

// ============================================================================
// 同步信号生成模块
// ============================================================================
//  生成符合视频标准的HS/VS/DE信号，用于控制帧缓冲读出时序
//  hdmi_pixel_clk ──▶ [时序生成] ──▶ HS/VS/DE + 读使能
//
syn_gen syn_gen_inst
(
    .I_pxl_clk   (hdmi_pixel_clk),                  // 像素时钟（VIDEO_PROFILE）
    .I_rst_n     (hdmi_rst_n),                      // 复位信号
    .I_h_total   (16'd`VP_H_TOTAL),
    .I_h_sync    (16'd`VP_H_SYNC),
    .I_h_bporch  (16'd`VP_H_BPORCH),
    .I_h_res     (16'd`VP_H_RES),
    .I_v_total   (16'd`VP_V_TOTAL),
    .I_v_sync    (16'd`VP_V_SYNC),
    .I_v_bporch  (16'd`VP_V_BPORCH),
    .I_v_res     (16'd`VP_V_RES),
    .I_rd_hres   (16'd`VP_RD_HRES),
    .I_rd_vres   (16'd`VP_RD_VRES),
    .I_hs_pol    (`VP_HS_POL),
    .I_vs_pol    (`VP_VS_POL),
    // 输出信号
    .O_rden      (syn_off0_re),                     // 读使能
    .O_de        (out_de),                          // 数据使能
    .O_hs        (syn_off0_hs),                     // 水平同步
    .O_vs        (syn_off0_vs)                      // 垂直同步
);

// ============================================================================
// 同步信号延迟对齐
// ============================================================================
//  通过延迟链对齐HS/VS/DE信号，确保与RGB数据同步
//  syn_gen输出 ──▶ [延迟链 N=5] ──▶ 对齐后的HS/VS/DE
//
always @(posedge hdmi_pixel_clk or negedge hdmi_rst_n)
begin
    if(!hdmi_rst_n)
    begin
        Pout_hs_dn <= {N{1'b1}};
        Pout_vs_dn <= {N{1'b1}};
        Pout_de_dn <= {N{1'b0}};
    end
    else
    begin
        Pout_hs_dn <= {Pout_hs_dn[N-2:0], syn_off0_hs};
        Pout_vs_dn <= {Pout_vs_dn[N-2:0], syn_off0_vs};
        Pout_de_dn <= {Pout_de_dn[N-2:0], out_de};
    end
end

// ============================================================================
// RGB数据格式转换 - 从帧缓冲输出
// ============================================================================
//  Video_Frame_Buffer输出 ──▶ [格式转换] ──▶ RGB888 ──▶ DVI_TX
//
`ifdef RD_VIDEO_WIDTH_16
    // RGB565转RGB888
    assign rgb_data = off0_syn_de ? {off0_syn_data[15:11], 3'd0, 
                                      off0_syn_data[10:5], 2'd0, 
                                      off0_syn_data[4:0], 3'd0} : 24'h00007f;
    assign rgb_vs   = Pout_vs_dn[1];
    assign rgb_hs   = Pout_hs_dn[1];
    assign rgb_de   = Pout_de_dn[1];
`endif

`ifdef RD_VIDEO_WIDTH_24
    // RGB888直通
    assign rgb_data = off0_syn_de ? off0_syn_data : 24'h00007f;
    assign rgb_vs   = Pout_vs_dn[4];
    assign rgb_hs   = Pout_hs_dn[4];
    assign rgb_de   = Pout_de_dn[4];
`endif

`ifdef RD_VIDEO_WIDTH_32
    // RGB888 + Alpha，取低24位
    assign rgb_data = off0_syn_de ? off0_syn_data[23:0] : 24'h00007f;
    assign rgb_vs   = Pout_vs_dn[1];
    assign rgb_hs   = Pout_hs_dn[1];
    assign rgb_de   = Pout_de_dn[1];
`endif

// ============================================================================
// DVI/HDMI发送模块
// ============================================================================
//  RGB数据(像素时钟) ──▶ [TMDS编码] ──▶ 串行数据(hdmi_serial_clk) ──▶ HDMI输出
//
DVI_TX_Top DVI_TX_Top_inst
(
    .I_rst_n      (hdmi_rst_n),                      // 复位信号，低有效
    .I_serial_clk (hdmi_serial_clk),                 // 串行时钟   5x
    .I_rgb_clk    (hdmi_pixel_clk),                  // RGB像素时钟1x
    .I_rgb_vs     (rgb_vs),                          // RGB垂直同步
    .I_rgb_hs     (rgb_hs),                          // RGB水平同步
    .I_rgb_de     (rgb_de),                          // RGB数据使能
    .I_rgb_r      (rgb_data[23:16]),                 // RGB红色数据
    .I_rgb_g      (rgb_data[15: 8]),                 // RGB绿色数据
    .I_rgb_b      (rgb_data[ 7: 0]),                 // RGB蓝色数据
    .O_tmds_clk_p (O_tmds_clk_p),                    // TMDS时钟正
    .O_tmds_clk_n (O_tmds_clk_n),                    // TMDS时钟负
    .O_tmds_data_p(O_tmds_data_p),                   // TMDS数据正 {r,g,b}
    .O_tmds_data_n(O_tmds_data_n)                    // TMDS数据负 {r,g,b}
);


// ============================================================================
// 复位信号
// ============================================================================

reg[31:0] reset_count = 32'd0;
reg key_value = 0;

// MCU 复位线同步后与按键复位合并
always @ (posedge I_sys_clk)
    mcu_rst_sync <= {mcu_rst_sync[0], I_mcu_rst_n};

assign rst_all_n = sys_rst_n & mcu_rst_sync[1];

always @ (posedge I_sys_clk)
    if (reset_count > 2_700_000)
        begin  
            reset_count <= 0;
            if (key_value != I_sys_rst_n && !I_sys_rst_n) 
                sys_rst_n <= ~sys_rst_n;
            key_value <= I_sys_rst_n;
        end
    else
        reset_count <= reset_count + 1;


reg[31:0] test_count = 32'd0;
always @ (posedge I_sys_clk)
    if (test_count >= 5)
        begin  
            test_count <= 0;
            O_test <= ~O_test;
        end
    else
        test_count <= test_count + 1;

endmodule
