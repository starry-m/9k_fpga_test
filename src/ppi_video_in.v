// Copyright (c) 2026 EAGLE Pi 6E Project
// SPDX-License-Identifier: BSD-3-Clause
// ============================================================================
// ppi_video_in：PPI 异步 SRAM 从机，接收 MCU 推送的 RGB565 图像并输出视频流
//
// 地址（16 位字地址，FPGA 锁存 DQ[13:0]）：
//   addr[13]=0  数据区
//     写：写入测试 RAM；视频模式下同时作为一个像素进入 FIFO
//     读：测试 RAM 对应地址
//   addr[13]=1  寄存器区（addr[2:0] 译码）
//     写 0：SOF，开始新的一帧
//     写 1：模式，bit0=1 测速模式（数据区写不产生像素），bit0=0 视频模式
//     读 0：帧计数
//     读 1：本帧已收像素数 [15:0]
//     读 2：{sof_seen, fifo_ovf, mode_test, 9'b0, 本帧已收像素数[19:16]}
//     读 3：版本号 FW_VERSION
//     读 4：水平分辨率
//     读 5：垂直分辨率
//
// 每帧须推送 FRAME_PIXELS 个像素；收到 SOF 时，不足一帧的部分补零。
// ============================================================================
module ppi_video_in #(
    parameter        FRAME_PIXELS = 921600,       // 1280 x 720
    parameter [15:0] H_RES        = 16'd1280,
    parameter [15:0] V_RES        = 16'd720
)(
    // PPI 采样时钟域（108MHz）
    input  wire        I_clk_ppi,
    input  wire        I_rst_ppi_n,

    // PPI 总线
    input  wire        I_cs_n,
    input  wire        I_adv,
    input  wire        I_we_n,
    input  wire        I_oe_n,
    input  wire        I_lb_n,                   // 未使用
    input  wire        I_ub_n,                   // 未使用
    inout  wire [15:0] IO_sram_data,             // 地址 / 数据复用

    // 视频输出时钟域（27MHz）
    input  wire        I_vin_clk,
    input  wire        I_vin_rst_n,
    input  wire        I_vfb_fifo_full,          // VFB 输入 FIFO 满

    output reg         O_vs_n,                   // 帧起始，低有效
    output reg         O_de,
    output reg  [15:0] O_data,                   // RGB565
    output reg         O_active,                 // 已收到 SOF
    output reg  [7:0]  O_frame_cnt,
    output wire [5:0]  O_dbg_stat                // 总线信号活动标志
);

    // ========================================================================
    // PPI 前端（108MHz）
    // ========================================================================
    reg [2:0]  cs_sync;
    reg [2:0]  adv_sync;
    reg [2:0]  we_sync;
    reg [2:0]  oe_sync;
    reg [15:0] dq_sync1;
    reg [15:0] dq_sync2;

    always @(posedge I_clk_ppi or negedge I_rst_ppi_n) begin
        if (!I_rst_ppi_n) begin
            cs_sync  <= 3'b111;
            adv_sync <= 3'b000;
            we_sync  <= 3'b111;
            oe_sync  <= 3'b111;
            dq_sync1 <= 16'd0;
            dq_sync2 <= 16'd0;
        end
        else begin
            cs_sync  <= {cs_sync[1:0],  I_cs_n};
            adv_sync <= {adv_sync[1:0], I_adv};
            we_sync  <= {we_sync[1:0],  I_we_n};
            oe_sync  <= {oe_sync[1:0],  I_oe_n};
            dq_sync1 <= IO_sram_data;
            dq_sync2 <= dq_sync1;
        end
    end

    wire cs_active  = ~cs_sync[1];
    wire adv_active =  adv_sync[1];
    wire we_active  = ~we_sync[1];
    wire oe_active  = ~oe_sync[1];

    // WE 上升沿提交写入
    wire we_rise = ~we_sync[2] & we_sync[1];

    // ADV 有效期间锁存地址
    reg [13:0] addr_latch;

    always @(posedge I_clk_ppi or negedge I_rst_ppi_n) begin
        if (!I_rst_ppi_n)
            addr_latch <= 14'd0;
        else if (cs_active && adv_active)
            addr_latch <= dq_sync2[13:0];
    end

    // WE 有效期间采样写数据
    reg [15:0] wr_data_hold;

    always @(posedge I_clk_ppi or negedge I_rst_ppi_n) begin
        if (!I_rst_ppi_n)
            wr_data_hold <= 16'd0;
        else if (cs_active && we_active)
            wr_data_hold <= dq_sync2;
    end

    // 写译码；收到首个 SOF 之前的像素丢弃
    reg region_data_q;
    reg region_pix_q;
    reg region_sof_q;
    reg region_mode_q;

    always @(posedge I_clk_ppi or negedge I_rst_ppi_n) begin
        if (!I_rst_ppi_n) begin
            region_data_q <= 1'b0;
            region_pix_q  <= 1'b0;
            region_sof_q  <= 1'b0;
            region_mode_q <= 1'b0;
        end
        else begin
            region_data_q <= !addr_latch[13];
            region_pix_q  <= !addr_latch[13] && sof_seen && !mode_test;
            region_sof_q  <= addr_latch[13] && (addr_latch[2:0] == 3'd0);
            region_mode_q <= addr_latch[13] && (addr_latch[2:0] == 3'd1);
        end
    end

    wire wr_data_region = we_rise && region_data_q;
    wire wr_pixel = we_rise && region_pix_q;
    wire wr_sof   = we_rise && region_sof_q;
    wire wr_mode  = we_rise && region_mode_q;

    // 模式寄存器，复位后为视频模式
    reg mode_test;

    always @(posedge I_clk_ppi or negedge I_rst_ppi_n) begin
        if (!I_rst_ppi_n)
            mode_test <= 1'b0;
        else if (wr_mode)
            mode_test <= wr_data_hold[0];
    end

    // 测试 RAM：8K x 16，用于总线读写校验
    reg [15:0] tram [0:8191];
    reg [15:0] tram_rd;
    reg [15:0] tram_rd_q;

    always @(posedge I_clk_ppi) begin
        if (wr_data_region)
            tram[addr_latch[12:0]] <= wr_data_hold;
        tram_rd   <= tram[addr_latch[12:0]];
        tram_rd_q <= tram_rd;
    end

    // 状态计数
    reg [15:0] frame_cnt_ppi;
    reg [19:0] pix_rcv_cnt;
    reg        sof_seen;
    reg        fifo_ovf;                         // 像素 FIFO 溢出，SOF 清零
    reg        sof_toggle;                       // SOF 跨时钟域翻转信号

    wire fifo_wr_full;
    wire fifo_wr_en = wr_pixel && !fifo_wr_full;

    always @(posedge I_clk_ppi or negedge I_rst_ppi_n) begin
        if (!I_rst_ppi_n) begin
            frame_cnt_ppi <= 16'd0;
            pix_rcv_cnt   <= 20'd0;
            sof_seen      <= 1'b0;
            fifo_ovf      <= 1'b0;
            sof_toggle    <= 1'b0;
        end
        else if (wr_sof) begin
            frame_cnt_ppi <= frame_cnt_ppi + 1'b1;
            pix_rcv_cnt   <= 20'd0;
            sof_seen      <= 1'b1;
            fifo_ovf      <= 1'b0;
            sof_toggle    <= ~sof_toggle;
        end
        else if (wr_pixel) begin
            if (fifo_wr_full)
                fifo_ovf <= 1'b1;
            else
                pix_rcv_cnt <= pix_rcv_cnt + 1'b1;
        end
    end

    // 读回
    localparam [15:0] FW_VERSION = 16'h1000;     // V1.0.0

    reg [15:0] reg_rd;
    reg [15:0] rd_data;

    always @(posedge I_clk_ppi) begin
        case (addr_latch[2:0])
            3'd0: reg_rd <= frame_cnt_ppi;
            3'd1: reg_rd <= pix_rcv_cnt[15:0];
            3'd2: reg_rd <= {sof_seen, fifo_ovf, mode_test, 9'd0, pix_rcv_cnt[19:16]};
            3'd3: reg_rd <= FW_VERSION;
            3'd4: reg_rd <= H_RES;
            3'd5: reg_rd <= V_RES;
            default: reg_rd <= 16'h0000;
        endcase
        rd_data <= addr_latch[13] ? reg_rd : tram_rd_q;
    end

    reg drive_en;

    always @(posedge I_clk_ppi or negedge I_rst_ppi_n) begin
        if (!I_rst_ppi_n)
            drive_en <= 1'b0;
        else
            drive_en <= cs_active && oe_active && !we_active;
    end

    assign IO_sram_data = drive_en ? rd_data : 16'hzzzz;

    // 总线信号活动标志，复位清零
    // [0]CS [1]ADV [2]WE [3]OE [4]寄存器区访问 [5]读驱动
    reg [5:0] dbg_stat;

    always @(posedge I_clk_ppi or negedge I_rst_ppi_n) begin
        if (!I_rst_ppi_n)
            dbg_stat <= 6'd0;
        else begin
            if (cs_active)                   dbg_stat[0] <= 1'b1;
            if (adv_active)                  dbg_stat[1] <= 1'b1;
            if (we_active)                   dbg_stat[2] <= 1'b1;
            if (oe_active)                   dbg_stat[3] <= 1'b1;
            if (cs_active && addr_latch[13]) dbg_stat[4] <= 1'b1;
            if (drive_en)                    dbg_stat[5] <= 1'b1;
        end
    end

    assign O_dbg_stat = dbg_stat;

    // ========================================================================
    // 异步像素 FIFO：16 位 x 512，格雷码指针
    // ========================================================================
    reg  [15:0] fifo_mem [0:511];

    // 写侧（108MHz）
    reg  [9:0] wptr_bin;
    reg  [9:0] wptr_gray;
    reg  [9:0] rptr_gray_s1, rptr_gray_s2;

    wire [9:0] wptr_bin_next  = wptr_bin + fifo_wr_en;
    wire [9:0] wptr_gray_next = (wptr_bin_next >> 1) ^ wptr_bin_next;

    always @(posedge I_clk_ppi or negedge I_rst_ppi_n) begin
        if (!I_rst_ppi_n) begin
            wptr_bin     <= 10'd0;
            wptr_gray    <= 10'd0;
            rptr_gray_s1 <= 10'd0;
            rptr_gray_s2 <= 10'd0;
        end
        else begin
            wptr_bin     <= wptr_bin_next;
            wptr_gray    <= wptr_gray_next;
            rptr_gray_s1 <= rptr_gray;
            rptr_gray_s2 <= rptr_gray_s1;
        end
    end

    always @(posedge I_clk_ppi) begin
        if (fifo_wr_en)
            fifo_mem[wptr_bin[8:0]] <= wr_data_hold;
    end

    // SOF 时的写指针，读侧清空 FIFO 至此位置
    reg [9:0] wptr_gray_snap;

    always @(posedge I_clk_ppi or negedge I_rst_ppi_n) begin
        if (!I_rst_ppi_n)
            wptr_gray_snap <= 10'd0;
        else if (wr_sof)
            wptr_gray_snap <= wptr_gray;
    end

    assign fifo_wr_full = (wptr_gray == {~rptr_gray_s2[9:8], rptr_gray_s2[7:0]});

    // 读侧（27MHz）
    reg  [9:0] rptr_bin;
    reg  [9:0] rptr_gray;
    reg  [9:0] wptr_gray_s1, wptr_gray_s2;

    wire       fifo_rd_empty = (rptr_gray == wptr_gray_s2);
    wire       fifo_rd_en;
    wire [9:0] rptr_bin_next  = rptr_bin + fifo_rd_en;
    wire [9:0] rptr_gray_next = (rptr_bin_next >> 1) ^ rptr_bin_next;

    always @(posedge I_vin_clk or negedge I_vin_rst_n) begin
        if (!I_vin_rst_n) begin
            rptr_bin     <= 10'd0;
            rptr_gray    <= 10'd0;
            wptr_gray_s1 <= 10'd0;
            wptr_gray_s2 <= 10'd0;
        end
        else begin
            rptr_bin     <= rptr_bin_next;
            rptr_gray    <= rptr_gray_next;
            wptr_gray_s1 <= wptr_gray;
            wptr_gray_s2 <= wptr_gray_s1;
        end
    end

    reg [15:0] fifo_rd_data;

    always @(posedge I_vin_clk) begin
        fifo_rd_data <= fifo_mem[rptr_bin[8:0]];
    end

    // ========================================================================
    // 组帧（27MHz）
    // ========================================================================
    localparam VS_CYCLES  = 8'd64;               // vs_n 脉冲宽度
    localparam GAP_CYCLES = 8'd64;               // vs_n 到首像素的间隔

    localparam S_IDLE   = 3'd0;                  // 等待首个 SOF
    localparam S_VS     = 3'd1;                  // 输出 vs_n
    localparam S_GAP    = 3'd2;
    localparam S_STREAM = 3'd3;                  // 输出像素
    localparam S_PAD    = 3'd4;                  // 补零
    localparam S_DONE   = 3'd5;                  // 等待下一个 SOF
    localparam S_FLUSH  = 3'd6;                  // 清空 FIFO 中的旧数据

    reg [2:0] sof_sync;

    always @(posedge I_vin_clk or negedge I_vin_rst_n) begin
        if (!I_vin_rst_n)
            sof_sync <= 3'b000;
        else
            sof_sync <= {sof_sync[1:0], sof_toggle};
    end

    wire sof_evt = sof_sync[2] ^ sof_sync[1];

    reg [2:0]  state;
    reg [7:0]  seq_cnt;
    reg [19:0] pix_sent;
    reg        have_word;                        // fifo_rd_data 有效
    reg        sof_pending;                      // 补零期间收到 SOF
    reg [9:0]  flush_target;

    wire emit_fifo = (state == S_STREAM) && have_word && !I_vfb_fifo_full;
    wire frame_completing = emit_fifo && (pix_sent == FRAME_PIXELS - 1);
    wire emit_pad  = (state == S_PAD) && !I_vfb_fifo_full
                     && (pix_sent < FRAME_PIXELS);

    assign fifo_rd_en = (state == S_FLUSH)
                        ? ((rptr_gray != flush_target) && !fifo_rd_empty)
                        : ((state == S_STREAM) && !fifo_rd_empty
                           && (!have_word || emit_fifo));

    always @(posedge I_vin_clk or negedge I_vin_rst_n) begin
        if (!I_vin_rst_n) begin
            state        <= S_IDLE;
            seq_cnt      <= 8'd0;
            pix_sent     <= 20'd0;
            have_word    <= 1'b0;
            sof_pending  <= 1'b0;
            flush_target <= 10'd0;
            O_vs_n      <= 1'b1;
            O_de        <= 1'b0;
            O_data      <= 16'd0;
            O_active    <= 1'b0;
            O_frame_cnt <= 8'd0;
        end
        else begin
            O_de <= 1'b0;

            if (sof_evt) begin
                O_active     <= 1'b1;
                flush_target <= wptr_gray_snap;
            end

            if (state == S_FLUSH)
                have_word <= 1'b0;
            else if (fifo_rd_en)
                have_word <= 1'b1;
            else if (emit_fifo)
                have_word <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (sof_evt)
                        state <= S_FLUSH;
                end

                S_VS: begin
                    O_vs_n  <= 1'b0;
                    seq_cnt <= seq_cnt + 1'b1;
                    if (sof_evt) begin
                        O_vs_n <= 1'b1;
                        state  <= S_FLUSH;
                    end
                    else if (seq_cnt == VS_CYCLES - 1) begin
                        O_vs_n  <= 1'b1;
                        seq_cnt <= 8'd0;
                        state   <= S_GAP;
                    end
                end

                S_GAP: begin
                    seq_cnt <= seq_cnt + 1'b1;
                    if (sof_evt)
                        state <= S_FLUSH;
                    else if (seq_cnt == GAP_CYCLES - 1) begin
                        pix_sent <= 20'd0;
                        state    <= S_STREAM;
                    end
                end

                S_STREAM: begin
                    if (emit_fifo) begin
                        O_de     <= 1'b1;
                        O_data   <= fifo_rd_data;
                        pix_sent <= pix_sent + 1'b1;

                        if (frame_completing) begin
                            O_frame_cnt <= O_frame_cnt + 1'b1;
                            state       <= S_DONE;
                        end
                    end

                    // 帧未完成时收到 SOF，先补零
                    if (sof_evt) begin
                        if (frame_completing || (pix_sent == 20'd0 && !emit_fifo))
                            state <= S_FLUSH;
                        else begin
                            sof_pending <= 1'b1;
                            state       <= S_PAD;
                        end
                    end
                end

                S_PAD: begin
                    if (emit_pad) begin
                        O_de     <= 1'b1;
                        O_data   <= 16'd0;
                        pix_sent <= pix_sent + 1'b1;

                        if (pix_sent == FRAME_PIXELS - 1) begin
                            O_frame_cnt <= O_frame_cnt + 1'b1;
                            sof_pending <= 1'b0;
                            state       <= sof_pending ? S_FLUSH : S_DONE;
                        end
                    end
                end

                S_DONE: begin
                    if (sof_evt)
                        state <= S_FLUSH;
                end

                S_FLUSH: begin
                    if (rptr_gray == flush_target) begin
                        seq_cnt <= 8'd0;
                        state   <= S_VS;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
