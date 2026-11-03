//============================================================================
// edge_detect.v  --  Sobel 3x3 边缘检测模块（96bit / 4像素每拍 / 720p60）
//
// 功能概述：
//   对灰度图做 3x3 Sobel 二维梯度，输出二值边缘图（边缘=白 0xFFFFFF，背景=黑）。
//   输入 I_tdata 为 4 像素 x RGB888（与 img_proc 一致，px0=[23:0]..px3=[95:72]）。
//
// 算法：
//   灰度  Y = (R + 2G + B) >> 2
//   Gx = (p02 + 2*p12 + p22) - (p00 + 2*p10 + p20)
//   Gy = (p20 + 2*p21 + p22) - (p00 + 2*p01 + p02)
//   mag = |Gx| + |Gy|（0..2040）
//   边缘 = mag > 自适应门限 ? 白 : 黑
//     自适应门限 = 整帧平均 mag * ADAPT_K/8（帧间闭环，首帧回退 EDGE_THRESH）
//
// 数据路径（满吞吐，总延迟 L_TOTAL = 644 拍）：
//   Y(组合) -> 行缓冲0(321拍) -> 行缓冲1(321拍) -> 对齐寄存器(l2_d1,1拍) -> Sobel(组合) -> 输出寄存器(1拍)
//
//   行缓冲 line_buffer_demosaic 每次延迟 1 行 = 321 拍（320 组数据 + 1 拍
//   tlast_lock 门控），且输出相对输入滞后 1 组（4 像素）列。因此三行对齐需
//   cur 补 2 拍、l1 补 1 拍、l2 补 0 拍（同 raw_matrix_3x3_buffer 手法），
//   再统一多 1 拍凑出水平 3 邻域（左邻靠 carry 寄存器、右邻靠下一组 px0）。
//
// 控制信号对齐：
//   O_tvalid = valid_2（第 2 级行缓冲输出有效）再延迟 2 拍，与 O_tdata 同拍；
//   O_tuser/O_tlast 用移位寄存器延迟 L_TOTAL-1 拍、输出寄存器再补 1 拍，
//   合计 L_TOTAL=644 拍。因上游 awb 的 tuser 本就领先数据 54 拍，延迟后该
//   54 拍领先关系保持不变，满足下游 video_in 帧首 FIFO 复位时序（见 awb.v 头注释）。
//
// 边界处理：
//   首末行/首末列窗口不完整，强制输出黑（v_cnt==0/719、h_cnt==0/319 对应
//   px0/px3），避免 0 填充与行间 carry 越界造成的白色边框伪边缘。
//============================================================================

module edge_detect #(
    parameter IMG_WIDTH    = 1280,        // 图像宽度（像素）
    parameter IMG_HEIGHT   = 720,         // 图像高度（行）
    parameter [7:0] EDGE_THRESH = 8'd128, // 首帧回退门限（尚无帧均值时）
    parameter [7:0] ADAPT_K     = 8'd16   // 自适应阈值系数：门限 = 帧均值 * K / 8（16 -> 2.0x）
)(
    input  wire        I_clk,
    input  wire        I_rst_n,      // 异步复位，低有效
    input  wire [95:0] I_tdata,      // 4 像素 x RGB888
    input  wire        I_tvalid,
    input  wire        I_tuser,      // 帧开始脉冲（SOF）
    input  wire        I_tlast,      // 行结束（EOL）
    output reg  [95:0] O_tdata,
    output reg         O_tvalid,
    output reg         O_tuser,
    output reg         O_tlast
);

    localparam IMG_WIDTH_4x = IMG_WIDTH >> 2;   // 每行 4 像素组数 = 320

    // 总延迟 L_TOTAL = 2 行缓冲(2*321) + 对齐 l2_d1(1) + 输出寄存器(1) = 644
    // 控制信号移位寄存器长度 = L_TOTAL - 1 = 643（输出寄存器再补 1 拍）
    localparam L_SR = 643;

    //------------------------------------------------------------------------
    // F_gray：灰度值（移位近似，无 DSP）
    //------------------------------------------------------------------------
    function [7:0] F_gray;
        input [7:0] r;
        input [7:0] g;
        input [7:0] b;
        begin
            F_gray = ({2'b00, r} + {1'b0, g, 1'b0} + {2'b00, b}) >> 2;
        end
    endfunction

    //------------------------------------------------------------------------
    // 灰度化：4 像素并行 Y（32bit），px3 在 [31:24]，px0 在 [7:0]
    //------------------------------------------------------------------------
    wire [31:0] Y = { F_gray(I_tdata[95:88], I_tdata[87:80], I_tdata[79:72]),  // px3
                      F_gray(I_tdata[71:64], I_tdata[63:56], I_tdata[55:48]),  // px2
                      F_gray(I_tdata[47:40], I_tdata[39:32], I_tdata[31:24]),  // px1
                      F_gray(I_tdata[23:16], I_tdata[15:8],  I_tdata[7:0]) };  // px0

    //------------------------------------------------------------------------
    // 2 个行缓冲：各延迟 1 行。数据 33bit 复用 line_buffer_demosaic（高 1 位恒 0）。
    // 两级都复位于 I_tuser（帧首）。因 awb 的 tuser 领先数据 54 拍，addra 只在
    // valid 时递增，故领先不影响地址对齐。
    //------------------------------------------------------------------------
    wire [32:0] lb0_in = {1'b0, Y};
    wire [32:0] lb0_out;
    wire [32:0] lb1_out;
    wire        lb0_valid;
    wire        lb1_valid;

    line_buffer_demosaic #(
        .IMG_HEIGHT (IMG_HEIGHT),
        .IMG_WIDTH  (IMG_WIDTH )
    ) u_lb0 (
        .I_clk   (I_clk    ),
        .I_rst_n (I_rst_n  ),
        .I_tuser (I_tuser  ),
        .I_valid (I_tvalid ),
        .I_data  (lb0_in   ),
        .O_valid (lb0_valid),
        .O_data  (lb0_out  )
    );

    line_buffer_demosaic #(
        .IMG_HEIGHT (IMG_HEIGHT),
        .IMG_WIDTH  (IMG_WIDTH )
    ) u_lb1 (
        .I_clk   (I_clk    ),
        .I_rst_n (I_rst_n  ),
        .I_tuser (I_tuser  ),
        .I_valid (lb0_valid),
        .I_data  (lb0_out  ),
        .O_valid (lb1_valid),
        .O_data  (lb1_out  )
    );

    wire [31:0] Y_l1 = lb0_out[31:0];   // 上一行
    wire [31:0] Y_l2 = lb1_out[31:0];   // 上两行

    //------------------------------------------------------------------------
    // 对齐 + 水平窗口移位寄存器
    //   cur(当前行)：4 级延迟（cur_d3 对齐列 c-3，cur_d4=c-4 左邻，cur_d2=c-2 右邻）
    //   l1(上一行) ：3 级延迟（l1_d2 对齐，l1_d3 左邻，l1_d1 右邻）
    //   l2(上两行) ：2 级延迟（l2_d1 对齐，l2_d2 左邻，Y_l2 右邻）
    //------------------------------------------------------------------------
    reg [31:0] cur_d1, cur_d2, cur_d3, cur_d4;
    reg [31:0] l1_d1,  l1_d2,  l1_d3;
    reg [31:0] l2_d1,  l2_d2;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            cur_d1 <= 32'd0; cur_d2 <= 32'd0; cur_d3 <= 32'd0; cur_d4 <= 32'd0;
            l1_d1  <= 32'd0; l1_d2  <= 32'd0; l1_d3  <= 32'd0;
            l2_d1  <= 32'd0; l2_d2  <= 32'd0;
        end else begin
            cur_d1 <= Y;      cur_d2 <= cur_d1; cur_d3 <= cur_d2; cur_d4 <= cur_d3;
            l1_d1  <= Y_l1;   l1_d2  <= l1_d1;  l1_d3  <= l1_d2;
            l2_d1  <= Y_l2;   l2_d2  <= l2_d1;
        end
    end

    //------------------------------------------------------------------------
    // F_sobel_mag：3x3 窗口 -> 梯度幅度 |Gx|+|Gy|（0..2040，11bit）
    //------------------------------------------------------------------------
    function [10:0] F_sobel_mag;
        input [7:0] p00, p01, p02;   // 顶行
        input [7:0] p10, p11, p12;   // 中行
        input [7:0] p20, p21, p22;   // 底行
        integer gx_pos, gx_neg, gy_pos, gy_neg;
        integer gx, gy, mag;
        begin
            // 各分量 0..1020，用 integer 承接，负梯度以有符号减法得到
            gx_pos = p02 + 2*p12 + p22;
            gx_neg = p00 + 2*p10 + p20;
            gy_pos = p20 + 2*p21 + p22;
            gy_neg = p00 + 2*p01 + p02;
            gx = gx_pos - gx_neg;   // -1020..1020
            gy = gy_pos - gy_neg;
            if (gx < 0) gx = -gx;
            if (gy < 0) gy = -gy;
            mag = gx + gy;          // 0..2040
            F_sobel_mag = mag[10:0];
        end
    endfunction

    //------------------------------------------------------------------------
    // Sobel 3x3 计算（对齐列 c-3 的 4 个像素）——先求梯度幅度 mag，再与
    // 自适应门限 S_thresh 比较得到二值边缘 e0~e3。
    //------------------------------------------------------------------------
    wire [10:0] mag0 = F_sobel_mag(
        l2_d2[31:24], l2_d1[7:0],  l2_d1[15:8],
        l1_d3[31:24], l1_d2[7:0],  l1_d2[15:8],
        cur_d4[31:24], cur_d3[7:0], cur_d3[15:8]
    );
    wire [10:0] mag1 = F_sobel_mag(
        l2_d1[7:0],  l2_d1[15:8],  l2_d1[23:16],
        l1_d2[7:0],  l1_d2[15:8],  l1_d2[23:16],
        cur_d3[7:0], cur_d3[15:8], cur_d3[23:16]
    );
    wire [10:0] mag2 = F_sobel_mag(
        l2_d1[15:8],  l2_d1[23:16], l2_d1[31:24],
        l1_d2[15:8],  l1_d2[23:16], l1_d2[31:24],
        cur_d3[15:8], cur_d3[23:16], cur_d3[31:24]
    );
    wire [10:0] mag3 = F_sobel_mag(
        l2_d1[23:16], l2_d1[31:24], Y_l2[7:0],
        l1_d2[23:16], l1_d2[31:24], l1_d1[7:0],
        cur_d3[23:16], cur_d3[31:24], cur_d2[7:0]
    );

    //------------------------------------------------------------------------
    // 自适应阈值：以整帧平均梯度幅度为基准，mag > mean_mag * ADAPT_K/8 判边缘。
    // 均值推导：一帧 1280x720 = 921600 像素 = 230400 拍 x 4 像素。
    //   S_mag_acc = 整帧 sum(mag) <= 230400 * 8160 = 1.88e9 < 2^31
    //   mean = S_mag_acc / 921600 = (S_mag_acc >> 20) * 1.1378
    //   -> 用 m1 + (m1 >> 3) = 1.125 * m1 近似，误差 < 1.2%（由 ADAPT_K 补偿）
    // 资源：1 个 32bit 累加器 + 1 个 11bit 均值寄存器，无 DSP。
    // 首帧无历史均值时回退 EDGE_THRESH/2（K=16 时首帧门限恰为 EDGE_THRESH）。
    //------------------------------------------------------------------------
    wire [12:0] S_mag_sum4 = {2'b00, mag0} + {2'b00, mag1} +
                              {2'b00, mag2} + {2'b00, mag3};

    reg  [31:0] S_mag_acc;
    reg  [10:0] S_mag_pivot;

    wire [10:0] S_mag_m1      = S_mag_acc[30:20];                        // acc >> 20（11bit）
    wire [11:0] S_mag_mean12  = {1'b0, S_mag_m1} + {3'b000, S_mag_m1[10:3]};
    wire [10:0] S_mag_mean    = S_mag_mean12[11] ? 11'h7FF : S_mag_mean12[10:0];
    wire [10:0] S_mag_mean_eff = (S_mag_acc == 32'd0) ? {2'b00, EDGE_THRESH[7:1]}
                                                      : S_mag_mean;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_mag_acc   <= 32'd0;
            S_mag_pivot <= {2'b00, EDGE_THRESH[7:1]};
        end else if (tuser_sr[L_SR-1]) begin
            S_mag_pivot <= S_mag_mean_eff;   // 帧首锁存上一帧均值
            S_mag_acc   <= 32'd0;            // 从新帧开始累加
        end else if (valid_2_d1) begin
            S_mag_acc <= S_mag_acc + {19'd0, S_mag_sum4};
        end
    end

    // 门限 = 均值 * K / 8，饱和到 11bit（>=2047 时等价于永不判边缘）
    wire [18:0] S_thresh_full = {8'b0, S_mag_pivot} * ADAPT_K;
    wire [10:0] S_thresh = (S_thresh_full[18:14] != 5'b0) ? 11'h7FF : S_thresh_full[13:3];

    wire e0 = (mag0 > S_thresh);
    wire e1 = (mag1 > S_thresh);
    wire e2 = (mag2 > S_thresh);
    wire e3 = (mag3 > S_thresh);

    //------------------------------------------------------------------------
    // 输出有效：valid_2（第 2 级行缓冲输出有效）延迟 1 拍，对齐 Sobel 组合结果
    //------------------------------------------------------------------------
    reg valid_2_d1;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            valid_2_d1 <= 1'b0;
        else
            valid_2_d1 <= lb1_valid;
    end

    //------------------------------------------------------------------------
    // 输出行列计数（驱动 valid_2_d1，用于边界抑制）
    //------------------------------------------------------------------------
    reg [13:0] h_cnt;
    reg [13:0] v_cnt;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n || I_tuser) begin
            h_cnt <= 14'd0;
            v_cnt <= 14'd0;
        end else begin
            h_cnt <= valid_2_d1 ? ((h_cnt == IMG_WIDTH_4x - 1) ? 14'd0 : h_cnt + 14'd1) : h_cnt;
            v_cnt <= (valid_2_d1 && h_cnt == IMG_WIDTH_4x - 1)
                     ? ((v_cnt == IMG_HEIGHT - 1) ? 14'd0 : v_cnt + 14'd1)
                     : v_cnt;
        end
    end

    //------------------------------------------------------------------------
    // 边界抑制：首末行 / 首末列（h_cnt==0 的 px0、h_cnt==319 的 px3）输出黑
    //------------------------------------------------------------------------
    wire border_row = (v_cnt == 0) || (v_cnt == IMG_HEIGHT - 1);
    wire e0_f = e0 & ~border_row & ~(h_cnt == 0);
    wire e1_f = e1 & ~border_row;
    wire e2_f = e2 & ~border_row;
    wire e3_f = e3 & ~border_row & ~(h_cnt == IMG_WIDTH_4x - 1);

    wire [95:0] sobel_data = { e3_f ? 24'hFFFFFF : 24'h000000,
                               e2_f ? 24'hFFFFFF : 24'h000000,
                               e1_f ? 24'hFFFFFF : 24'h000000,
                               e0_f ? 24'hFFFFFF : 24'h000000 };

    //------------------------------------------------------------------------
    // 控制信号移位寄存器：tuser/tlast 延迟 L_SR=643 拍（输出寄存器再补 1 拍 = 644）
    //------------------------------------------------------------------------
    reg [L_SR-1:0] tuser_sr;
    reg [L_SR-1:0] tlast_sr;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            tuser_sr <= {L_SR{1'b0}};
            tlast_sr <= {L_SR{1'b0}};
        end else begin
            tuser_sr <= {tuser_sr[L_SR-2:0], I_tuser};
            tlast_sr <= {tlast_sr[L_SR-2:0], I_tlast};
        end
    end

    //------------------------------------------------------------------------
    // 输出寄存器
    //------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tdata  <= 96'd0;
            O_tvalid <= 1'b0;
            O_tuser  <= 1'b0;
            O_tlast  <= 1'b0;
        end else begin
            O_tdata  <= sobel_data;
            O_tvalid <= valid_2_d1;
            O_tuser  <= tuser_sr[L_SR-1];
            O_tlast  <= tlast_sr[L_SR-1];
        end
    end

endmodule
