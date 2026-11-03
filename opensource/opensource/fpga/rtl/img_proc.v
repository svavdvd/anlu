//============================================================================
// img_proc.v  --  图像处理算法模块（96bit / 4像素每拍 / 720p60）
//
// 功能概述：
//   7 档图像处理模式，在帧边界（tuser）切换，3 级满吞吐流水线。
//
// 档位定义（I_mode）：
//   3'd0 : 直通（Bypass）       -- 输出 = 输入（半量色调曲线 + 亮度/对比度）
//   3'd1 : 灰度化（Grayscale）  -- Y = (R + 2G + B) >> 2（移位近似，无 DSP）
//   3'd2 : 二值化（Binary）     -- 滑动窗口局部均值门限（自适应阈值，见 binarize_sw）
//   3'd3 : 反色（Invert）       -- 每通道输出 255 - x（按位取反）
//   3'd4 : 伽马校正（Gamma）    -- gamma 2.2，8 段折线近似
//   3'd5 : 边缘检测（Sobel）    -- 3x3 Sobel 梯度，输出二值边缘图
//   3'd6 : 卡通化（Cartoon）    -- 在直通档色调曲线上 1.25x 提饱和 + 每通道高 4bit
//                                  量化（4096 色）+ 3x3 Sobel 黑描边
//   3'd7 : 直通 + 颜色标注      -- 直通曲线叠加红/蓝物体外接矩形框（HSV 阈值法,
//                                 见 red/blue_object_detect.v，检测结果经本模块输出）
//
// 数据格式：
//   96bit = 4 像素 x RGB888
//   I_tdata[23:0]   = 像素 0（最先输出），{R,G,B} = [23:16],[15:8],[7:0]
//   I_tdata[47:24]  = 像素 1
//   I_tdata[71:48]  = 像素 2
//   I_tdata[95:72]  = 像素 3
//
// 流水线结构（3 级满吞吐）：
//   Stage 1 (S1)      : 寄存输入数据与控制信号，采样当前生效档位，
//                       组合计算灰度/伽马/亮度对比度/二值化结果
//   Stage 2 (S2)      : 寄存各档位结果与原始数据/控制
//   Stage 3 (O)       : 按档位选择最终输出并寄存
//
// 关键约束：
//   tuser/tlast 与 tdata/tvalid 保持等拍相对延迟（此处统一延迟 3 拍），
//   保证下游 video_in 帧起始定位正确，不会画面错位或滚动。
//============================================================================

module img_proc (
    input  wire        I_clk,
    input  wire        I_rst_n,      // 异步复位，低有效
    input  wire [2:0]  I_mode,       // 档位请求（由顶层提供，可能随时变化）
    input  wire [2:0]  I_brightness, // 亮度档位 0~7（仅直通档生效，1x~4x）
    input  wire [2:0]  I_contrast,   // 对比度档位 0~7（仅直通档生效，0.5x~2.5x）
    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    input  wire        I_tuser,      // 帧开始脉冲（SOF）
    input  wire        I_tlast,      // 行结束（EOL）
    output reg  [95:0] O_tdata,
    output reg         O_tvalid,
    output reg         O_tuser,
    output reg         O_tlast,

    // 三色物体检测的框边界命中掩码 / 判色掩码，供 mode 7 叠加绘制。
    // 检测本身已移到 isp_top：红/蓝跑在 AWB 之后（原始数据路径），绿单独
    // 跑在 AWB 之前；送到这里的信号均已与 I_tdata 对齐（绿由 isp_top 延迟
    // AWB 拍数后送入，红/蓝天然同拍），内部再延迟 2 拍与 S2_curve 对齐。
    // 检测结果(bbox/质心)不再经本模块。
    input  wire [3:0]  I_red_box_hit,
    input  wire [3:0]  I_red_mask,
    input  wire [3:0]  I_blue_box_hit,
    input  wire [3:0]  I_blue_mask,
    input  wire [3:0]  I_green_box_hit,
    input  wire [3:0]  I_green_mask,

    // 三色物理坐标标签(isp_top 的 coord_one 产生, 每帧恒定)。
    // 打包格式 [54:0] = {lab_valid, xneg, lab_x[10:0], lab_y[9:0], xdig[15:0], ydig[15:0]}
    input  wire [54:0] I_red_lab,
    input  wire [54:0] I_blue_lab,
    input  wire [54:0] I_green_lab
);

    //------------------------------------------------------------------------
    // 帧边界生效档位（shadow 寄存器，防撕裂）
    // S_mode_act 仅在 tuser 有效时从 I_mode 载入，帧内保持不变
    //------------------------------------------------------------------------
    reg [2:0] S_mode_act;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            S_mode_act <= 3'd0;
        else if (I_tuser)
            S_mode_act <= I_mode;
    end

    //------------------------------------------------------------------------
    // 亮度/对比度档位：帧边界生效（防帧内撕裂），复位默认 亮度1x/对比度1.5x
    //------------------------------------------------------------------------
    reg [2:0] S_bright_act;
    reg [2:0] S_contrast_act;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_bright_act   <= 3'd2;     // 默认亮度 1.25x（档位表第 2 档）
            S_contrast_act <= 3'd1;     // 默认对比度 1.00x（档位表第 1 档）
        end else if (I_tuser) begin
            S_bright_act   <= I_brightness;
            S_contrast_act <= I_contrast;
        end
    end

    //------------------------------------------------------------------------
    // 三色物体检测已移到 isp_top，跑在 AWB 之前（pre-AWB 的 demosaic 输出），
    // 修复“红蓝准、绿色不准”的问题——AWB 固定增益 R×1.25/B×1.34/G×1.0 会压低
    // 绿色的有效饱和度。这里只接收检测的框/掩码（已对齐 I_tdata），延迟 2 拍
    // 与 S2_curve 对齐后用于 mode 7 叠加绘制。
    //------------------------------------------------------------------------
    // 框边界命中掩码延迟 2 拍, 与 S2_curve 对齐
    reg [3:0] S1_box_hit;
    reg [3:0] S2_box_hit;
    reg [3:0] S1_red_mask;
    reg [3:0] S2_red_mask;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S1_box_hit  <= 4'd0;
            S2_box_hit  <= 4'd0;
            S1_red_mask <= 4'd0;
            S2_red_mask <= 4'd0;
        end else begin
            S1_box_hit  <= I_red_box_hit;
            S2_box_hit  <= S1_box_hit;
            S1_red_mask <= I_red_mask;
            S2_red_mask <= S1_red_mask;
        end
    end

    // 蓝色框边界命中掩码延迟 2 拍, 与 S2_curve 对齐(检测在 isp_top, 已对齐 I_tdata)
    reg [3:0] S1_blue_hit;
    reg [3:0] S2_blue_hit;
    reg [3:0] S1_blue_mask;
    reg [3:0] S2_blue_mask;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S1_blue_hit  <= 4'd0;
            S2_blue_hit  <= 4'd0;
            S1_blue_mask <= 4'd0;
            S2_blue_mask <= 4'd0;
        end else begin
            S1_blue_hit  <= I_blue_box_hit;
            S2_blue_hit  <= S1_blue_hit;
            S1_blue_mask <= I_blue_mask;
            S2_blue_mask <= S1_blue_mask;
        end
    end

    // 绿色框边界命中掩码延迟 2 拍, 与 S2_curve 对齐(检测在 isp_top, 已对齐 I_tdata)
    reg [3:0] S1_green_hit;
    reg [3:0] S2_green_hit;
    reg [3:0] S1_green_mask;
    reg [3:0] S2_green_mask;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S1_green_hit  <= 4'd0;
            S2_green_hit  <= 4'd0;
            S1_green_mask <= 4'd0;
            S2_green_mask <= 4'd0;
        end else begin
            S1_green_hit  <= I_green_box_hit;
            S2_green_hit  <= S1_green_hit;
            S1_green_mask <= I_green_mask;
            S2_green_mask <= S1_green_mask;
        end
    end

    //------------------------------------------------------------------------
    // 像素坐标计数(与 I_tdata 同拍): h_cnt 0..319(4 像素一组), v_cnt 0..719
    //   计数方式与 object_detect_3color 内部完全一致, 保证与检测器同源;
    //   再各延迟 2 拍, 得到 S2 级(S2_curve)当前像素的坐标, 供坐标文字叠加使用。
    //------------------------------------------------------------------------
    localparam integer IMG_W4 = 1280 / 4;   // 320

    reg [8:0]  S_h_cnt;
    reg [9:0]  S_v_cnt;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n || I_tuser) begin
            S_h_cnt <= 9'd0;
            S_v_cnt <= 10'd0;
        end else if (I_tvalid) begin
            S_h_cnt <= (S_h_cnt == (IMG_W4 - 1)) ? 9'd0 : S_h_cnt + 9'd1;
            S_v_cnt <= (S_h_cnt == (IMG_W4 - 1))
                     ? ((S_v_cnt == 10'd719) ? 10'd0 : S_v_cnt + 10'd1)
                     : S_v_cnt;
        end
    end

    reg [10:0] S1_px, S2_px;
    reg [9:0]  S1_py, S2_py;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S1_px <= 11'd0; S2_px <= 11'd0;
            S1_py <= 10'd0; S2_py <= 10'd0;
        end else begin
            S1_px <= {S_h_cnt, 2'b00};      // 组号 -> 组内首像素的 x
            S2_px <= S1_px;
            S1_py <= S_v_cnt;
            S2_py <= S1_py;
        end
    end

    //------------------------------------------------------------------------
    // 第 1 级流水线：寄存输入与控制
    //------------------------------------------------------------------------
    reg [95:0] S1_data;
    reg        S1_valid;
    reg        S1_user;
    reg        S1_last;
    reg [2:0]  S1_mode;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S1_data  <= 96'd0;
            S1_valid <= 1'b0;
            S1_user  <= 1'b0;
            S1_last  <= 1'b0;
            S1_mode  <= 3'd0;
        end else begin
            S1_data  <= I_tdata;
            S1_valid <= I_tvalid;
            S1_user  <= I_tuser;
            S1_last  <= I_tlast;
            S1_mode  <= S_mode_act;
        end
    end

    //------------------------------------------------------------------------
    // 第 1 级组合计算：灰度 / 伽马 / 亮度对比度 / 二值化
    //------------------------------------------------------------------------

    // --- 像素 0~3 提取（显式常量下标切片，兼容 TD 综合器）---
    wire [7:0] S1_px0_r = S1_data[23:16];
    wire [7:0] S1_px0_g = S1_data[15:8];
    wire [7:0] S1_px0_b = S1_data[7:0];

    wire [7:0] S1_px1_r = S1_data[47:40];
    wire [7:0] S1_px1_g = S1_data[39:32];
    wire [7:0] S1_px1_b = S1_data[31:24];

    wire [7:0] S1_px2_r = S1_data[71:64];
    wire [7:0] S1_px2_g = S1_data[63:56];
    wire [7:0] S1_px2_b = S1_data[55:48];

    wire [7:0] S1_px3_r = S1_data[95:88];
    wire [7:0] S1_px3_g = S1_data[87:80];
    wire [7:0] S1_px3_b = S1_data[79:72];

    // --- 4 像素灰度值（移位近似：Y = (R + 2G + B) >> 2，无 DSP）---
    wire [7:0] S1_y0 = F_gray(S1_px0_r, S1_px0_g, S1_px0_b);
    wire [7:0] S1_y1 = F_gray(S1_px1_r, S1_px1_g, S1_px1_b);
    wire [7:0] S1_y2 = F_gray(S1_px2_r, S1_px2_g, S1_px2_b);
    wire [7:0] S1_y3 = F_gray(S1_px3_r, S1_px3_g, S1_px3_b);

    // 灰度化：三通道均输出灰度值 Y
    wire [95:0] S1_gray_all = {S1_y3, S1_y3, S1_y3,
                               S1_y2, S1_y2, S1_y2,
                               S1_y1, S1_y1, S1_y1,
                               S1_y0, S1_y0, S1_y0};

    // --- 4 像素伽马校正值（gamma=2.2 折线近似）---
    wire [23:0] S1_gam0 = {F_gamma(S1_px0_r), F_gamma(S1_px0_g), F_gamma(S1_px0_b)};
    wire [23:0] S1_gam1 = {F_gamma(S1_px1_r), F_gamma(S1_px1_g), F_gamma(S1_px1_b)};
    wire [23:0] S1_gam2 = {F_gamma(S1_px2_r), F_gamma(S1_px2_g), F_gamma(S1_px2_b)};
    wire [23:0] S1_gam3 = {F_gamma(S1_px3_r), F_gamma(S1_px3_g), F_gamma(S1_px3_b)};

    wire [95:0] S1_gamma_all = {S1_gam3, S1_gam2, S1_gam1, S1_gam0};

    //------------------------------------------------------------------------
    // 逐帧亮度均值（只给卡通化用）—— 卡通化之所以"糊成一片粉色雾"，是因为
    // 量化把一个很窄的灰度区间压成了 2~3 个色阶。这里用整帧的平均亮度做
    // 自动色阶的支点，让减色前的画面先被拉到接近满量程。
    //
    // 均值推导：一帧 1280x720 = 921600 像素 = 230400 拍 x 4 像素。
    //   S_y_acc = 整帧 sum(Y)  <= 921600 * 255 = 2.35 亿 < 2^28
    //   mean = S_y_acc / 921600 = (S_y_acc >> 20) * 1.1378
    //   -> 用 m1 + (m1 >> 3) = 1.125 * m1 近似，误差 < 1.2%
    // 内存代价：1 个 32bit 累加器 + 1 个 8bit 支点寄存器，无 DSP。
    //------------------------------------------------------------------------
    reg  [31:0] S_y_acc;
    reg  [7:0]  S_y_pivot;

    wire [9:0] S1_y_sum4 = {2'd0, S1_y0} + {2'd0, S1_y1} +
                           {2'd0, S1_y2} + {2'd0, S1_y3};
    wire [7:0] S_y_m1   = S_y_acc[27:20];
    wire [8:0] S_y_mean9 = {1'b0, S_y_m1} + {4'd0, S_y_m1[7:3]};
    wire [7:0] S_y_mean  = S_y_mean9[8] ? 8'd255 : S_y_mean9[7:0];

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_y_acc   <= 32'd0;
            S_y_pivot <= 8'd128;
        end else if (S1_user) begin
            S_y_pivot <= S_y_mean;              // 帧首锁存上一帧的均值
            S_y_acc   <= {22'd0, S1_y_sum4};    // 从当前拍重新累加
        end else if (S1_valid) begin
            S_y_acc <= S_y_acc + {22'd0, S1_y_sum4};
        end
    end

    // --- 亮度/对比度处理的落点已改到「输出级」（见本文件后面的 S3_any_* 段）---
    // 原来只在直通档(3'd0)做，导致灰度/反色/伽马三档按 K2/K3 没反应；
    // 现在把 F_bc 提到「六档选完之后」，对最终要送出去的像素统一处理，
    // 于是六个模式全部受 K2(亮度)/K3(对比度) 控制，且 F_bc 例化数仍是 12 个，
    // 不增加任何 DSP（全是移位加法）。
    // 二值化(3'd2)/边缘(3'd5) 输出只有 0 与 255：对比度 >= 1.0 时 0->0、255->255
    // 不变，对比度 < 1.0 时会向 128 收敛（这是期望行为，等于降低二值锐度）。

    //------------------------------------------------------------------------
    // 第 2 级流水线：寄存各档位结果与原始数据/控制
    //------------------------------------------------------------------------
    reg [95:0] S2_data;         // 原始数据（直通/反色/亮度对比度共用）
    reg [95:0] S2_gray_all;     // 灰度档
    reg [95:0] S2_gamma_all;    // 伽马档
    reg        S2_valid;
    reg        S2_user;
    reg        S2_last;
    reg [2:0]  S2_mode;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S2_data      <= 96'd0;
            S2_gray_all  <= 96'd0;
            S2_gamma_all <= 96'd0;
            S2_valid     <= 1'b0;
            S2_user      <= 1'b0;
            S2_last      <= 1'b0;
            S2_mode      <= 3'd0;
        end else begin
            S2_data      <= S1_data;
            S2_gray_all  <= S1_gray_all;
            S2_gamma_all <= S1_gamma_all;
            S2_valid     <= S1_valid;
            S2_user      <= S1_user;
            S2_last      <= S1_last;
            S2_mode      <= S1_mode;
        end
    end

    //------------------------------------------------------------------------
    // 第 3 级流水线（输出级）：按档位选择最终输出并寄存
    //------------------------------------------------------------------------

    // 档位选择（组合逻辑）——只做「算法选择」，亮度/对比度统一放在后面
    reg [95:0] S3_sel_data;

    //------------------------------------------------------------------------
    // 直通档的色调曲线（第七版新增）——解决「画面过于暗淡」
    //
    // 纯线性放大解决不了发暗：室内场景的有效像素大量落在 40..120，线性 2.5x
    // 先把亮部削顶（灯、白墙直接 255），暗部还是暗，于是整幅又灰又糊。
    // 手机照片好看的本质是「抬暗部、压高光」，所以这里给直通档叠一条曲线：
    //
    //     y = ( x + gamma(x) ) >> 1
    //
    // gamma(x) 就是模式 4 本来就在算的那条 8 段折线（F_gamma，gamma 2.2 解码），
    // S2_gamma_all 每像素都在算，原来只有模式 4 用，这里直接复用 —— 不新增乘法、
    // 不新增 DSP，只多 12 个 9bit 加法器和 12 个右移。
    // 取 50% 混合而不是全量 gamma，是为了保留对比度：全量 gamma 会发灰，
    // 半量既能明显抬暗部（x=60 → 约 96），又不会把画面压平。
    //
    // 注意加法必须先扩到 9bit，否则 a+b 在 8bit 里溢出（200+200=144）。
    //------------------------------------------------------------------------
    function [7:0] F_mix_half;
        input [7:0] a;
        input [7:0] b;
        begin F_mix_half = ({1'b0, a} + {1'b0, b}) >> 1; end
    endfunction

    wire [95:0] S2_curve = {
        F_mix_half(S2_data[95:88], S2_gamma_all[95:88]),   // px3 R
        F_mix_half(S2_data[87:80], S2_gamma_all[87:80]),   // px3 G
        F_mix_half(S2_data[79:72], S2_gamma_all[79:72]),   // px3 B
        F_mix_half(S2_data[71:64], S2_gamma_all[71:64]),   // px2 R
        F_mix_half(S2_data[63:56], S2_gamma_all[63:56]),   // px2 G
        F_mix_half(S2_data[55:48], S2_gamma_all[55:48]),   // px2 B
        F_mix_half(S2_data[47:40], S2_gamma_all[47:40]),   // px1 R
        F_mix_half(S2_data[39:32], S2_gamma_all[39:32]),   // px1 G
        F_mix_half(S2_data[31:24], S2_gamma_all[31:24]),   // px1 B
        F_mix_half(S2_data[23:16], S2_gamma_all[23:16]),   // px0 R
        F_mix_half(S2_data[15:8],  S2_gamma_all[15:8] ),   // px0 G
        F_mix_half(S2_data[7:0],   S2_gamma_all[7:0]  )    // px0 B
    };

    //------------------------------------------------------------------------
    // 识别档(mode 7) 物理坐标文字：三色各一套点阵发生器
    //   标签内容在 isp_top 的 coord_one 里每帧算一次(坐标换算 + 十进制拆分),
    //   这里只做像素级查表, 每拍查一次(一拍 4 像素恰好落在同一个字模点上)。
    //   仅 S2_mode==7 时输出, 其它档位完全不受影响。
    //------------------------------------------------------------------------
    localparam [23:0] LAB_TEXT_COLOR = 24'hFFFFFF;   // 坐标文字：白色

    wire [3:0] S2_lab_r;
    wire [3:0] S2_lab_b;
    wire [3:0] S2_lab_g;
    wire       S2_lab_en = (S2_mode == 3'd7);

    coord_label u_lab_red (
        .I_en        (S2_lab_en        ),
        .I_lab_valid (I_red_lab[54]    ),
        .I_lab_x     (I_red_lab[52:42] ),
        .I_lab_y     (I_red_lab[41:32] ),
        .I_xneg      (I_red_lab[53]    ),
        .I_xdig      (I_red_lab[31:16] ),
        .I_ydig      (I_red_lab[15:0]  ),
        .I_px        (S2_px            ),
        .I_py        (S2_py            ),
        .O_hit       (S2_lab_r         )
    );

    coord_label u_lab_blue (
        .I_en        (S2_lab_en        ),
        .I_lab_valid (I_blue_lab[54]   ),
        .I_lab_x     (I_blue_lab[52:42]),
        .I_lab_y     (I_blue_lab[41:32]),
        .I_xneg      (I_blue_lab[53]   ),
        .I_xdig      (I_blue_lab[31:16]),
        .I_ydig      (I_blue_lab[15:0] ),
        .I_px        (S2_px            ),
        .I_py        (S2_py            ),
        .O_hit       (S2_lab_b         )
    );

    coord_label u_lab_green (
        .I_en        (S2_lab_en         ),
        .I_lab_valid (I_green_lab[54]   ),
        .I_lab_x     (I_green_lab[52:42]),
        .I_lab_y     (I_green_lab[41:32]),
        .I_xneg      (I_green_lab[53]   ),
        .I_xdig      (I_green_lab[31:16]),
        .I_ydig      (I_green_lab[15:0] ),
        .I_px        (S2_px             ),
        .I_py        (S2_py             ),
        .O_hit       (S2_lab_g          )
    );

    // 三色标签任一处命中即画白字(重叠时同色, 无优先级问题)
    wire [3:0] S2_lab_hit = S2_lab_r | S2_lab_b | S2_lab_g;

    //------------------------------------------------------------------------
    // 直通 + 颜色标注(mode 7): 在直通色调曲线上叠加红/蓝/绿物体外接矩形框
    //   红物体框 BOX_COLOR(绿), 蓝物体框 BLUE_BOX_COLOR(蓝), 绿物体框 GREEN_BOX_COLOR(黄), 1 像素宽;
    //   S2_box_hit/S2_blue_hit/S2_green_hit 已与 S2_curve 对齐。
    //------------------------------------------------------------------------
    localparam [23:0] BOX_COLOR        = 24'h00FF00;   // 绿色框(红物体)
    localparam [23:0] RED_MASK_COLOR   = 24'hFF00FF;   // 洋红: 判红像素调试涂色
    localparam [23:0] BLUE_BOX_COLOR   = 24'h0000FF;   // 蓝色框(蓝物体)
    localparam [23:0] BLUE_MASK_COLOR  = 24'h00FFFF;   // 青:   判蓝像素调试涂色
    localparam [23:0] GREEN_BOX_COLOR  = 24'hFFFF00;   // 黄色框(绿物体)
    localparam [23:0] GREEN_MASK_COLOR = 24'hFFFFFF;   // 白:   判绿像素调试涂色

    // 优先级: 判绿像素(白) > 判蓝像素(青) > 判红像素(洋红) > 绿框(黄) > 蓝框 > 红框(绿) > 正常画面
    wire [95:0] S2_box_data = {
        S2_lab_hit[3] ? LAB_TEXT_COLOR : S2_green_mask[3] ? GREEN_MASK_COLOR : (S2_blue_mask[3] ? BLUE_MASK_COLOR : (S2_red_mask[3] ? RED_MASK_COLOR : (S2_green_hit[3] ? GREEN_BOX_COLOR : (S2_blue_hit[3] ? BLUE_BOX_COLOR : (S2_box_hit[3] ? BOX_COLOR : S2_curve[95:72]))))),
        S2_lab_hit[2] ? LAB_TEXT_COLOR : S2_green_mask[2] ? GREEN_MASK_COLOR : (S2_blue_mask[2] ? BLUE_MASK_COLOR : (S2_red_mask[2] ? RED_MASK_COLOR : (S2_green_hit[2] ? GREEN_BOX_COLOR : (S2_blue_hit[2] ? BLUE_BOX_COLOR : (S2_box_hit[2] ? BOX_COLOR : S2_curve[71:48]))))),
        S2_lab_hit[1] ? LAB_TEXT_COLOR : S2_green_mask[1] ? GREEN_MASK_COLOR : (S2_blue_mask[1] ? BLUE_MASK_COLOR : (S2_red_mask[1] ? RED_MASK_COLOR : (S2_green_hit[1] ? GREEN_BOX_COLOR : (S2_blue_hit[1] ? BLUE_BOX_COLOR : (S2_box_hit[1] ? BOX_COLOR : S2_curve[47:24]))))),
        S2_lab_hit[0] ? LAB_TEXT_COLOR : S2_green_mask[0] ? GREEN_MASK_COLOR : (S2_blue_mask[0] ? BLUE_MASK_COLOR : (S2_red_mask[0] ? RED_MASK_COLOR : (S2_green_hit[0] ? GREEN_BOX_COLOR : (S2_blue_hit[0] ? BLUE_BOX_COLOR : (S2_box_hit[0] ? BOX_COLOR : S2_curve[23:0])))))
    };

    always @(*) begin
        case (S2_mode)
            3'd1:    S3_sel_data = S2_gray_all;   // 灰度化
            3'd3:    S3_sel_data = ~S2_data;      // 反色（255 - x）
            3'd4:    S3_sel_data = S2_gamma_all;  // 伽马校正（全量曲线）
            3'd7:    S3_sel_data = S2_box_data;   // 直通 + 红色标注
            default: S3_sel_data = S2_curve;      // 0/6 直通：半量色调曲线
        endcase
    end

    //------------------------------------------------------------------------
    // 卡通化（mode 6 第 1 步）：在「直通档色调曲线」上提饱和 + 4bit 量化
    //
    // 【为什么源必须是 S2_curve，而不是原始 I_tdata】—— 这是"暗红色恐怖片
    //   滤镜"的真正根因，两个问题叠在一起：
    //
    //   a) 缺了抬暗部。S2_curve = ( x + gamma(x) ) >> 1 是直通档给人看的画面，
    //      它的作用就是"抬暗部、压高光"。卡通档原来直接量化原始 I_tdata，
    //      于是比其它所有档都暗一截，暗场景下几乎只剩黑块。
    //
    //   b) 靠逐通道硬饱和来补亮 -> 偏红。原来用 F_lift = x*1.5 饱和提亮，
    //      而 AWB 已经把 R 抬了 1.4375x、B 抬了 1.21875x，于是 R 最先撞 255、
    //      G/B 还没撞：亮部 R 被钉在 255 而 G/B 继续增长，三通道比例被破坏，
    //      整片亮区发红；加上黑色描边，就是"暗红色恐怖片滤镜"。
    //      gamma 曲线是单调映射、值域仍是 0..255、永不削顶，换用它之后
    //      既提亮又完全不改色相。
    //
    // 【饱和 1.25x】c' = c + ((c - Y) >> 2)，纯移位加减，无 DSP。
    //   减色之后色块会发灰，这一步把色块拉鲜艳；以 Y 为中心对称扩张，
    //   不改变色相，所以不会重新引入偏色。
    //
    // 【4bit 量化用四舍五入】q = (x + 8) 取高 4bit。直接截断平均会让画面
    //   暗 8 级（约 3%），四舍五入没有这个直流偏置；x >= 248 时钉在 240，
    //   9bit 中间量保证不溢出。
    //------------------------------------------------------------------------
    function [7:0] F_sat125;
        input [7:0] c;
        input [7:0] y;
        reg signed [10:0] d;
        reg signed [10:0] t;
        begin
            d = $signed({2'b00, c}) - $signed({2'b00, y});   // -255..255
            t = $signed({2'b00, c}) + (d >>> 2);             // ±64 以内
            if (t < 0)
                F_sat125 = 8'd0;
            else if (t > 255)
                F_sat125 = 8'd255;
            else
                F_sat125 = t[7:0];
        end
    endfunction

    function [7:0] F_quant4;
        input [7:0] x;
        reg   [8:0] t;
        begin
            t = {1'b0, x} + 9'd8;                              // 四舍五入
            F_quant4 = t[8] ? 8'hF0 : {t[7:4], 4'b0000};       // 取高 4bit
        end
    endfunction

    //------------------------------------------------------------------------
    // 【v4 自动色阶】F_stretch：以整帧平均亮度 S_y_pivot 为支点做 2x 拉伸。
    //
    // 为什么非加不可：v3 的实测是"整幅糊成一片粉色雾，只有顶灯周围有黑边"。
    //   原因是室内有效像素大量挤在 100~170 这一小段，直接 16 级量化会把它压成
    //   2~3 个色阶 —— 表现为平坦色块 + 没有层次。先围绕支点拉 2 倍，让减色前的
    //   画面接近满量程，量化出来的才是"有结构的大色块"而不是"一片雾"。
    //
    //   支点跟着整帧均值走（自动色阶），所以房间变亮变暗、曝光漂移都能自适应，
    //   不需要用户去按亮度键硬补。
    //   逐通道独立做，只改亮度不改色相，因此不会重新引入偏色。
    //   代价：纯移位加法，0 个 DSP。
    //------------------------------------------------------------------------
    function [7:0] F_stretch;
        input [7:0] x;
        input [7:0] p;
        reg signed [10:0] d;
        reg signed [11:0] t;
        begin
            d = $signed({2'b00, x}) - $signed({2'b00, p});   // -255..255
            t = 12'sd128 + (d <<< 1);                        // 以 128 为支点 2x
            if (t < 0)
                F_stretch = 8'd0;
            else if (t > 255)
                F_stretch = 8'd255;
            else
                F_stretch = t[7:0];
        end
    endfunction

    //------------------------------------------------------------------------
    // 【v4 关键】减色只在"亮度域"做，色度原样保留。
    //
    //   v3 的实测是"整幅一层粉红雾，只有顶灯周围有黑边"，根因有两个：
    //   1) 逐通道量化：中性灰在 AWB 之后 R/G/B 只差 1 个 LSB，但正好踩在 16 级
    //      量化的台阶边界上（104->96、106->112），整幅中性色一起跳成洋红
    //      —— 1 LSB 的失衡被放大成 16 级（6%）偏色。
    //   2) 减色前没有色阶拉伸：室内有效像素挤在 100~170，16 级量化只剩 3 级，
    //      于是"糊成一片"。
    //
    //   现在的做法：
    //     Y  = (R+2G+B)>>2              （色阶前，未拉伸）
    //     Yq = F_quant4(Y)              -> 16 级硬边色块
    //     Ys = F_stretch(Yq, 整帧均值)   -> 自动色阶，占满量程
    //     通道 = clamp(Ys + 1.5*(C - Y)) -> 色度只做 1.5x 增强，不进量化器
    //
    //   关键：色度用的是"拉伸前"的差值（只有 ±2 个 LSB），放大 1.5 倍后仍然
    //   很小，不可能把整幅拉到某个色相上；而亮度经过拉伸，层次是满的。
    //   代价：纯加减 / 移位，0 个 DSP。
    //------------------------------------------------------------------------

    // 4 个像素的亮度（拉伸前，与色度同一时刻、同一来源）
    wire [7:0] S2c_y0 = F_gray(S2_curve[23:16], S2_curve[15:8], S2_curve[7:0]);
    wire [7:0] S2c_y1 = F_gray(S2_curve[47:40], S2_curve[39:32], S2_curve[31:24]);
    wire [7:0] S2c_y2 = F_gray(S2_curve[71:64], S2_curve[63:56], S2_curve[55:48]);
    wire [7:0] S2c_y3 = F_gray(S2_curve[95:88], S2_curve[87:80], S2_curve[79:72]);

    // 量化 16 级后再做自动色阶 -> 每个像素的"底色"
    wire [7:0] S2c_ys0 = F_stretch(F_quant4(S2c_y0), S_y_pivot);
    wire [7:0] S2c_ys1 = F_stretch(F_quant4(S2c_y1), S_y_pivot);
    wire [7:0] S2c_ys2 = F_stretch(F_quant4(S2c_y2), S_y_pivot);
    wire [7:0] S2c_ys3 = F_stretch(F_quant4(S2c_y3), S_y_pivot);

    function [7:0] F_cart;
        input [7:0] c;      // 拉伸前的通道
        input [7:0] y;      // 拉伸前的亮度
        input [7:0] ys;     // 量化 + 拉伸后的底色
        reg signed [10:0] d;
        reg signed [11:0] t;
        begin
            d = ({2'b00, c}) - ({2'b00, y});   // 色度 -255..255
            t = ({2'b00, ys}) + d + (d >>> 1);        // 底色 + 1.5*chroma
            if (t < 0)
                F_cart = 8'd0;
            else if (t > 255)
                F_cart = 8'd255;
            else
                F_cart = t[7:0];
        end
    endfunction

    wire [95:0] cartoon_q;
    assign cartoon_q[23:16] = F_cart(S2_curve[23:16], S2c_y0, S2c_ys0);   // px0 R
    assign cartoon_q[15:8]  = F_cart(S2_curve[15:8],  S2c_y0, S2c_ys0);   // px0 G
    assign cartoon_q[7:0]   = F_cart(S2_curve[7:0],   S2c_y0, S2c_ys0);   // px0 B
    assign cartoon_q[47:40] = F_cart(S2_curve[47:40], S2c_y1, S2c_ys1);   // px1 R
    assign cartoon_q[39:32] = F_cart(S2_curve[39:32], S2c_y1, S2c_ys1);   // px1 G
    assign cartoon_q[31:24] = F_cart(S2_curve[31:24], S2c_y1, S2c_ys1);   // px1 B
    assign cartoon_q[71:64] = F_cart(S2_curve[71:64], S2c_y2, S2c_ys2);   // px2 R
    assign cartoon_q[63:56] = F_cart(S2_curve[63:56], S2c_y2, S2c_ys2);   // px2 G
    assign cartoon_q[55:48] = F_cart(S2_curve[55:48], S2c_y2, S2c_ys2);   // px2 B
    assign cartoon_q[95:88] = F_cart(S2_curve[95:88], S2c_y3, S2c_ys3);   // px3 R
    assign cartoon_q[87:80] = F_cart(S2_curve[87:80], S2c_y3, S2c_ys3);   // px3 G
    assign cartoon_q[79:72] = F_cart(S2_curve[79:72], S2c_y3, S2c_ys3);   // px3 B

    //------------------------------------------------------------------------
    // 边缘检测（mode 5 & mode 6）：与 S1/S2/S3 流水并行运行，常开。
    //
    // 【输入必须始终是原始 I_tdata，不能换成量化图】
    //   把 3bit/4bit 量化图送进 Sobel 会出两个问题：
    //   1) 量化会在平滑渐变区造出"色带轮廓"，每一圈都超门限 -> 假描边遍地；
    //   2) 暗场景量化后大片像素落到同一级，真边缘反而一起消失。
    //   之前 mode 6 画面几乎全黑就是这个原因（假描边把整幅盖成黑）。
    //   原始 8bit 数据的梯度是连续的，Sobel 只对真实亮度跳变响应。
    //
    // 【延迟】数据路径的延迟是 4 拍，不是 644 拍。
    //   edge_detect 内部 cur_d1..cur_d3 与 valid_2_d1 同为 3 级，再各加输出
    //   寄存器 1 级，所以 O_tdata 相对 I_tdata 只晚 4 拍；行缓冲是"每行 320
    //   组数据"的环形缓冲，只影响 3x3 窗口的取行，不额外增加输出延迟。
    //   文件里那个 L_SR=643 的移位寄存器是给 tuser/tlast 用的（按时钟计），
    //   和数据路径不同步，那是个已知缺陷，但 mode 5 一直这么跑，不动它。
    //   所以 cartoon_delay 的拍数要按 4 定，而不是 644 或 323。
    //
    // 档位切换瞬间存在 1 帧过渡毛刺（S3 与 edge 帧结构不同步），属可接受现象。
    //------------------------------------------------------------------------
    wire [95:0] edge_O_tdata;
    wire        edge_O_tvalid;
    wire        edge_O_tuser;
    wire        edge_O_tlast;

    edge_detect #(
        .IMG_WIDTH   (1280),
        .IMG_HEIGHT  (720 )
    ) u_edge_detect (
        .I_clk    (I_clk        ),
        .I_rst_n  (I_rst_n      ),
        .I_tdata  (I_tdata      ),
        .I_tvalid (I_tvalid     ),
        .I_tuser  (I_tuser      ),
        .I_tlast  (I_tlast      ),
        .O_tdata  (edge_O_tdata ),
        .O_tvalid (edge_O_tvalid),
        .O_tuser  (edge_O_tuser ),
        .O_tlast  (edge_O_tlast )
    );

    //------------------------------------------------------------------------
    // 滑动窗口二值化（mode 2）：与 S1/S2/S3 流水并行运行，常开。
    // 其输出延迟 4818 拍（15 行缓存 + 16x16 窗口），远大于 S3 的 3 拍，
    // 故 mode 2 直接采用 binarize_sw 自带的 tvalid/tuser/tlast（内部已对齐）。
    // 档位切换瞬间存在 1 帧过渡毛刺（S3 与 binarize_sw 帧结构不同步），同 mode 5。
    //------------------------------------------------------------------------
    wire [95:0] bin_O_tdata;
    wire        bin_O_tvalid;
    wire        bin_O_tuser;
    wire        bin_O_tlast;

    binarize_sw #(
        .IMG_WIDTH    (1280),
        .IMG_HEIGHT   (720 )
    ) u_binarize_sw (
        .I_clk    (I_clk       ),
        .I_rst_n  (I_rst_n     ),
        .I_tdata  (I_tdata     ),
        .I_tvalid (I_tvalid    ),
        .I_tuser  (I_tuser     ),
        .I_tlast  (I_tlast     ),
        .O_tdata  (bin_O_tdata ),
        .O_tvalid (bin_O_tvalid),
        .O_tuser  (bin_O_tuser ),
        .O_tlast  (bin_O_tlast )
    );

    //------------------------------------------------------------------------
    // 卡通化（mode 6）：亮度分层色块 + 边缘黑描边
    //   1) 减色见上方 cartoon_q（源是 S2_curve：2x 自动色阶 -> 亮度 16 级量化
    //      + 色度 1.5x 保留）
    //   2) edge_detect 始终吃原始 I_tdata，描边落在真实亮度跳变上
    //   3) 卡通色经 cartoon_delay 延迟 2 拍，与描边对齐
    //   4) 描边像素 -> 纯黑；非描边 -> 卡通色
    //
    // 为什么只延迟 2 拍（不是 4 拍）：
    //   cartoon_q 的源是 S2_curve，它组合在 S2 级寄存器上，比 I_tdata 晚 2 拍；
    //   而 edge_O_tdata 比 I_tdata 晚 4 拍。两者差 4-2 = 2 拍，正好是这里的拍数。
    //   （v2 版量化的是原始 I_tdata，那时才需要 4 拍。）
    //
    // 为什么不是 BRAM 环形缓冲：
    //   延迟只需要 2 拍，2 x 48bit = 96 个触发器就够了，用 RAM 反而要
    //   实例化 ERAM、还要按"有效拍"计数（edge_detect 是按时钟打拍，单位
    //   不一致会横向错几个像素）。纯寄存器链与 edge_detect 同为每时钟一拍，
    //   天然对齐。
    //
    // 资源：0 DSP、0 ERAM、约 100 FF + 约 120 LUT
    //------------------------------------------------------------------------
    localparam integer CT_STAGES = 2;   // S2_curve(t+2) -> edge_O_tdata(t+4)

    // v4 起卡通色是完整 96bit（亮度已量化、色度保留），不再压成 12x4bit，
    // 所以这里直接原样延迟。96bit x 2 级 = 192 个 FF，仍不占 ERAM。
    wire [95:0] cartoon_dout;

    cartoon_delay #(
        .WIDTH (96        ),
        .STAGES(CT_STAGES )
    ) u_cartoon_delay (
        .I_clk   (I_clk        ),
        .I_rst_n (I_rst_n      ),
        .I_din   (cartoon_q    ),
        .O_dout  (cartoon_dout )
    );

    wire [95:0] cartoon_d = cartoon_dout;

    // 描边 -> 纯黑，非描边 -> 卡通色
    wire [95:0] cartoon_out;
    assign cartoon_out[23:0]  = (|edge_O_tdata[23:0])  ? 24'h000000 : cartoon_d[23:0];
    assign cartoon_out[47:24] = (|edge_O_tdata[47:24]) ? 24'h000000 : cartoon_d[47:24];
    assign cartoon_out[71:48] = (|edge_O_tdata[71:48]) ? 24'h000000 : cartoon_d[71:48];
    assign cartoon_out[95:72] = (|edge_O_tdata[95:72]) ? 24'h000000 : cartoon_d[95:72];

    //------------------------------------------------------------------------
    // 输出级：先在各个模式之间选出数据源，再统一做亮度/对比度，最后寄存
    //   —— 这样 K2(亮度)/K3(对比度) 对全部模式都生效
    //      （原实现把 F_bc 放在第 1 级且只喂给直通档，其它档按键没反应）
    //------------------------------------------------------------------------
    wire [95:0] S3_any_data  = (S2_mode == 3'd6) ? cartoon_out  :
                               (S2_mode == 3'd5) ? edge_O_tdata  :
                               (S2_mode == 3'd2) ? bin_O_tdata   :
                                                   S3_sel_data;

    wire        S3_any_valid = (S2_mode == 3'd6) ? edge_O_tvalid :
                               (S2_mode == 3'd5) ? edge_O_tvalid :
                               (S2_mode == 3'd2) ? bin_O_tvalid  :
                                                   S2_valid;
    wire        S3_any_user  = (S2_mode == 3'd6) ? edge_O_tuser  :
                               (S2_mode == 3'd5) ? edge_O_tuser  :
                               (S2_mode == 3'd2) ? bin_O_tuser   :
                                                   S2_user;
    wire        S3_any_last  = (S2_mode == 3'd6) ? edge_O_tlast  :
                               (S2_mode == 3'd5) ? edge_O_tlast  :
                               (S2_mode == 3'd2) ? bin_O_tlast   :
                                                   S2_last;

    // 12 个通道逐个做亮度/对比度（纯移位加法，零 DSP，仍只例化 12 个 F_bc）
    wire [95:0] S3_bc_data = {
        F_bc(S3_any_data[95:88], S_bright_act, S_contrast_act),   // px3 R
        F_bc(S3_any_data[87:80], S_bright_act, S_contrast_act),   // px3 G
        F_bc(S3_any_data[79:72], S_bright_act, S_contrast_act),   // px3 B
        F_bc(S3_any_data[71:64], S_bright_act, S_contrast_act),   // px2 R
        F_bc(S3_any_data[63:56], S_bright_act, S_contrast_act),   // px2 G
        F_bc(S3_any_data[55:48], S_bright_act, S_contrast_act),   // px2 B
        F_bc(S3_any_data[47:40], S_bright_act, S_contrast_act),   // px1 R
        F_bc(S3_any_data[39:32], S_bright_act, S_contrast_act),   // px1 G
        F_bc(S3_any_data[31:24], S_bright_act, S_contrast_act),   // px1 B
        F_bc(S3_any_data[23:16], S_bright_act, S_contrast_act),   // px0 R
        F_bc(S3_any_data[15:8],  S_bright_act, S_contrast_act),   // px0 G
        F_bc(S3_any_data[7:0],   S_bright_act, S_contrast_act)    // px0 B
    };

    // 输出寄存器
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tdata  <= 96'd0;
            O_tvalid <= 1'b0;
            O_tuser  <= 1'b0;
            O_tlast  <= 1'b0;
        end else begin
            O_tdata  <= S3_bc_data;
            O_tvalid <= S3_any_valid;
            O_tuser  <= S3_any_user;
            O_tlast  <= S3_any_last;
        end
    end

    //========================================================================
    // 函数定义
    //========================================================================

    //------------------------------------------------------------------------
    // F_gray: 灰度值计算（移位近似，无 DSP）
    // Y = (R + 2G + B) >> 2 = 0.25R + 0.5G + 0.25B
    // 相比标准 BT.601（0.299R + 0.587G + 0.114B）为常用低代价近似；
    // 二值化只依赖亮度单调性，结果几乎无差，灰度显示也足够。
    // 最大中间值：255 + 510 + 255 = 1020（10bit，不溢出），>>2 后 0..255。
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
    // F_gamma: 伽马校正（gamma=2.2，亮度提升曲线）
    // 8 段折线近似：y = 255 * (x/255)^(1/2.2)
    // 分段依据：x[7:5] 选择段号（0~7），x[4:0] 为段内局部值（0~31）
    // 斜率为 8.8 定点数（真实斜率 x 256），计算后右移 8 位还原
    // 结果超过 255 时饱和到 255
    //------------------------------------------------------------------------
    function [7:0] F_gamma;
        input [7:0] x;
        integer S_g_base;
        integer S_g_slope;
        integer S_g_result;
        begin
            case (x[7:5])
                // 段 0: x=0..31,   基准值=0,   斜率=818
                3'd0: begin S_g_base = 0;   S_g_slope = 818; end
                // 段 1: x=32..63,  基准值=99,  斜率=306
                3'd1: begin S_g_base = 99;  S_g_slope = 306; end
                // 段 2: x=64..95,  基准值=136, 斜率=232
                3'd2: begin S_g_base = 136; S_g_slope = 232; end
                // 段 3: x=96..127, 基准值=164, 斜率=182
                3'd3: begin S_g_base = 164; S_g_slope = 182; end
                // 段 4: x=128..159,基准值=186, 斜率=166
                3'd4: begin S_g_base = 186; S_g_slope = 166; end
                // 段 5: x=160..191,基准值=206, 斜率=149
                3'd5: begin S_g_base = 206; S_g_slope = 149; end
                // 段 6: x=192..223,基准值=224, 斜率=133
                3'd6: begin S_g_base = 224; S_g_slope = 133; end
                // 段 7: x=224..255,基准值=240, 斜率=124
                3'd7: begin S_g_base = 240; S_g_slope = 124; end
            endcase
            // y = base + (local * slope) >> 8
            S_g_result = S_g_base + ((x[4:0] * S_g_slope) >> 8);
            // 饱和保护：超过 255 则钳位到 255
            if (S_g_result > 255)
                F_gamma = 8'd255;
            else
                F_gamma = S_g_result[7:0];
        end
    endfunction

    //------------------------------------------------------------------------
    // bright_lut: 亮度增益查表（Q8 定点，256=1.0x，11bit 容纳最大 4x=1024）
    //------------------------------------------------------------------------
    function [10:0] bright_lut;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: bright_lut = 11'd256;    // 1.00x
                3'd1: bright_lut = 11'd320;    // 1.25x
                3'd2: bright_lut = 11'd384;    // 1.50x
                3'd3: bright_lut = 11'd512;    // 2.00x
                3'd4: bright_lut = 11'd640;    // 2.50x
                3'd5: bright_lut = 11'd768;    // 3.00x
                3'd6: bright_lut = 11'd896;    // 3.50x
                3'd7: bright_lut = 11'd1024;   // 4.00x
                default: bright_lut = 11'd256;
            endcase
        end
    endfunction

    //------------------------------------------------------------------------
    // contrast_lut: 对比度系数查表（Q8 定点，256=1.0x，10bit 最大 2.5x=640）
    //------------------------------------------------------------------------
    function [9:0] contrast_lut;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: contrast_lut = 10'd128;   // 0.50x
                3'd1: contrast_lut = 10'd192;   // 0.75x
                3'd2: contrast_lut = 10'd256;   // 1.00x
                3'd3: contrast_lut = 10'd320;   // 1.25x
                3'd4: contrast_lut = 10'd384;   // 1.50x
                3'd5: contrast_lut = 10'd448;   // 1.75x
                3'd6: contrast_lut = 10'd512;   // 2.00x
                3'd7: contrast_lut = 10'd640;   // 2.50x
                default: contrast_lut = 10'd256;
            endcase
        end
    endfunction

    //------------------------------------------------------------------------
    // F_bc: 亮度/对比度处理（六个模式共用，放在输出级）——全移位加法，不占任何 DSP
    //   1) 对比度：以 128 为中心有符号拉伸  t = ((x-128)*C)>>8 + 128，饱和
    //        取 C = 32*k  =>  (dx*C)>>8 恒等于 (dx*k)>>3，gain = C/256 = k/8
    //        k  = {6,8,10,11,12,14,16,18}
    //             => 0.750 / 1.000 / 1.250 / 1.375 / 1.500 / 1.750 / 2.000 / 2.250
    //   2) 亮度：整体增益                 y = (t*B)>>8，饱和到 255
    //        取 B = 32*M  =>  (t*B)>>8 恒等于 (t*M)>>3，gain = M/8
    //        M  = {6,8,10,12,14,16,18,20}
    //             => 0.750 / 1.000 / 1.250 / 1.500 / 1.750 / 2.000 / 2.250 / 2.500
    //   两个旋钮都是 8 个位置(0..7)、7 个相邻间隔。
    //   第六版把亮度上限从 1.75x 抬到 2.50x（实测室内偏暗，1.75x 不够），
    //   并把默认改成「亮度 1.50x、对比度 1.00x」——偏暗场景下对比度拉高只会
    //   把暗部压得更黑，看起来反而更暗，所以默认不再预置 1.375x 对比度。
    //   k 上限 18、dx 上限 128 => |积| <= 2304，中间量用 13bit 有符号。
    //   M 上限 20、t 上限 255   => |积| <= 5100，中间量必须用 14bit 有符号（13bit 会溢出）。
    //------------------------------------------------------------------------

    // dx(-128..127) * k ，k 由档位选：6,8,10,11,12,14,16,18
    function signed [12:0] F_bc_mul_k;
        input signed [7:0] dx;
        input        [2:0] lv;
        reg   signed [12:0] x;
        begin
            x = {{5{dx[7]}}, dx};                       // 符号扩展到 13bit
            case (lv)
                3'd0:    F_bc_mul_k = (x <<< 2) + (x <<< 1);            // k=6   (0.750x)
                3'd1:    F_bc_mul_k = x <<< 3;                          // k=8   (1.000x)
                3'd2:    F_bc_mul_k = (x <<< 3) + (x <<< 1);            // k=10  (1.250x)
                3'd3:    F_bc_mul_k = (x <<< 3) + (x <<< 1) + x;        // k=11  (1.375x)
                3'd4:    F_bc_mul_k = (x <<< 3) + (x <<< 2);            // k=12  (1.500x)
                3'd5:    F_bc_mul_k = (x <<< 3) + (x <<< 2) + (x <<< 1);// k=14  (1.750x)
                3'd6:    F_bc_mul_k = x <<< 4;                          // k=16  (2.000x)
                default: F_bc_mul_k = (x <<< 4) + (x <<< 1);            // k=18  (2.250x)
            endcase
        end
    endfunction

    // t(0..255) * M ，M 由档位选：6,7,8,9,10,11,12,14
    // 【注意】t 是无符号量 0..255，绝对不能用 signed [7:0] 承接：
    //         否则 t>=128 会被解释成负数，亮度 >= 0.5 的像素全部被压成纯黑
    //         （之前灯光变黑洞就是这个原因）。
    function signed [13:0] F_bc_mul_m;
        input        [7:0] t;
        input        [2:0] lv;
        reg   signed [13:0] x;
        begin
            x = {6'b000000, t};                         // 零扩展到 14bit，恒为正
            case (lv)
                3'd0:    F_bc_mul_m = (x <<< 2) + (x <<< 1);            // M=6   (0.750x)
                3'd1:    F_bc_mul_m = x <<< 3;                          // M=8   (1.000x)
                3'd2:    F_bc_mul_m = (x <<< 3) + (x <<< 1);            // M=10  (1.250x)
                3'd3:    F_bc_mul_m = (x <<< 3) + (x <<< 2);            // M=12  (1.500x)
                3'd4:    F_bc_mul_m = (x <<< 3) + (x <<< 2) + (x <<< 1);// M=14  (1.750x)
                3'd5:    F_bc_mul_m = x <<< 4;                          // M=16  (2.000x)
                3'd6:    F_bc_mul_m = (x <<< 4) + (x <<< 1);            // M=18  (2.250x)
                default: F_bc_mul_m = (x <<< 4) + (x <<< 2);            // M=20  (2.500x)
            endcase
        end
    endfunction

    function [7:0] F_bc;
        input [7:0] x;
        input [2:0] lv_b;   // 亮度档位 0..7
        input [2:0] lv_c;   // 对比度档位 0..7
        reg signed [12:0] dx;
        reg signed [12:0] t;
        reg signed [13:0] tb;
        begin
            // 对比度：以 128 为中心拉伸
            dx = $signed({1'b0, x}) - 13'sd128;          // -128 .. +127
            t  = F_bc_mul_k(dx[7:0], lv_c) >>> 3;        // (dx*k)>>3 == (dx*C)>>8
            t  = t + 13'sd128;
            if (t < 0) t = 0;
            else if (t > 255) t = 255;
            // 亮度：整体增益
            tb = F_bc_mul_m(t[7:0], lv_b) >>> 3;         // (t*M)>>3 == (t*B)>>8
            if (tb < 0) tb = 0;
            else if (tb > 255) tb = 255;
            F_bc = tb[7:0];
        end
    endfunction

endmodule

