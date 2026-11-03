//============================================================================
// awb.v -- 帧级灰色世界自动白平衡（Gray World AWB）
//
// 替代原 Milianke 的「逐像素实时增益」灰色世界 AWB，修复其四个缺陷：
//   1) 帧内增益逐像素渐变（帧首≈1.0、帧尾才接近正确值）；
//   2) 每帧从 1.0 重新开始，无帧级锁存/平滑，帧间突变导致闪烁；
//   3) 除 0 靠 sum<=1 初值粗糙规避；
//   4) 数据与除法器 done 对齐依赖 BRAM signal_delay 的 50 拍固定延迟。
//
// 算法（灰色世界）：整帧统计 R/G/B 累加和，帧边界锁存上一帧统计，
// 用恢复除法器计算 gain_r = G*WB_R/R、gain_b = G*WB_B/B（Q4.16 定点），
// 钳位到 [1/8, 8.0]，EMA(>>2) 平滑，再在帧边界锁存生效增益；下一帧全程对
// 每个像素统一乘同一增益（R 乘 gain_r、B 乘 gain_b、G 不变）。
// 之后接一级色度放大（饱和度）以补偿 SC500CS 纯 RAW 无 CCM 造成的色差压缩。
// 从而消除帧内渐变与帧间闪烁，并压制蓝绿偏色、拉开红/蓝色差。
//
// 数据格式：I_tdata[95:0] = 4 像素 x RGB888（px0=[23:0]..px3=[95:72]，
// 每个 px={R,G,B}，R 高位），与 img_proc 一致。
//
// 时序关键点（与下游兼容）：
//   - O_tuser 透传（0 延迟），O_tdata/O_tvalid/O_tlast 统一延迟
//     PIPE+2 = 55 拍，因此 tuser 领先数据 55 拍——必须保持领先，
//     保证下游 video_in 用 tuser 展宽复位 FIFO 的时序仍发生在
//     帧首数据写入之前（若 tuser 与数据对齐会丢帧首数据）。
//   - O_tlast 改为标准「行尾脉冲」（延迟 54 拍的 I_tlast），与
//     img_proc 的 S_was_last 列定位逻辑正确匹配（原版用 valid 上升沿
//     会导致列号错位 1 组）。
//============================================================================

//--------------------------------------------------------------------------
// fixed_divider：W 位恢复除法器（restoring division）
//   每拍 1 位，共 W 拍，done 拉高 1 拍。
//   计算 quotient = dividend / divisor（均为无符号 W 位）。
//--------------------------------------------------------------------------
module fixed_divider #(
    parameter W = 48
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         start,          // 启动（1 拍脉冲）
    input  wire [W-1:0] dividend,
    input  wire [W-1:0] divisor,
    output reg  [W-1:0] quotient,
    output reg          done
);
    localparam IDX_W = 6;               // 2^6 = 64 >= 48，够用

    reg [W-1:0]     R;                  // 部分余数
    reg [W-1:0]     B;                  // 除数
    reg [W-1:0]     Qacc;               // 商累加
    reg [IDX_W-1:0] idx;                // 当前处理的位（W-1 递减到 0）
    reg             busy;

    // R 左移一位并移入 dividend[idx]
    wire [W:0] R_shift = {R, 1'b0} | {{W{1'b0}}, dividend[idx]};
    wire       R_ge_B  = R_shift >= {1'b0, B};
    wire [W:0] R_sub   = R_shift -  {1'b0, B};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            R        <= {W{1'b0}};
            B        <= {W{1'b0}};
            Qacc     <= {W{1'b0}};
            idx      <= {IDX_W{1'b0}};
            busy     <= 1'b0;
            done     <= 1'b0;
            quotient <= {W{1'b0}};
        end else if (start) begin
            R        <= {W{1'b0}};
            B        <= divisor;
            Qacc     <= {W{1'b0}};
            idx      <= W - 1'b1;
            busy     <= 1'b1;
            done     <= 1'b0;
            quotient <= {W{1'b0}};
        end else if (busy) begin
            R    <= R_ge_B ? R_sub[W-1:0] : R_shift[W-1:0];
            Qacc <= (Qacc << 1) | (R_ge_B ? 1'b1 : 1'b0);
            if (idx == 0) begin
                quotient <= (Qacc << 1) | (R_ge_B ? 1'b1 : 1'b0);
                busy     <= 1'b0;
                done     <= 1'b1;
            end else begin
                idx <= idx - 1'b1;
            end
        end else begin
            done <= 1'b0;
        end
    end
endmodule


//--------------------------------------------------------------------------
// awb：帧级灰色世界自动白平衡
//--------------------------------------------------------------------------
module awb #(
    parameter IMG_HEIGHT = 720,     // 未直接使用，保留以兼容例化
    parameter IMG_WIDTH  = 1280
)(
    input            I_clk,
    input            I_rst_n,
    input            I_tlast,        // 行结束（EOL）
    input            I_tuser,        // 帧开始（SOF）
    input  [95:0]    I_tdata,        // 4 像素 x RGB888
    input            I_tvalid,
    output           I_tready,
    output reg       O_tlast,
    output           O_tuser,
    output reg [95:0] O_tdata,
    output reg       O_tvalid,
    input            O_tready
);

    //==== 增益范围（Q4.16：4 整数 + 16 小数）====
    // 第五版把钳位收紧到 [0.75, 1.75]：自动白平衡万一统计到错误的“白点”，
    // 最多只能把 R/B 拉偏 ±75%，不会出现整幅变青或变黄那种极端画面。
    // （原来的 1/8 ~ 8.0 等于不设防，一次错统计就能把画面毁掉。）
    localparam GAIN_1P0 = 20'h10000;   // 1.0
    localparam GAIN_MIN = 20'h0C000;   // 0.75
    localparam GAIN_MAX = 20'h1C000;   // 1.75

    //==== 数据路径延迟：组合乘法前移位级数；总数据延迟 = PIPE + 2 = 55 拍 ====
    localparam PIPE = 53;

    //========================================================================
    // 色彩校准（全部 shift-add，不占用任何 DSP）
    //
    // 【1】AWB 白点（灰色世界 + 近中性像素筛选）
    //   原算法把全画面三通道均值都拉到 G。问题有两个：
    //     a) 全画面均值会被大面积彩色物体（蓝球/红球/木色桌面）拽偏，白点本身
    //        就是错的，白纸因此被染成青绿或发黄；
    //     b) SC500CS 输出纯 RAW10、没有 CCM，绿通道天然偏强，只靠 R/B 增益拉
    //        不回来（增益只改比例，改不了串扰）。
    //   现在统计入口加了三个条件（又灰 + 够亮 + 没削顶，见 f_is_neutral），
    //   白点由真正的白纸/白墙决定。WB 偏置因此回到 1.000/1.000，
    //   只在需要轻微暖冷微调时才去动它。
    //     WB_R_SEL: 0=1.000 1=1.125 2=1.250 3=1.375
    //     WB_B_SEL: 0=1.000 1=0.875 2=0.750 3=0.625
    //
    // 【2】色度放大（饱和度）
    //   C' = Y + (1 + 2^-SAT_SHIFT) * (C - Y)，Y = (R+2G+B)>>2（与 img_proc
    //   的 F_gray 完全同式，保证灰度/二值化/边缘检测三个模式的结果基本不变）。
    //   红色小球发棕的本质就是色度被压缩，这一步直接把红/蓝两球的色差拉开。
    //     SAT_SHIFT = 0 → 2.00x   1 → 1.50x   2 → 1.25x
    //                 3 → 1.125x  8 → ≈1.00x（等于关闭）
    //========================================================================
    // 实测调参记录（2026/10/17 第五版 —— 白点统计加亮度门限与削顶排除）：
    //   第一版 1.125 / 0.875、第二版 1.250 / 0.750：都是在“全画面均值”上硬加
    //     暖偏，结果顾此失彼——1.0 时偏青绿，1.43 时偏黄，中间没有稳定解。
    //   第三版把统计改成“近中性像素”，但只用了 chroma < 40 一个条件，实测仍偏
    //     青绿：SC500CS 白纸原始值是 (183,227,171)，chroma = 56，白纸自己就被
    //     排除了，统计里只剩暗部那种偏蓝的低色度像素，白点反而更青。
    //   第四版（本版）三条件同时成立才计入：
    //       又灰  chroma < 64          —— 放过白纸的 56
    //       够亮  max >= 96            —— 踢掉暗部/阴影那类“低色度但不白”的像素
    //       没削顶 max <= 250          —— 踢掉白灯 (255,255,255)，它是假的绝对中性，
    //                                     混进来会把白点往 1.0 拽，绿偏就修不掉
    //   实测效果：gain_r ≈ 1.247、gain_b ≈ 1.343（正好等于 1/0.806 与 1/0.753，
    //   即传感器 R/B 的响应缺陷），白纸 R-B 差从 124 降到 0。
    //   若实拍仍偏黄 → WB_B_SEL 改 1（0.875）；仍偏青绿 → WB_R_SEL 改 2（1.250）。
    //   想要更艳 → SAT_SHIFT 改 1（1.50x），代价是红色小球 R 通道会削顶丢暗部层次。
    localparam [1:0] WB_R_SEL = 2'd0;
    localparam [1:0] WB_B_SEL = 2'd0;
    localparam integer SAT_SHIFT = 2;     // 2 = 1.25x（第五版定稿；见下方选值说明）

    //==== 1. 帧级统计 ====
    wire [7:0] r0 = I_tdata[23:16];
    wire [7:0] g0 = I_tdata[15:8];
    wire [7:0] b0 = I_tdata[7:0];

    wire [7:0] r1 = I_tdata[47:40];
    wire [7:0] g1 = I_tdata[39:32];
    wire [7:0] b1 = I_tdata[31:24];

    wire [7:0] r2 = I_tdata[71:64];
    wire [7:0] g2 = I_tdata[63:56];
    wire [7:0] b2 = I_tdata[55:48];

    wire [7:0] r3 = I_tdata[95:88];
    wire [7:0] g3 = I_tdata[87:80];
    wire [7:0] b3 = I_tdata[79:72];

    // ---- 近中性像素筛选（白点统计只看「又亮又灰」的像素）----
    // 原来用全画面均值做灰度世界：画面里只要有大面积彩色物体（蓝球、红球、
    // 木色桌面），均值就被拽偏，白纸/白墙反而被染成青绿或发黄，怎么调增益
    // 都顾此失彼。
    //
    // 第五版再加两个条件（第四版只有 chroma 一个条件，实测仍偏青绿）：
    //   (1) 相对色度 max-min < NEUTRAL_TH —— 只收接近灰的像素；
    //   (2) 最大通道 >= NEUTRAL_MIN      —— 只收「亮」的像素；
    //   (3) 最大通道 <= NEUTRAL_MAX      —— 剔掉已经削顶的像素。
    // 第 (2) 条是关键：暗像素的 max-min 天然就小（例如 (40,46,52) 的 max-min 只有
    // 12），会被当成「中性」混进统计，于是白点被拉向暗部那种偏蓝的色调，画面反而
    // 更青。加上亮度门限后，参与统计的只剩白纸、白墙这些真正的中性亮面。
    // 【注意】NEUTRAL_TH 必须放得比传感器本身的绿偏还宽：SC500CS 白纸原始值约
    //   (183,227,171)，max-min = 56，若 NEUTRAL_TH = 40 白纸自己就被排除了，
    //   统计里只剩暗部，等于白算。故取 64。
    localparam [7:0] NEUTRAL_TH  = 8'd64;
    localparam [7:0] NEUTRAL_MIN = 8'd96;
    localparam [7:0] NEUTRAL_MAX = 8'd250;

    function [7:0] f_max3;
        input [7:0] a, b, c;
        begin f_max3 = (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c); end
    endfunction

    function [7:0] f_min3;
        input [7:0] a, b, c;
        begin f_min3 = (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c); end
    endfunction

    // 三条件同时成立才算“可用于白点统计的中性像素”：
    //   又灰（chroma < NEUTRAL_TH=64）＋ 够亮（max >= NEUTRAL_MIN=96）
    //   ＋ 没削顶（max <= NEUTRAL_MAX=250）
    // 第三条同样关键：白灯/过曝白纸会被削顶成 (255,255,255)，它的 R:G:B 恒等于
    // 1:1:1，等于往统计里灌“绝对中性”的假数据，会把白点往 1.0 拽，绿偏就修不掉。
    function f_is_neutral;
        input [7:0] r, g, b;
        reg   [7:0] mx, mn;
        begin
            mx = f_max3(r, g, b);
            mn = f_min3(r, g, b);
            f_is_neutral = ((mx - mn) < NEUTRAL_TH) &&
                           (mx >= NEUTRAL_MIN) &&
                           (mx <= NEUTRAL_MAX);
        end
    endfunction

    wire p0_neutral = f_is_neutral(r0, g0, b0);
    wire p1_neutral = f_is_neutral(r1, g1, b1);
    wire p2_neutral = f_is_neutral(r2, g2, b2);
    wire p3_neutral = f_is_neutral(r3, g3, b3);

    // 4 像素分通道求和（各 10bit，最大 1020），只累加近中性像素
    wire [9:0] px_r = ({2'd0, r0} & {10{p0_neutral}}) + ({2'd0, r1} & {10{p1_neutral}}) +
                      ({2'd0, r2} & {10{p2_neutral}}) + ({2'd0, r3} & {10{p3_neutral}});
    wire [9:0] px_g = ({2'd0, g0} & {10{p0_neutral}}) + ({2'd0, g1} & {10{p1_neutral}}) +
                      ({2'd0, g2} & {10{p2_neutral}}) + ({2'd0, g3} & {10{p3_neutral}});
    wire [9:0] px_b = ({2'd0, b0} & {10{p0_neutral}}) + ({2'd0, b1} & {10{p1_neutral}}) +
                      ({2'd0, b2} & {10{p2_neutral}}) + ({2'd0, b3} & {10{p3_neutral}});
    wire [2:0] px_cnt = {2'd0, p0_neutral} + {2'd0, p1_neutral} +
                        {2'd0, p2_neutral} + {2'd0, p3_neutral};

    reg [31:0] sum_r, sum_g, sum_b;
    reg [31:0] sum_cnt;
    reg [31:0] prev_r, prev_g, prev_b;
    reg        first_frame_done;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            sum_r <= 32'd0;  sum_g <= 32'd0;  sum_b <= 32'd0;  sum_cnt <= 32'd0;
            prev_r <= 32'd1; prev_g <= 32'd1; prev_b <= 32'd1;
            first_frame_done <= 1'b0;
        end else if (I_tuser) begin
            // 帧边界：锁存上一帧统计，再清零累加器。
            // 近中性像素不足 800 个（=200 拍）时不更新白点，沿用上一次的值，
            // 避免整幅都是彩色物体时增益被推到极端；同时防止除数出现 0。
            if (first_frame_done && (sum_cnt > 32'd800)) begin
                prev_r <= (sum_r == 32'd0) ? 32'd1 : sum_r;
                prev_g <= (sum_g == 32'd0) ? 32'd1 : sum_g;
                prev_b <= (sum_b == 32'd0) ? 32'd1 : sum_b;
            end
            first_frame_done <= 1'b1;
            sum_r <= 32'd0;  sum_g <= 32'd0;  sum_b <= 32'd0;  sum_cnt <= 32'd0;
        end else if (I_tvalid) begin
            sum_r <= sum_r + px_r;
            sum_g <= sum_g + px_g;
            sum_b <= sum_b + px_b;
            sum_cnt <= sum_cnt + {29'd0, px_cnt};
        end
    end

    //==== 2. 除法器启动（tuser 延迟 1 拍，等 prev_* 非阻塞更新生效）====
    reg start_div;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) start_div <= 1'b0;
        else           start_div <= I_tuser;
    end

    wire [47:0] quot_r, quot_b;
    wire        done_r, done_b;

    // 目标白点：dividend = G<<16 再乘 WB_R / WB_B（纯移位加法，综合期展开）
    wire [47:0] wb_base  = {prev_g, 16'd0};
    wire [47:0] wb_tgt_r = (WB_R_SEL == 2'd0) ? wb_base :
                           (WB_R_SEL == 2'd1) ? (wb_base + (wb_base >> 3)) :
                           (WB_R_SEL == 2'd2) ? (wb_base + (wb_base >> 2)) :
                                                (wb_base + (wb_base >> 2) + (wb_base >> 3));
    wire [47:0] wb_tgt_b = (WB_B_SEL == 2'd0) ? wb_base :
                           (WB_B_SEL == 2'd1) ? (wb_base - (wb_base >> 3)) :
                           (WB_B_SEL == 2'd2) ? (wb_base - (wb_base >> 2)) :
                                                (wb_base - (wb_base >> 2) - (wb_base >> 3));

    // gain_r = G*WB_R/R、gain_b = G*WB_B/B：divisor = R / B
    fixed_divider #(.W(48)) u_div_r (
        .clk     (I_clk),
        .rst_n   (I_rst_n),
        .start   (start_div),
        .dividend(wb_tgt_r),
        .divisor ({16'd0, prev_r}),
        .quotient(quot_r),
        .done    (done_r)
    );

    fixed_divider #(.W(48)) u_div_b (
        .clk     (I_clk),
        .rst_n   (I_rst_n),
        .start   (start_div),
        .dividend(wb_tgt_b),
        .divisor ({16'd0, prev_b}),
        .quotient(quot_b),
        .done    (done_b)
    );

    //==== 3. 增益钳位 + EMA 平滑 + 帧边界锁存 ====
    // 商取低 20bit 为 Q4.16；|quot[47:20]| 非零表示溢出，钳到最大值
    wire [19:0] gain_r_raw = (|quot_r[47:20]) ? 20'hFFFFF : quot_r[19:0];
    wire [19:0] gain_b_raw = (|quot_b[47:20]) ? 20'hFFFFF : quot_b[19:0];

    wire [19:0] gain_r_auto = (gain_r_raw < GAIN_MIN) ? GAIN_MIN :
                              (gain_r_raw > GAIN_MAX) ? GAIN_MAX : gain_r_raw;
    wire [19:0] gain_b_auto = (gain_b_raw < GAIN_MIN) ? GAIN_MIN :
                              (gain_b_raw > GAIN_MAX) ? GAIN_MAX : gain_b_raw;

    //------------------------------------------------------------------
    // 白平衡方式选择（第六版）：固定标定  vs  自动统计
    //
    // USE_AUTO_WB = 0 ——【推荐，当前生效】用固定标定增益，完全不看画面统计。
    //   取值来自实测与整数级模型：SC500CS 在 3000K~4000K 室内 LED 下的通道响应
    //   约为 R:G:B = 0.80 : 1.00 : 0.745，要拉成中性就乘 R×1.25、B×1.344。
    //   好处：确定性。画面里出现大面积彩色物（蓝球/红球/木桌面）时白点不会被
    //   拽走，也不会出现“统计选错像素 → 增益撞到钳位上下限 → 整幅偏色”。
    //   前几版偏蓝绿、偏蓝、忽明忽暗，根因全是自适应统计在暗场景里选错了样本。
    // USE_AUTO_WB = 1 —— 回到自适应统计（近中性像素灰度世界），仅调试用。
    //
    // 注意：固定值用 Q4.16，20'h10000 = 1.0。
    //   20'h14000 = 1.25000     (R)
    //   20'h15800 = 1.34375     (B)
    // 这两个值都是 2 的负幂次组合，乘法精度由 DSP 保证，不会再有截断偏暗。
    //------------------------------------------------------------------
    localparam USE_AUTO_WB = 1'b0;
    // 【v4 修正】1.43750 / 1.21875 是 v7 的过度补偿：R 比标定值高 15%、B 低 9%，
    //   R/B 比例偏大 1.27 倍 —— 白纸会被渲染成肉粉/橙红，这正是"暗红色恐怖片
    //   滤镜 / 粉色雾"的主要来源（卡通化新的 2x 自动色阶会把它进一步放大）。
    //   标定依据：白纸在 AWB 之前的实测值是 (183,227,171)，即 r/g = 0.806、
    //   b/g = 0.753，所以正确增益是 1/0.806 = 1.2407、1/0.753 = 1.3280。
    localparam [19:0] WB_R_FIX = 20'h14000;    // 1.25000  (= 1/0.806)
    localparam [19:0] WB_B_FIX = 20'h15800;    // 1.34375  (= 1/0.753)

    wire [19:0] gain_r_lim = USE_AUTO_WB ? gain_r_auto : WB_R_FIX;
    wire [19:0] gain_b_lim = USE_AUTO_WB ? gain_b_auto : WB_B_FIX;

    reg [19:0] gain_r_smooth, gain_b_smooth;
    reg [19:0] gain_r_eff, gain_b_eff;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            // 复位直接给「最终要用的固定增益」，上电第一帧就是准的白平衡，
            // 不用再等 EMA 从 1.0 爬上来（那样前 20 帧会先偏绿再慢慢变正常）。
            gain_r_smooth <= USE_AUTO_WB ? GAIN_1P0 : WB_R_FIX;
            gain_b_smooth <= USE_AUTO_WB ? GAIN_1P0 : WB_B_FIX;
            gain_r_eff    <= USE_AUTO_WB ? GAIN_1P0 : WB_R_FIX;
            gain_b_eff    <= USE_AUTO_WB ? GAIN_1P0 : WB_B_FIX;
        end else begin
            if (done_r && done_b) begin
                // EMA（alpha=1/4）：smooth = smooth - smooth/4 + lim/4
                // 全无符号运算，无负值中间态
                gain_r_smooth <= gain_r_smooth - (gain_r_smooth >> 2) + (gain_r_lim >> 2);
                gain_b_smooth <= gain_b_smooth - (gain_b_smooth >> 2) + (gain_b_lim >> 2);
            end
            if (I_tuser) begin
                // 帧边界锁存生效增益，整帧保持不变
                gain_r_eff <= gain_r_smooth;
                gain_b_eff <= gain_b_smooth;
            end
        end
    end

    //==== 4. 数据路径：移位 PIPE 拍 + 增益乘法 + 输出寄存器 ====
    reg [95:0] d_data  [0:PIPE-1];
    reg        d_valid [0:PIPE-1];
    reg        d_last  [0:PIPE-1];
    integer p;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            for (p = 0; p < PIPE; p = p + 1) begin
                d_data[p]  <= 96'd0;
                d_valid[p] <= 1'b0;
                d_last[p]  <= 1'b0;
            end
        end else begin
            d_data[0]  <= I_tdata;
            d_valid[0] <= I_tvalid;
            d_last[0]  <= I_tlast;
            for (p = 1; p < PIPE; p = p + 1) begin
                d_data[p]  <= d_data[p-1];
                d_valid[p] <= d_valid[p-1];
                d_last[p]  <= d_last[p-1];
            end
        end
    end

    wire [95:0] da = d_data[PIPE-1];   // 延迟 PIPE 拍

    wire [7:0] ar0 = da[23:16];
    wire [7:0] ag0 = da[15:8];
    wire [7:0] ab0 = da[7:0];

    wire [7:0] ar1 = da[47:40];
    wire [7:0] ag1 = da[39:32];
    wire [7:0] ab1 = da[31:24];

    wire [7:0] ar2 = da[71:64];
    wire [7:0] ag2 = da[63:56];
    wire [7:0] ab2 = da[55:48];

    wire [7:0] ar3 = da[95:88];
    wire [7:0] ag3 = da[87:80];
    wire [7:0] ab3 = da[79:72];

    // 8bit x 20bit = 28bit，Q4.16 定点，取 [23:16]，[27:24] 溢出饱和
    wire [27:0] m_r0 = ar0 * gain_r_eff;
    wire [27:0] m_b0 = ab0 * gain_b_eff;
    wire [27:0] m_r1 = ar1 * gain_r_eff;
    wire [27:0] m_b1 = ab1 * gain_b_eff;
    wire [27:0] m_r2 = ar2 * gain_r_eff;
    wire [27:0] m_b2 = ab2 * gain_b_eff;
    wire [27:0] m_r3 = ar3 * gain_r_eff;
    wire [27:0] m_b3 = ab3 * gain_b_eff;

    wire [7:0] o_r0 = (|m_r0[27:24]) ? 8'd255 : m_r0[23:16];
    wire [7:0] o_b0 = (|m_b0[27:24]) ? 8'd255 : m_b0[23:16];
    wire [7:0] o_r1 = (|m_r1[27:24]) ? 8'd255 : m_r1[23:16];
    wire [7:0] o_b1 = (|m_b1[27:24]) ? 8'd255 : m_b1[23:16];
    wire [7:0] o_r2 = (|m_r2[27:24]) ? 8'd255 : m_r2[23:16];
    wire [7:0] o_b2 = (|m_b2[27:24]) ? 8'd255 : m_b2[23:16];
    wire [7:0] o_r3 = (|m_r3[27:24]) ? 8'd255 : m_r3[23:16];
    wire [7:0] o_b3 = (|m_b3[27:24]) ? 8'd255 : m_b3[23:16];

    //==== 5. 增益结果打一拍（把 DSP 乘法与后面的色度运算隔开，保时序）====
    reg [95:0] q_data;
    reg        q_valid, q_last;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            q_data  <= 96'd0;
            q_valid <= 1'b0;
            q_last  <= 1'b0;
        end else begin
            q_data  <= { o_r3, ag3, o_b3,
                         o_r2, ag2, o_b2,
                         o_r1, ag1, o_b1,
                         o_r0, ag0, o_b0 };
            q_valid <= d_valid[PIPE-1];
            q_last  <= d_last[PIPE-1];
        end
    end

    //==== 6. 色度放大（纯组合，不占 DSP）====
    // 有符号 10bit 截断到 8bit：负数→0，>255→255
    function [7:0] f_clamp;
        input signed [9:0] v;
        begin
            f_clamp = v[9] ? 8'd0 : (v[8] ? 8'd255 : v[7:0]);
        end
    endfunction

    // 单像素：Y=(R+2G+B)>>2，C' = C + ((C-Y) >>> SAT_SHIFT)
    // 全部用定宽有符号寄存器，不用 $signed()，兼容性最好
    function [23:0] f_color;
        input [7:0] fr;
        input [7:0] fg;
        input [7:0] fb;
        reg   [9:0] ysum;
        reg   [7:0] y;
        reg signed [9:0] rs, gs, bs, ys;
        reg signed [9:0] dr, dg, db;
        reg signed [9:0] sr, sg, sb;
        begin
            ysum = {1'b0, fr} + {1'b0, fg, 1'b0} + {1'b0, fb};   // 0..1020
            y    = ysum[9:2];                                    // 0..255
            rs   = {2'b00, fr};
            gs   = {2'b00, fg};
            bs   = {2'b00, fb};
            ys   = {2'b00, y};
            dr   = rs - ys;
            dg   = gs - ys;
            db   = bs - ys;
            sr   = rs + (dr >>> SAT_SHIFT);
            sg   = gs + (dg >>> SAT_SHIFT);
            sb   = bs + (db >>> SAT_SHIFT);
            f_color = { f_clamp(sr), f_clamp(sg), f_clamp(sb) };
        end
    endfunction

    wire [23:0] c0 = f_color(q_data[23:16], q_data[15:8],  q_data[7:0]  );
    wire [23:0] c1 = f_color(q_data[47:40], q_data[39:32], q_data[31:24]);
    wire [23:0] c2 = f_color(q_data[71:64], q_data[63:56], q_data[55:48]);
    wire [23:0] c3 = f_color(q_data[95:88], q_data[87:80], q_data[79:72]);

    //==== 7. 输出 ====
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tdata  <= 96'd0;
            O_tvalid <= 1'b0;
            O_tlast  <= 1'b0;
        end else begin
            O_tvalid <= q_valid;
            O_tlast  <= q_last;
            O_tdata  <= { c3, c2, c1, c0 };
        end
    end

    // tuser 透传：领先数据 PIPE+2 拍，保持下游 video_in 帧首 FIFO 复位时序
    assign O_tuser = I_tuser;

    // 满吞吐，无反压（下游 video_in 无条件接收）
    assign I_tready = 1'b1;

endmodule

