//============================================================================
// coord_batch.v -- 三色坐标批次收集 + 跨时钟域握手(相机域 -> 串口域)
//
// 背景:
//   object_detect_3color 是时分复用, 帧 N 判红 / 帧 N+1 判蓝 / 帧 N+2 判绿,
//   每帧只更新一种颜色。串口要一次把三色坐标都送出去, 所以这里做"批次":
//   在绿色 pass 结束(一轮走完)时, 把本次轮次的红/蓝/绿坐标一起锁存成
//   一个快照, 再翻 1bit toggle 跨到 100MHz 串口域。
//
// 坐标定义(与机械臂固件 main.c 的 PixelToArm 一致):
//   原点 = 图像底边中点 = 像素 (640, 720)
//   px = 框中心 x - 640        向右为正, 范围 -640..+640
//   py = 720 - 框中心 y        向上为正, 范围 0..720
//   中心取 (min+max+1)/2 的 2 倍形式再除 2, 全程整数。
//
// 每个条目 30bit:  {py[9:0], px[10:0], type[7:0], valid}
//   O_batch[29:0]  = 红   ('R' = 0x52)
//   O_batch[59:30] = 蓝   ('B' = 0x42)
//   O_batch[89:60] = 绿   ('G' = 0x47)
//
// 跨时钟域:
//   先写数据(O_batch), 下一拍才翻 O_tgl; 接收端三级同步 + 边沿检测后再等
//   一拍采数据, 保证采到的多 bit 数据在 toggle 变化前后始终稳定。
//   (数据每次更新保持 3 帧 ≈ 50ms, 远大于同步延迟, 满足 data+toggle 握手前提)
//
// 资源: 3 组暂存 + 1 组 90bit 快照 + 两个小加法器/移位, 无乘法无除法。
//============================================================================
module coord_batch #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720
)(
    input  wire        I_clk,        // 相机域 S_csi_rx_clk (60MHz)
    input  wire        I_rst_n,
    input  wire        I_en,         // 仅在识别模式(mode 7)才提交批次

    // 红色: bbox 已更新脉冲(检测器 frame_done 延迟 1 拍) + 当前框
    input  wire        I_r_upd,
    input  wire        I_r_valid,
    input  wire [10:0] I_r_minx,
    input  wire [9:0]  I_r_miny,
    input  wire [10:0] I_r_maxx,
    input  wire [9:0]  I_r_maxy,

    // 蓝色
    input  wire        I_b_upd,
    input  wire        I_b_valid,
    input  wire [10:0] I_b_minx,
    input  wire [9:0]  I_b_miny,
    input  wire [10:0] I_b_maxx,
    input  wire [9:0]  I_b_maxy,

    // 绿色(本轮最后一个 pass, 用它做提交点)
    input  wire        I_g_upd,
    input  wire        I_g_valid,
    input  wire [10:0] I_g_minx,
    input  wire [9:0]  I_g_miny,
    input  wire [10:0] I_g_maxx,
    input  wire [9:0]  I_g_maxy,

    output reg  [89:0] O_batch,      // 三色快照(见文件头位域说明)
    output reg         O_tgl         // 新快照指示, 每提交一次翻转
);

    localparam [12:0] HALF_W2 = IMG_WIDTH;       // 2 x (W/2) = 1280
    localparam [11:0] H2      = IMG_HEIGHT * 2;  // 1440

    //------------------------------------------------------------------------
    // 框中心相对原点的偏移(像素)
    //   px = (min+max+1)/2 - 640
    //   py = 720 - (min+max+1)/2
    //
    // 全程无符号做, 不依赖 signed 运算:
    //   c  = mn+mx+1            (13bit, 0..2559)
    //   d  = c - 1280  (mod 2^13) -> 负数时自然回卷成补码
    //   取 d[11:1] 就是"算术右移 1 位后的低 11 位", 正是 11bit 有符号结果
    //------------------------------------------------------------------------
    function [10:0] F_px;
        input [10:0] mn, mx;
        reg [12:0] c, d;
        begin
            c    = {2'b00, mn} + {2'b00, mx} + 13'd1;
            d    = c - HALF_W2;
            F_px = d[11:1];
        end
    endfunction

    function [9:0] F_py;
        input [9:0] mn, mx;
        reg [11:0] s;
        begin
            s    = H2 - ({2'b00, mn} + {2'b00, mx} + 12'd1);
            F_py = s[11:1];
        end
    endfunction

    //------------------------------------------------------------------------
    // 每色最新坐标暂存(各自 frame_done 时更新)
    //------------------------------------------------------------------------
    reg        r_v, b_v;
    reg [10:0] r_px, b_px;
    reg [9:0]  r_py, b_py;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            r_v  <= 1'b0;
            r_px <= 11'd0;
            r_py <= 10'd0;
            b_v  <= 1'b0;
            b_px <= 11'd0;
            b_py <= 10'd0;
        end else begin
            if (I_r_upd) begin
                r_v  <= I_r_valid;
                r_px <= F_px(I_r_minx, I_r_maxx);
                r_py <= F_py(I_r_miny, I_r_maxy);
            end
            if (I_b_upd) begin
                b_v  <= I_b_valid;
                b_px <= F_px(I_b_minx, I_b_maxx);
                b_py <= F_py(I_b_miny, I_b_maxy);
            end
        end
    end

    //------------------------------------------------------------------------
    // 打包: 绿用当拍输入(此刻 bbox 已更新 1 拍), 红蓝用暂存值
    //------------------------------------------------------------------------
    wire [29:0] g_pack = { F_py(I_g_miny, I_g_maxy), F_px(I_g_minx, I_g_maxx),
                           8'h47, I_g_valid };
    wire [29:0] b_pack = { b_py, b_px, 8'h42, b_v };
    wire [29:0] r_pack = { r_py, r_px, 8'h52, r_v };

    //------------------------------------------------------------------------
    // 提交: 绿色 bbox 更新脉冲作为一轮的收尾, 先写数据再翻 toggle
    //------------------------------------------------------------------------
    reg [1:0] st;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            st      <= 2'd0;
            O_batch <= 90'd0;
            O_tgl   <= 1'b0;
        end else begin
            case (st)
                2'd0: begin
                    if (I_g_upd && I_en) begin
                        O_batch <= { g_pack, b_pack, r_pack };
                        st      <= 2'd1;
                    end
                end
                2'd1: begin
                    // 数据已稳定至少 1 个相机周期, 再翻 toggle 通知串口域
                    O_tgl <= ~O_tgl;
                    st    <= 2'd0;
                end
                default: st <= 2'd0;
            endcase
        end
    end

endmodule
