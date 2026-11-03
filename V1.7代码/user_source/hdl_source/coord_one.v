//============================================================================
// coord_one.v -- 单个颜色外接框 -> 屏上物理坐标 + 标签排版参数
//
// 坐标定义(比赛要求):
//   原点 = 显示屏底边中点 = 图像底边中点, 即像素 (IMG_WIDTH/2, IMG_HEIGHT)
//   x 轴向右为正, y 轴向上为正, 单位 cm, 显示保留 2 位小数(1/100 cm = 0.1mm)
//   取值点 = 外接框中心 = 三色方块中心(俯视 2D 平面, 忽略近大远小)
//
// 【标定】K_Q4 = 每像素对应多少个 0.01cm, Q4 定点(真实值 = K_Q4/16)
//     K_Q4 = 视野宽度(cm) x 100 x 16 / 1280 = 视野宽度(cm) x 1.25
//     视野 40cm -> 50(默认)   30cm -> 38   50cm -> 63   25cm -> 31
//   标定方法: 白纸上放直尺, 量出画面左右边缘之间的实际宽度, 代入上式即可。
//
// 实现要点:
//   * 只用移位/加减/常数比较, 无除法器、无 DSP;
//   * 每帧只在自身 frame_done 之后算一次(数据静止期), 结果打拍输出,
//     绝不进入像素通路关键路径;
//   * 像素中心取 (min+max+1)/2 的 2 倍形式, 全程整数, 无 0.5px 系统偏差。
//============================================================================
module coord_one #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter integer K_Q4       = 50       // 每像素 0.01cm 数, Q4 定点
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_upd,        // 该色 bbox 已更新(检测器 frame_done 延迟 1 拍)
    input  wire        I_bvalid,     // 该色本帧是否有框
    input  wire [10:0] I_minx,
    input  wire [9:0]  I_miny,
    input  wire [10:0] I_maxx,
    input  wire [9:0]  I_maxy,

    output reg         O_lab_valid,  // 标签有效(= I_bvalid)
    output reg  [10:0] O_lab_x,      // 标签左上角 x(已按 4 像素对齐)
    output reg  [9:0]  O_lab_y,      // 标签左上角 y
    output reg         O_xneg,       // x 为负(原点左侧)
    output reg  [15:0] O_xdig,       // {十位, 个位, 十分位, 百分位}, 4'hF=空白
    output reg  [15:0] O_ydig
);

    //------------------------------------------------------------------------
    // 标签尺寸(必须与 coord_label.v 保持一致)
    //   8 个字符 x 16px 间距 = 128;  2 行 x 24px 行距 = 48
    //------------------------------------------------------------------------
    localparam integer LAB_W   = 128;
    localparam integer LAB_H   = 48;
    localparam integer LAB_GAP = 6;

    localparam signed [12:0] HALF2 = IMG_WIDTH;         // 2 x (W/2) = 1280
    localparam        [12:0] BOT2  = IMG_HEIGHT * 2;    // 1440
    localparam        [11:0] Y_LIM = IMG_HEIGHT - LAB_H;
    localparam        [11:0] X_LIM = IMG_WIDTH  - LAB_W;

    //------------------------------------------------------------------------
    // 常数除法/取余(逐位长除法展开, 综合成比较+减法链, 不依赖综合器的除法推断)
    //   F_div100: v/100, v<4096 -> 0..40
    //   F_mod100: v%100,        -> 0..99
    //   F_div10 / F_mod10: 针对 0..99 -> 0..9
    //------------------------------------------------------------------------
    function [5:0] F_div100;
        input [11:0] v;
        reg [11:0] rem;
        reg [5:0]  q;
        begin
            rem = v;
            q   = 6'd0;
            if (rem >= 12'd3200) begin rem = rem - 12'd3200; q = q + 6'd32; end
            if (rem >= 12'd1600) begin rem = rem - 12'd1600; q = q + 6'd16; end
            if (rem >= 12'd800)  begin rem = rem - 12'd800;  q = q + 6'd8;  end
            if (rem >= 12'd400)  begin rem = rem - 12'd400;  q = q + 6'd4;  end
            if (rem >= 12'd200)  begin rem = rem - 12'd200;  q = q + 6'd2;  end
            if (rem >= 12'd100)  begin rem = rem - 12'd100;  q = q + 6'd1;  end
            F_div100 = q;
        end
    endfunction

    function [6:0] F_mod100;
        input [11:0] v;
        reg [11:0] rem;
        begin
            rem = v;
            if (rem >= 12'd3200) rem = rem - 12'd3200;
            if (rem >= 12'd1600) rem = rem - 12'd1600;
            if (rem >= 12'd800)  rem = rem - 12'd800;
            if (rem >= 12'd400)  rem = rem - 12'd400;
            if (rem >= 12'd200)  rem = rem - 12'd200;
            if (rem >= 12'd100)  rem = rem - 12'd100;
            F_mod100 = rem[6:0];
        end
    endfunction

    function [3:0] F_div10;
        input [6:0] v;
        reg [6:0] rem;
        reg [3:0] q;
        begin
            rem = v;
            q   = 4'd0;
            if (rem >= 7'd80) begin rem = rem - 7'd80; q = q + 4'd8; end
            if (rem >= 7'd40) begin rem = rem - 7'd40; q = q + 4'd4; end
            if (rem >= 7'd20) begin rem = rem - 7'd20; q = q + 4'd2; end
            if (rem >= 7'd10) begin rem = rem - 7'd10; q = q + 4'd1; end
            F_div10 = q;
        end
    endfunction

    function [3:0] F_mod10;
        input [6:0] v;
        reg [6:0] rem;
        begin
            rem = v;
            if (rem >= 7'd80) rem = rem - 7'd80;
            if (rem >= 7'd40) rem = rem - 7'd40;
            if (rem >= 7'd20) rem = rem - 7'd20;
            if (rem >= 7'd10) rem = rem - 7'd10;
            F_mod10 = rem[3:0];
        end
    endfunction

    //------------------------------------------------------------------------
    // v * K_Q4: 按 K_Q4 的二进制位展开成移位相加(常数位在综合时折叠,
    // 不产生乘法器/DSP; 若综合器未折叠, 功能同样正确, 只是多几个加法器)
    //------------------------------------------------------------------------
    function [19:0] F_mul_k;
        input [12:0] v;
        integer i;
        reg [19:0] acc;
        begin
            acc = 20'd0;
            for (i = 0; i < 12; i = i + 1)
                if (((K_Q4 >> i) & 1) != 0)
                    acc = acc + ({7'd0, v} << i);
            F_mul_k = acc;
        end
    endfunction

    //------------------------------------------------------------------------
    // 框中心(2 倍形式) 与 相对原点的偏移
    //   cx2 = min+max+1 = 2 x 中心x          (0..2*W)
    //   dxh = cx2 - 2*(W/2) = 2 x (cx - 640) (-W..+W)
    //   yyh = 2*H - cy2      = 2 x (H - cy)  (0..2*H)
    // 换算: 0.01cm = dxh * K_Q4 / 32   (因为 dxh 是半像素单位, K_Q4 是 Q4)
    //------------------------------------------------------------------------
    wire [11:0] cx2 = {1'b0, I_minx} + {1'b0, I_maxx} + 12'd1;
    wire [11:0] cy2 = {1'b0, I_miny} + {1'b0, I_maxy} + 12'd1;

    wire signed [12:0] dxh = $signed({1'b0, cx2}) - HALF2;
    wire        [12:0] yyh = BOT2 - {1'b0, cy2};

    wire               xneg  = dxh[12];
    wire        [12:0] xmag  = xneg ? ((~dxh) + 13'd1) : dxh;
    wire        [19:0] xprod = F_mul_k(xmag);
    wire        [19:0] yprod = F_mul_k(yyh);

    // 四舍五入后右移 5 位(= /32); 12bit 饱和保护(正常视野下最大 20.00cm, 撞不到)
    wire [19:0] xsum  = xprod + 20'd16;
    wire [19:0] ysum  = yprod + 20'd16;
    wire [11:0] xval  = xsum[19:5] > 20'd4095 ? 12'd4095 : xsum[16:5];
    wire [11:0] yval  = ysum[19:5] > 20'd4095 ? 12'd4095 : ysum[16:5];

    // 十进制拆分: 整数部分(0..40) + 小数两位(0..99)
    wire [5:0]  x_int  = F_div100(xval);
    wire [6:0]  x_frac = F_mod100(xval);
    wire [5:0]  y_int  = F_div100(yval);
    wire [6:0]  y_frac = F_mod100(yval);

    wire [3:0]  x_tens = (x_int >= 6'd10) ? F_div10({1'b0, x_int}) : 4'hF;  // 前导零留白
    wire [3:0]  x_ones = F_mod10({1'b0, x_int});
    wire [3:0]  y_tens = (y_int >= 6'd10) ? F_div10({1'b0, y_int}) : 4'hF;
    wire [3:0]  y_ones = F_mod10({1'b0, y_int});

    wire [15:0] xdig = { x_tens, x_ones, F_div10(x_frac), F_mod10(x_frac) };
    wire [15:0] ydig = { y_tens, y_ones, F_div10(y_frac), F_mod10(y_frac) };

    //------------------------------------------------------------------------
    // 标签位置: 优先贴在框上方, 上方放不下(贴到画面顶)就改到框下方;
    //   左侧优先与框左边缘对齐, 右侧越界则左移贴到画面右边缘;
    //   x 再向下取整到 4 的倍数 —— 保证 4 像素/拍的一组恰好落在同一个字模点上,
    //   这样 coord_label 每拍只需查一次字模(省下 3/4 的字模 ROM)。
    //------------------------------------------------------------------------
    wire [11:0] ly_up = {1'b0, I_miny} - (LAB_H + LAB_GAP);
    wire [11:0] ly_dn = {1'b0, I_maxy} + LAB_GAP;
    wire        up_ok = (I_miny >= (LAB_H + LAB_GAP));
    wire [11:0] ly_r  = up_ok ? ly_up : ly_dn;
    wire [11:0] ly_c  = (ly_r > Y_LIM) ? Y_LIM : ly_r;
    wire [11:0] lx_c  = ({1'b0, I_minx} > X_LIM) ? X_LIM : {1'b0, I_minx};
    wire [11:0] lx_a  = {lx_c[11:2], 2'b00};

    //------------------------------------------------------------------------
    // 输出寄存(在该色 bbox 更新后一拍写入, 帧内恒定)
    //------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_lab_valid <= 1'b0;
            O_lab_x     <= 11'd0;
            O_lab_y     <= 10'd0;
            O_xneg      <= 1'b0;
            O_xdig      <= 16'hFFFF;
            O_ydig      <= 16'hFFFF;
        end else if (I_upd) begin
            O_lab_valid <= I_bvalid;
            O_lab_x     <= lx_a[10:0];
            O_lab_y     <= ly_c[9:0];
            O_xneg      <= xneg;
            O_xdig      <= xdig;
            O_ydig      <= ydig;
        end
    end

endmodule
