//============================================================================
// coord_label.v -- 像素级坐标文字叠加(mode 7 识别档专用)
//
// 在标签矩形内按 3x5 点阵字模放大 4 倍画出两行文字:
//     X= 12.34        <- 第一行: x 坐标(cm)
//     Y=  5.67        <- 第二行: y 坐标(cm)
//   前导零留白, 负号占独立字符位, 保证两行等宽对齐。
//
// 【点阵尺寸】字模 3x5, 放大 4 倍 -> 单字 12x20 像素
//   字符间距 16px(右侧 4px 空白), 行距 24px(下方 4px 空白)
//   整块 8 字符 x 16 = 128 宽, 2 行 x 24 = 48 高
//
// 【为什么每拍只查一次字模】
//   标签左边界在 coord_one 里已对齐到 4 的倍数, 而本模块的输入像素坐标
//   I_px 恒为 4 的倍数(一拍 4 个像素的首地址), 因此 rel_x 也必为 4 的倍数:
//   一拍内的 4 个像素恰好落在同一个字模点上(字模放大正好是 4 倍),
//   于是"字符列/字模行/字符序号"对 4 个像素完全相同, 只需查 1 次,
//   输出 O_hit[3:0] 四位同值 —— 相比逐像素查表省下 3/4 的字模 ROM。
//
// 资源: 15 个 3x5 字模 = 15 x 15bit 常数表 + 少量比较器, 单实例约 60~80 LUT
//============================================================================
module coord_label #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720
)(
    input  wire        I_en,          // 识别档使能(mode==7)
    input  wire        I_lab_valid,   // 该色本帧有框
    input  wire [10:0] I_lab_x,       // 标签左上角(4 像素对齐)
    input  wire [9:0]  I_lab_y,
    input  wire        I_xneg,
    input  wire [15:0] I_xdig,        // {十位, 个位, 十分位, 百分位}
    input  wire [15:0] I_ydig,
    input  wire [10:0] I_px,          // 本拍 4 像素的首像素坐标
    input  wire [9:0]  I_py,
    output wire [3:0]  O_hit          // 4 像素文字命中掩码
);

    localparam integer PITCH  = 16;   // 字符间距
    localparam integer LINESZ = 24;   // 行距
    localparam integer NCOL   = 8;    // 每行字符数

    //------------------------------------------------------------------------
    // 相对标签左上角的位置
    //------------------------------------------------------------------------
    wire signed [12:0] rx = $signed({2'b00, I_px}) - $signed({2'b00, I_lab_x});
    wire signed [12:0] ry = $signed({3'b000, I_py}) - $signed({3'b000, I_lab_y});

    wire in_x = (rx >= 0) && (rx < (PITCH * NCOL));
    wire in_y = (ry >= 0) && (ry < (LINESZ * 2));

    wire        line = (ry >= LINESZ);
    wire [5:0]  ly   = line ? (ry[5:0] - 6'd24) : {1'b0, ry[4:0]};   // 行内偏移 0..23
    wire [2:0]  frow = ly[4:2];              // 字模行 0..5(5 为行间空白)
    wire [3:0]  cidx = rx[7:4];              // 字符序号 0..7
    wire [3:0]  loc  = rx[3:0];              // 字符内水平位置 0..15
    wire [1:0]  fcol = loc[3:2];             // 字模列 0..3(3 为字间空白)

    wire glyph_on = (frow < 3'd5) && (fcol < 2'd3);

    //------------------------------------------------------------------------
    // 字符选择
    //------------------------------------------------------------------------
    function [7:0] F_nib_ch;
        input [3:0] n;
        begin
            if (n == 4'hF) F_nib_ch = 8'h20;        // 空白
            else           F_nib_ch = {4'h3, n};    // '0'..'9'
        end
    endfunction

    localparam [7:0] CH_X   = 8'h58;
    localparam [7:0] CH_Y   = 8'h59;
    localparam [7:0] CH_EQ  = 8'h3D;
    localparam [7:0] CH_DOT = 8'h2E;
    localparam [7:0] CH_MIN = 8'h2D;
    localparam [7:0] CH_SP  = 8'h20;

    reg [7:0] ch;
    always @(*) begin
        case (cidx)
            4'd0:    ch = line ? CH_Y : CH_X;
            4'd1:    ch = CH_EQ;
            4'd2:    ch = ((line ? 1'b0 : I_xneg) ? CH_MIN : CH_SP);   // y 恒为正
            4'd3:    ch = F_nib_ch(line ? I_ydig[15:12] : I_xdig[15:12]);
            4'd4:    ch = F_nib_ch(line ? I_ydig[11:8]  : I_xdig[11:8]);
            4'd5:    ch = CH_DOT;
            4'd6:    ch = F_nib_ch(line ? I_ydig[7:4]   : I_xdig[7:4]);
            default: ch = F_nib_ch(line ? I_ydig[3:0]   : I_xdig[3:0]);
        endcase
    end

    //------------------------------------------------------------------------
    // 3x5 字模(每行 3bit, MSB 在左; 5 行共 15bit, 高位先)
    //------------------------------------------------------------------------
    reg [14:0] g;
    always @(*) begin
        case (ch)
            8'h30:   g = 15'b111_101_101_101_111;   // 0
            8'h31:   g = 15'b010_110_010_010_111;   // 1
            8'h32:   g = 15'b111_001_111_100_111;   // 2
            8'h33:   g = 15'b111_001_111_001_111;   // 3
            8'h34:   g = 15'b101_101_111_001_001;   // 4
            8'h35:   g = 15'b111_100_111_001_111;   // 5
            8'h36:   g = 15'b111_100_111_101_111;   // 6
            8'h37:   g = 15'b111_001_001_001_001;   // 7
            8'h38:   g = 15'b111_101_111_101_111;   // 8
            8'h39:   g = 15'b111_101_111_001_111;   // 9
            8'h2E:   g = 15'b000_000_000_000_100;   // .
            8'h2D:   g = 15'b000_000_111_000_000;   // -
            8'h3D:   g = 15'b000_111_000_111_000;   // =
            8'h58:   g = 15'b101_101_010_101_101;   // X
            8'h59:   g = 15'b101_101_010_010_010;   // Y
            default: g = 15'b000_000_000_000_000;   // 空格及未定义
        endcase
    end

    // 字模点取位: bit = 14 - 3*row - col  (row<=4, col<=2 -> 0..14)
    wire [2:0] fr  = (frow > 3'd4) ? 3'd4 : frow;
    wire [4:0] bidx = 5'd14 - ({2'b00, fr} + {2'b00, fr} + {2'b00, fr}) - {3'b000, fcol};
    wire       gbit = g[bidx[3:0]];

    assign O_hit = (I_en && I_lab_valid && in_x && in_y && glyph_on && gbit)
                 ? 4'b1111 : 4'b0000;

endmodule
