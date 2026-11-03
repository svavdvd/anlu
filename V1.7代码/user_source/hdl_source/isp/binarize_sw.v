//============================================================================
// binarize_sw.v  --  滑动窗口局部门限二值化（自适应阈值，96bit/4像素每拍/720p60）
//
// 算法（局部均值门限，无 DSP/除法器/乘法器）：
//   灰度  Y = (R + 2G + B) >> 2
//   对每个像素取 WxW=16x16 因果窗口（锚点=像素即窗口右下角，覆盖左上邻域），
//   门限 = 窗口均值 + 偏移C（正偏移，像素须显著亮于均值才判白，抑制均匀场噪声
//   与边缘渐变的等高线纹路）；因 16x16=256=2^8，用左移+比较替代除法：
//       bin = (pixel << 8) > (窗口和 + (C << 8))   ? 白 : 黑
//   即：pixel 比局部均值亮出至少 C 级 → 白；否则 → 黑。
//
// 流水结构（满吞吐，总延迟 L_TOTAL）：
//   Y(组合) -> 15 级 line_buffer_33(各 321 拍) -> 列对齐(每行延迟 15-k 组)
//   -> 纵向 16 行求和 cs(组合) -> [寄 c0..c3 + 锚点 a] -> 4 组移位 cX_d1..d4
//   -> 横向 16 列窗和 win(组合) -> [寄 w0..w3 + 锚点 a1] -> 比较 -> 输出寄存器
//   两处流水寄存器（cs 后、win 后）把 16 行加法树与 16 列加法树拆开，满足 112.5MHz 时序。
//
// 列对齐原因：line_buffer_33 每级固有 1 组列滞后，级联 15 级后第 k 行滞后 k 组，
//   故第 k 行延迟 (15-k) 组，使 16 行对齐到同一列组（原理同 edge_detect 的
//   cur_d4/l1_d3/l2_d2）。
//
// 横向窗口注意：16 列窗只对 px3（组右端）与组边界对齐；px0/1/2 的窗向左错
//   1/2/3 列，故需额外保留第 5 组（G-4）的逐列和，用「减本组右列、加左 4 组对应列」
//   的并行方式得到各列窗和。
//
// 边界处理（因果窗口只缺上/左）：
//   上 15 行：由 15 级行缓存 valid 门控自然变黑（O_tvalid 仍按整帧 720 行拉高，
//   使下游 DDR 帧不丢行，仅顶部 15 行黑）。
//   左 ~16 列：h_cnt < 4 组时强制黑（窗口横向越界 + 行首横向滑动和冲刷）。
//   右/下边界：因果窗口向左上取，天然完整，无需抑制。
//
// 控制信号：tuser/tlast/tvalid 用移位寄存器延迟 L_TOTAL，保持上游 awb 传来的
//   「tuser 领先数据 54 拍」关系不变（同 edge_detect 的 L_SR 手法）。
//============================================================================

module binarize_sw #(
    parameter IMG_WIDTH  = 1280,       // 图像宽度（像素）
    parameter IMG_HEIGHT = 720,        // 图像高度（行）
    parameter WIN        = 16,         // 窗口边长（须为 2 的幂且 4 的倍数）
    parameter LOG2_WIN2  = 8,          // log2(WIN*WIN)，16x16=256 -> 8
    parameter [7:0] THRESH_OFFSET = 8'd4  // 正偏移 C：pixel > 均值 + C 判白（压均匀场噪声/等高线，4~16 可调）
)(
    input  wire        I_clk,
    input  wire        I_rst_n,        // 异步复位，低有效
    input  wire [95:0] I_tdata,        // 4 像素 x RGB888（px0=[23:0]..px3=[95:72]）
    input  wire        I_tvalid,
    input  wire        I_tuser,        // 帧开始脉冲（SOF）
    input  wire        I_tlast,        // 行结束（EOL）
    output reg  [95:0] O_tdata,
    output reg         O_tvalid,
    output reg         O_tuser,
    output reg         O_tlast
);

    localparam IMG_WIDTH_4x = IMG_WIDTH >> 2;   // 每行 4 像素组数 = 320
    localparam WIN_4x       = WIN >> 2;         // 窗口横向组数 = 4
    localparam NBUF         = WIN - 1;          // 行缓存级数 = 15

    // 数据路径逐像素延迟 = ALIGN(15) + 流水(a0/a1 共2) + 输出寄存器(1) = 18 拍
    // （15 级行缓存只决定窗口历史何时凑齐 = bin_valid 的 15 行预热，不决定逐像素延迟）
    localparam L_TOTAL = NBUF + 3;   // 18
    localparam L_SR    = L_TOTAL - 1;  // 17

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
    // 15 级行缓存（复用 line_buffer_33 推断 BRAM 512x33），级联得到 15 行历史
    //------------------------------------------------------------------------
    wire [32:0] lb_out   [0:NBUF-1];
    wire        lb_valid [0:NBUF-1];

    line_buffer_33 #(
        .IMG_HEIGHT (IMG_HEIGHT),
        .IMG_WIDTH  (IMG_WIDTH )
    ) u_lb0 (
        .I_clk   (I_clk       ),
        .I_rst_n (I_rst_n     ),
        .I_tuser (I_tuser     ),
        .I_valid (I_tvalid    ),
        .I_data  ({1'b0, Y}   ),
        .O_valid (lb_valid[0] ),
        .O_data  (lb_out[0]   )
    );

    genvar b;
    generate
        for (b = 1; b < NBUF; b = b + 1) begin : LB
            line_buffer_33 #(
                .IMG_HEIGHT (IMG_HEIGHT ),
                .IMG_WIDTH  (IMG_WIDTH  )
            ) u_lb (
                .I_clk   (I_clk         ),
                .I_rst_n (I_rst_n       ),
                .I_tuser (I_tuser       ),
                .I_valid (lb_valid[b-1] ),
                .I_data  (lb_out[b-1]   ),
                .O_valid (lb_valid[b]   ),
                .O_data  (lb_out[b]     )
            );
        end
    endgenerate

    //------------------------------------------------------------------------
    // 16 行流：row[0]=当前行，row[k]=第 k 级行缓存输出（k 行前，k 组列滞后）
    //------------------------------------------------------------------------
    wire [31:0] row [0:NBUF];
    assign row[0] = Y;
    genvar r;
    generate
        for (r = 0; r < NBUF; r = r + 1) begin : ROW
            assign row[r+1] = lb_out[r][31:0];
        end
    endgenerate

    //------------------------------------------------------------------------
    // 列对齐：第 k 行延迟 (NBUF-k)=15-k 组，使 16 行对齐到同一列组 g-15
    //------------------------------------------------------------------------
    wire [31:0] aligned [0:NBUF];
    genvar k;
    generate
        for (k = 0; k < WIN; k = k + 1) begin : ALIGN
            reg [31:0] sr [0:NBUF];
            integer i;
            always @(posedge I_clk or negedge I_rst_n) begin
                if (!I_rst_n) begin
                    for (i = 0; i < WIN; i = i + 1) sr[i] <= 32'd0;
                end else begin
                    sr[0] <= row[k];
                    for (i = 0; i < NBUF; i = i + 1) sr[i+1] <= sr[i];
                end
            end
            assign aligned[k] = sr[NBUF - k];
        end
    endgenerate

    //------------------------------------------------------------------------
    // F_sum16：16 个 8bit 值平衡树求和 -> 12bit（无 DSP，LUT 加法树）
    //------------------------------------------------------------------------
    function [11:0] F_sum16;
        input [7:0] a0, a1, a2, a3, a4, a5, a6, a7,
                    a8, a9, a10, a11, a12, a13, a14, a15;
        integer p0, p1, p2, p3, p4, p5, p6, p7;
        integer q0, q1, q2, q3;
        integer r0, r1, t;
        begin
            p0 = {1'b0, a0}  + {1'b0, a1};
            p1 = {1'b0, a2}  + {1'b0, a3};
            p2 = {1'b0, a4}  + {1'b0, a5};
            p3 = {1'b0, a6}  + {1'b0, a7};
            p4 = {1'b0, a8}  + {1'b0, a9};
            p5 = {1'b0, a10} + {1'b0, a11};
            p6 = {1'b0, a12} + {1'b0, a13};
            p7 = {1'b0, a14} + {1'b0, a15};
            q0 = p0 + p1; q1 = p2 + p3; q2 = p4 + p5; q3 = p6 + p7;
            r0 = q0 + q1; r1 = q2 + q3;
            t  = r0 + r1;
            F_sum16 = t[11:0];
        end
    endfunction

    //------------------------------------------------------------------------
    // 纵向 16 行求和 -> 4 列 colsum（12bit，0..4080）
    //------------------------------------------------------------------------
    wire [11:0] cs0 = F_sum16(aligned[0][7:0],   aligned[1][7:0],   aligned[2][7:0],   aligned[3][7:0],
                              aligned[4][7:0],   aligned[5][7:0],   aligned[6][7:0],   aligned[7][7:0],
                              aligned[8][7:0],   aligned[9][7:0],   aligned[10][7:0],  aligned[11][7:0],
                              aligned[12][7:0],  aligned[13][7:0],  aligned[14][7:0],  aligned[15][7:0]);
    wire [11:0] cs1 = F_sum16(aligned[0][15:8],  aligned[1][15:8],  aligned[2][15:8],  aligned[3][15:8],
                              aligned[4][15:8],  aligned[5][15:8],  aligned[6][15:8],  aligned[7][15:8],
                              aligned[8][15:8],  aligned[9][15:8],  aligned[10][15:8], aligned[11][15:8],
                              aligned[12][15:8], aligned[13][15:8], aligned[14][15:8], aligned[15][15:8]);
    wire [11:0] cs2 = F_sum16(aligned[0][23:16], aligned[1][23:16], aligned[2][23:16], aligned[3][23:16],
                              aligned[4][23:16], aligned[5][23:16], aligned[6][23:16], aligned[7][23:16],
                              aligned[8][23:16], aligned[9][23:16], aligned[10][23:16],aligned[11][23:16],
                              aligned[12][23:16],aligned[13][23:16],aligned[14][23:16],aligned[15][23:16]);
    wire [11:0] cs3 = F_sum16(aligned[0][31:24], aligned[1][31:24], aligned[2][31:24], aligned[3][31:24],
                              aligned[4][31:24], aligned[5][31:24], aligned[6][31:24], aligned[7][31:24],
                              aligned[8][31:24], aligned[9][31:24], aligned[10][31:24],aligned[11][31:24],
                              aligned[12][31:24],aligned[13][31:24],aligned[14][31:24],aligned[15][31:24]);

    //------------------------------------------------------------------------
    // 流水寄存器 1：寄 colsum -> c0..c3；寄锚点当前行 -> a0（同组 G，时序一致）
    //------------------------------------------------------------------------
    reg [11:0] c0, c1, c2, c3;
    reg [31:0] a0;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            c0 <= 12'd0; c1 <= 12'd0; c2 <= 12'd0; c3 <= 12'd0;
            a0 <= 32'd0;
        end else begin
            c0 <= cs0; c1 <= cs1; c2 <= cs2; c3 <= cs3;
            a0 <= aligned[0];
        end
    end

    //------------------------------------------------------------------------
    // colsum 移位寄存器：cX_d1..d4 对应组 G-1..G-4
    //------------------------------------------------------------------------
    reg [11:0] c0_d1, c0_d2, c0_d3, c0_d4;
    reg [11:0] c1_d1, c1_d2, c1_d3, c1_d4;
    reg [11:0] c2_d1, c2_d2, c2_d3, c2_d4;
    reg [11:0] c3_d1, c3_d2, c3_d3, c3_d4;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            c0_d1 <= 12'd0; c0_d2 <= 12'd0; c0_d3 <= 12'd0; c0_d4 <= 12'd0;
            c1_d1 <= 12'd0; c1_d2 <= 12'd0; c1_d3 <= 12'd0; c1_d4 <= 12'd0;
            c2_d1 <= 12'd0; c2_d2 <= 12'd0; c2_d3 <= 12'd0; c2_d4 <= 12'd0;
            c3_d1 <= 12'd0; c3_d2 <= 12'd0; c3_d3 <= 12'd0; c3_d4 <= 12'd0;
        end else begin
            c0_d1 <= c0; c0_d2 <= c0_d1; c0_d3 <= c0_d2; c0_d4 <= c0_d3;
            c1_d1 <= c1; c1_d2 <= c1_d1; c1_d3 <= c1_d2; c1_d4 <= c1_d3;
            c2_d1 <= c2; c2_d2 <= c2_d1; c2_d3 <= c2_d2; c2_d4 <= c2_d3;
            c3_d1 <= c3; c3_d2 <= c3_d1; c3_d3 <= c3_d2; c3_d4 <= c3_d3;
        end
    end

    //------------------------------------------------------------------------
    // 横向 16 列窗和（16bit，0..65280），并行平衡树，从流水寄存器 c/cX_dX 出发
    //------------------------------------------------------------------------
    wire [13:0] gcs    = {2'b00,c0}    + {2'b00,c1}    + {2'b00,c2}    + {2'b00,c3};
    wire [13:0] gcs_d1 = {2'b00,c0_d1} + {2'b00,c1_d1} + {2'b00,c2_d1} + {2'b00,c3_d1};
    wire [13:0] gcs_d2 = {2'b00,c0_d2} + {2'b00,c1_d2} + {2'b00,c2_d2} + {2'b00,c3_d2};
    wire [13:0] gcs_d3 = {2'b00,c0_d3} + {2'b00,c1_d3} + {2'b00,c2_d3} + {2'b00,c3_d3};

    wire [12:0] c01    = {1'b0,c0} + {1'b0,c1};
    wire [12:0] c012   = {1'b0,c0} + {1'b0,c1} + {1'b0,c2};
    wire [12:0] c4_23  = {1'b0,c2_d4} + {1'b0,c3_d4};
    wire [12:0] c4_123 = {1'b0,c1_d4} + {1'b0,c2_d4} + {1'b0,c3_d4};

    wire [15:0] win3 = {2'b00,gcs} + {2'b00,gcs_d1} + {2'b00,gcs_d2} + {2'b00,gcs_d3};
    wire [15:0] win2 = {3'b000,c012} + {2'b00,gcs_d1} + {2'b00,gcs_d2} + {2'b00,gcs_d3} + {4'b0000,c3_d4};
    wire [15:0] win1 = {3'b000,c01}  + {2'b00,gcs_d1} + {2'b00,gcs_d2} + {2'b00,gcs_d3} + {3'b000,c4_23};
    wire [15:0] win0 = {4'b0000,c0}  + {2'b00,gcs_d1} + {2'b00,gcs_d2} + {2'b00,gcs_d3} + {3'b000,c4_123};

    //------------------------------------------------------------------------
    // 流水寄存器 2：寄窗和 -> w0..w3；寄锚点 -> a1（同组 G，与 w 对齐）
    //------------------------------------------------------------------------
    reg [15:0] w0, w1, w2, w3;
    reg [31:0] a1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            w0 <= 16'd0; w1 <= 16'd0; w2 <= 16'd0; w3 <= 16'd0;
            a1 <= 32'd0;
        end else begin
            w0 <= win0; w1 <= win1; w2 <= win2; w3 <= win3;
            a1 <= a0;
        end
    end

    //------------------------------------------------------------------------
    // 比较：bin = (pixel << 8) > (win + (C << 8))，即 pixel > 局部均值 + C
    //   C = THRESH_OFFSET 为正偏移。左边 pixel<<8 最大 65280(16bit)、
    //   右边 win + C<<8 最大 130560(17bit)，均扩到 17bit 再比。
    //------------------------------------------------------------------------
    wire [7:0] ctr0 = a1[7:0];
    wire [7:0] ctr1 = a1[15:8];
    wire [7:0] ctr2 = a1[23:16];
    wire [7:0] ctr3 = a1[31:24];

    wire [16:0] w0_thr = {1'b0, w0} + {1'b0, THRESH_OFFSET, {LOG2_WIN2{1'b0}}};
    wire [16:0] w1_thr = {1'b0, w1} + {1'b0, THRESH_OFFSET, {LOG2_WIN2{1'b0}}};
    wire [16:0] w2_thr = {1'b0, w2} + {1'b0, THRESH_OFFSET, {LOG2_WIN2{1'b0}}};
    wire [16:0] w3_thr = {1'b0, w3} + {1'b0, THRESH_OFFSET, {LOG2_WIN2{1'b0}}};

    wire bin0 = ({1'b0, ctr0, {LOG2_WIN2{1'b0}}} > w0_thr);
    wire bin1 = ({1'b0, ctr1, {LOG2_WIN2{1'b0}}} > w1_thr);
    wire bin2 = ({1'b0, ctr2, {LOG2_WIN2{1'b0}}} > w2_thr);
    wire bin3 = ({1'b0, ctr3, {LOG2_WIN2{1'b0}}} > w3_thr);

    wire [95:0] bin_data = { bin3 ? 24'hFFFFFF : 24'h000000,
                             bin2 ? 24'hFFFFFF : 24'h000000,
                             bin1 ? 24'hFFFFFF : 24'h000000,
                             bin0 ? 24'hFFFFFF : 24'h000000 };

    //------------------------------------------------------------------------
    // 数据有效：末级行缓存 valid 延迟 2 拍（经 c/a0、w/a1 两级流水，与 bin_data 对齐）
    //------------------------------------------------------------------------
    reg [1:0] dvalid_sr;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            dvalid_sr <= 2'd0;
        else
            dvalid_sr <= {dvalid_sr[0], lb_valid[NBUF-1]};
    end
    wire bin_valid = dvalid_sr[1];

    //------------------------------------------------------------------------
    // 列计数（用于左边界抑制），帧首/复位清零，数据有效时递增、行尾回绕
    //------------------------------------------------------------------------
    reg [9:0] h_cnt;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n || I_tuser)
            h_cnt <= 10'd0;
        else if (bin_valid)
            h_cnt <= (h_cnt == IMG_WIDTH_4x - 1) ? 10'd0 : h_cnt + 10'd1;
    end

    // 左边界：前 WIN_4x 组（16 列）输出黑
    wire [95:0] bin_masked = (h_cnt < WIN_4x) ? 96'd0 : bin_data;

    //------------------------------------------------------------------------
    // 控制信号移位寄存器：tuser/tlast/tvalid 延迟 L_SR 拍（输出寄存器再补 1 拍）
    //------------------------------------------------------------------------
    reg [L_SR-1:0] tuser_sr;
    reg [L_SR-1:0] tlast_sr;
    reg [L_SR-1:0] tvalid_sr;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            tuser_sr  <= {L_SR{1'b0}};
            tlast_sr  <= {L_SR{1'b0}};
            tvalid_sr <= {L_SR{1'b0}};
        end else begin
            tuser_sr  <= {tuser_sr[L_SR-2:0],  I_tuser};
            tlast_sr  <= {tlast_sr[L_SR-2:0],  I_tlast};
            tvalid_sr <= {tvalid_sr[L_SR-2:0], I_tvalid};
        end
    end

    //------------------------------------------------------------------------
    // 输出寄存器：数据有效时输出（首 15 行 bin_valid 为低 -> 输出黑），
    // tvalid/tuser/tlast 延迟 L_TOTAL 拍，保持 54 拍领先关系。
    //------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tdata  <= 96'd0;
            O_tvalid <= 1'b0;
            O_tuser  <= 1'b0;
            O_tlast  <= 1'b0;
        end else begin
            O_tdata  <= bin_valid ? bin_masked : 96'd0;
            O_tvalid <= tvalid_sr[L_SR-1];
            O_tuser  <= tuser_sr[L_SR-1];
            O_tlast  <= tlast_sr[L_SR-1];
        end
    end

endmodule
