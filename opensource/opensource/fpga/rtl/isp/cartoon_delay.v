//============================================================================
// cartoon_delay.v  --  卡通化（mode 6）专用的颜色对齐延迟链
//
// 为什么需要它：
//   edge_detect 的 3x3 Sobel 输出相对输入晚 4 拍（cur_d1..cur_d3 三级对齐，
//   再加输出寄存器一级）。卡通化的色块是"即时"的，必须延迟相同拍数，
//   黑色描边才会正好压在色块边界上。
//
// 为什么是寄存器链而不是 BRAM：
//   延迟只有 4 拍，48bit x 4 = 192 个触发器，比实例化一个 ERAM 更省。
//   更重要的是单位一致：edge_detect 的数据路径是每个时钟打一拍，
//   而 line_buffer 那类环形缓冲是按"有效拍"推进的。本设计里 I_tvalid 的
//   占空比约 75%，两种单位混用会横向错开好几个像素。纯寄存器链与
//   edge_detect 完全同拍，不存在这个问题。
//
// 位序（din 与 dout 完全对称，改的时候两边必须一起改）：
//   [47:44] px3 R   [43:40] px3 G   [39:36] px3 B
//   [35:32] px2 R   [31:28] px2 G   [27:24] px2 B
//   [23:20] px1 R   [19:16] px1 G   [15:12] px1 B
//   [11: 8] px0 R   [ 7: 4] px0 G   [ 3: 0] px0 B
//   每通道 4bit（对应 img_proc 里 4bit/4096 色量化）
//
// 只存颜色，不存 tvalid/tuser/tlast。卡通化输出直接复用 edge_detect 的
// O_tvalid/O_tuser/O_tlast，这样 mode 6 与 mode 5 的帧结构完全一致，
// 切模式时画面不会上下跳。
//============================================================================
module cartoon_delay #(
    parameter integer WIDTH  = 48,     // 数据位宽（4bit x 3通道 x 4像素）
    parameter integer STAGES = 4       // 延迟拍数（与 edge_detect 数据延迟一致）
)(
    input  wire             I_clk,
    input  wire             I_rst_n,
    input  wire [WIDTH-1:0] I_din,
    output wire [WIDTH-1:0] O_dout
);

    // 移位链：sr[0] 是最新数据，sr[STAGES-1] 最老，直接作为输出
    reg [WIDTH-1:0] sr [0:STAGES-1];
    integer i;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            for (i = 0; i < STAGES; i = i + 1)
                sr[i] <= {WIDTH{1'b0}};
        end else begin
            sr[0] <= I_din;
            for (i = 1; i < STAGES; i = i + 1)
                sr[i] <= sr[i-1];
        end
    end

    // STAGES=4 时 sr[3] 恰好是 I_din 延迟 4 拍
    assign O_dout = sr[STAGES-1];

endmodule
