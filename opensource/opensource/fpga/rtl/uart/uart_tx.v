//============================================================================
// uart_tx.v -- 通用异步串口发送 (115200 / 8N1)
//
// 帧格式: 起始位(0) + 8 数据位(LSB 先发) + 停止位(1)，无校验位
// 波特率: 每 bit 占 BIT_CNT = CLK_FREQ/BAUD 个时钟周期。
//         100MHz 时钟下 115200 波特 -> BIT_CNT = 868 (实际 115207Hz, 误差 0.006%)
//
// 接口:
//   I_send : 1 拍高脉冲, 表示 I_din 有效, 启动一次发送
//   I_din  : 待发送字节 (8bit)
//   O_busy : 高电平表示正在发送 (busy 期间不应再发新字节, 新 send 会被忽略)
//   O_tx   : 串行输出, 空闲为高电平
//============================================================================
module uart_tx #(
    parameter CLK_FREQ = 100_000_000,   // 输入时钟频率 (Hz)
    parameter BAUD     = 115200         // 波特率 (bps)
)(
    input  wire       I_clk,
    input  wire       I_rst_n,          // 异步复位, 低有效
    input  wire [7:0] I_din,
    input  wire       I_send,
    output reg        O_tx,
    output reg        O_busy
);

    localparam [15:0] BIT_CNT = CLK_FREQ / BAUD;   // 每 bit 的时钟周期数 = 868

    reg [9:0]  tx_shift;      // 发送移位寄存器: [9]=停止位, [8:1]=数据, [0]=起始位
    reg [15:0] bit_cnt;       // 当前 bit 内的计数
    reg [3:0]  bit_idx;       // 已发送 bit 序号 0..9 (共 10 bit)

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tx     <= 1'b1;     // 空闲高电平
            O_busy   <= 1'b0;
            tx_shift <= 10'd0;
            bit_cnt  <= 16'd0;
            bit_idx  <= 4'd0;
        end else if (I_send && !O_busy) begin
            // 装载新字节并立即输出起始位; bit_cnt 从 1 起, 使每 bit 恰好 BIT_CNT 拍
            O_busy   <= 1'b1;
            O_tx     <= 1'b0;
            tx_shift <= {1'b1, I_din, 1'b0};   // 停止位=1, 数据, 起始位=0
            bit_cnt  <= 16'd1;
            bit_idx  <= 4'd0;
        end else if (O_busy) begin
            // 正在发送: 输出当前 bit, 每 BIT_CNT 拍右移一次
            O_tx <= tx_shift[0];
            if (bit_cnt == BIT_CNT - 1) begin
                bit_cnt <= 16'd0;
                if (bit_idx == 4'd9) begin
                    O_busy  <= 1'b0;        // 10 bit 全部发完
                    bit_idx <= 4'd0;
                end else begin
                    tx_shift <= tx_shift >> 1;
                    bit_idx  <= bit_idx + 4'd1;
                end
            end else begin
                bit_cnt <= bit_cnt + 16'd1;
            end
        end else begin
            O_tx <= 1'b1;   // 空闲保持高电平
        end
    end

endmodule
