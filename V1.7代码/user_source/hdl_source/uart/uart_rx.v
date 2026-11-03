//============================================================================
// uart_rx.v -- 通用异步串口接收 (115200 / 8N1)
//
// 帧格式: 起始位(0) + 8 数据位(LSB 先收) + 停止位(1)，无校验位
//
// 原理: 检测起始位下降沿 -> 延迟到起始位中点校验(抗毛刺) ->
//       之后每隔 1 bit 在中点采样 8 个数据位 -> 采样停止位 ->
//       输出一个字节并给出 1 拍 O_valid 脉冲。
//
// 接口:
//   I_rx    : 串行输入 (空闲高电平)
//   O_dout  : 接收到的字节
//   O_valid : 1 拍脉冲, 表示 O_dout 有效
//============================================================================
module uart_rx #(
    parameter CLK_FREQ = 100_000_000,   // 输入时钟频率 (Hz)
    parameter BAUD     = 115200         // 波特率 (bps)
)(
    input  wire       I_clk,
    input  wire       I_rst_n,          // 异步复位, 低有效
    input  wire       I_rx,
    output reg  [7:0] O_dout,
    output reg        O_valid
);

    localparam [15:0] BIT_CNT = CLK_FREQ / BAUD;   // 每 bit 时钟周期数 = 868
    localparam [15:0] HALF    = BIT_CNT / 2;       // 半 bit = 434

    // 输入同步 + 起始位下降沿检测
    reg rx_d1, rx_d2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            rx_d1 <= 1'b1;
            rx_d2 <= 1'b1;
        end else begin
            rx_d1 <= I_rx;
            rx_d2 <= rx_d1;
        end
    end
    wire rx_negedge = rx_d2 & ~rx_d1;   // 下降沿 = 起始位开始

    localparam S_IDLE  = 2'd0;
    localparam S_START = 2'd1;   // 等待到起始位中点
    localparam S_DATA  = 2'd2;   // 采样 8 个数据位
    localparam S_STOP  = 2'd3;   // 采样停止位并输出

    reg [1:0]  state;
    reg [15:0] cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  rx_shift;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            state    <= S_IDLE;
            cnt      <= 16'd0;
            bit_idx  <= 3'd0;
            rx_shift <= 8'd0;
            O_dout   <= 8'd0;
            O_valid  <= 1'b0;
        end else begin
            O_valid <= 1'b0;    // 默认单拍脉冲清零

            case (state)
                S_IDLE: begin
                    if (rx_negedge) begin
                        state <= S_START;
                        cnt   <= 16'd0;
                    end
                end

                S_START: begin
                    // 等到起始位中点, 校验仍为 0 (否则视为毛刺, 回到空闲)
                    if (cnt == HALF - 1) begin
                        cnt <= 16'd0;
                        if (I_rx == 1'b0) begin
                            state   <= S_DATA;
                            bit_idx <= 3'd0;
                        end else begin
                            state <= S_IDLE;
                        end
                    end else begin
                        cnt <= cnt + 16'd1;
                    end
                end

                S_DATA: begin
                    if (cnt == BIT_CNT - 1) begin
                        rx_shift <= {I_rx, rx_shift[7:1]};   // LSB 先收, 右移
                        cnt      <= 16'd0;
                        if (bit_idx == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                        end
                    end else begin
                        cnt <= cnt + 16'd1;
                    end
                end

                S_STOP: begin
                    if (cnt == BIT_CNT - 1) begin
                        O_dout  <= rx_shift;
                        O_valid <= 1'b1;
                        state   <= S_IDLE;
                    end else begin
                        cnt <= cnt + 16'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
