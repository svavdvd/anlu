//============================================================================
// uart_cmd.v -- 机械臂抓取命令发送 (115200/8N1)
//
// 功能: 把抓取点坐标 (x, y, 整数 mm) 和分类字符 (type) 格式化成一行 ASCII
//       "x,y,type\r\n" 通过 UART 发出, 然后等待 STM32 的 "done" 应答。
//
// 接口:
//   I_x, I_y : 抓取点坐标 (mm, 有符号整数)
//   I_type   : 分类字符 ('B'/'Y' ...), 决定放置方案
//   I_start  : 1 拍脉冲, 触发一次发送 (O_busy 期间忽略)
//   O_busy   : 高电平 = 正在发送/等待应答, 期间不要触发新命令
//   O_done   : 1 拍脉冲, 收到 STM32 "done" (成功)
//   O_err    : 1 拍脉冲, 收到 "ERR" 或超时 (失败)
//
// 命令格式与 STM32 端 sscanf("%f,%f, %c") 匹配:
//   例: "180,0,B\r\n"、"100,-150,Y\r\n"
//============================================================================
module uart_cmd #(
    parameter CLK_FREQ = 100_000_000,   // 输入时钟频率 (Hz)
    parameter TIMEOUT  = 33'd6_000_000_000   // 等应答超时 (拍), 60s @100MHz
)(
    input  wire             I_clk,
    input  wire             I_rst_n,
    input  wire signed [15:0] I_x,
    input  wire signed [15:0] I_y,
    input  wire [7:0]       I_type,
    input  wire             I_start,
    output reg              O_busy,
    output reg              O_done,
    output reg              O_err,

    output reg  [7:0]       O_tx_data,
    output reg              O_tx_send,
    input  wire             I_tx_busy,
    input  wire [7:0]       I_rx_data,
    input  wire             I_rx_valid
);

    //------------------------------------------------------------------------
    // 锁存命令
    //------------------------------------------------------------------------
    reg signed [15:0] x_l, y_l;
    reg [7:0] type_l;

    //------------------------------------------------------------------------
    // 二进制 -> BCD (double-dabble): 16bit 无符号 -> 5 位 BCD (20bit)
    //------------------------------------------------------------------------
    function [19:0] F_bin2bcd;
        input [15:0] bin;
        reg [35:0] acc;
        integer k;
        begin
            acc = {20'b0, bin};
            for (k = 0; k < 16; k = k + 1) begin
                if (acc[19:16] >= 4'd5) acc[19:16] = acc[19:16] + 4'd3;
                if (acc[23:20] >= 4'd5) acc[23:20] = acc[23:20] + 4'd3;
                if (acc[27:24] >= 4'd5) acc[27:24] = acc[27:24] + 4'd3;
                if (acc[31:28] >= 4'd5) acc[31:28] = acc[31:28] + 4'd3;
                if (acc[35:32] >= 4'd5) acc[35:32] = acc[35:32] + 4'd3;
                acc = acc << 1;
            end
            F_bin2bcd = acc[35:16];
        end
    endfunction

    // 绝对值 + BCD (负号单独处理)
    wire [15:0] x_mag = x_l[15] ? (~x_l[15:0] + 16'd1) : x_l[15:0];
    wire [15:0] y_mag = y_l[15] ? (~y_l[15:0] + 16'd1) : y_l[15:0];
    wire [19:0] x_bcd = F_bin2bcd(x_mag);
    wire [19:0] y_bcd = F_bin2bcd(y_mag);

    wire x_neg = x_l[15];
    wire y_neg = y_l[15];

    // 5 位十进制: xd4=万位 ... xd0=个位
    wire [3:0] xd4 = x_bcd[19:16], xd3 = x_bcd[15:12], xd2 = x_bcd[11:8],
               xd1 = x_bcd[7:4],   xd0 = x_bcd[3:0];
    wire [3:0] yd4 = y_bcd[19:16], yd3 = y_bcd[15:12], yd2 = y_bcd[11:8],
               yd1 = y_bcd[7:4],   yd0 = y_bcd[3:0];

    // 第一个非零位(最高有效位), 用于去掉前导 0
    wire [2:0] x_first = xd4 ? 4 : xd3 ? 3 : xd2 ? 2 : xd1 ? 1 : 0;
    wire [2:0] y_first = yd4 ? 4 : yd3 ? 3 : yd2 ? 2 : yd1 ? 1 : 0;

    function [3:0] F_xdig;
        input [2:0] i;
        begin
        case (i)
            3'd0: F_xdig = xd0;
            3'd1: F_xdig = xd1;
            3'd2: F_xdig = xd2;
            3'd3: F_xdig = xd3;
            default: F_xdig = xd4;
        endcase
        end
    endfunction
    function [3:0] F_ydig;
        input [2:0] i;
        begin
        case (i)
            3'd0: F_ydig = yd0;
            3'd1: F_ydig = yd1;
            3'd2: F_ydig = yd2;
            3'd3: F_ydig = yd3;
            default: F_ydig = yd4;
        endcase
        end
    endfunction

    //------------------------------------------------------------------------
    // 发送完成检测 (busy 下降沿) + 应答检测 ("done" / "ERR")
    //------------------------------------------------------------------------
    reg tx_busy_d1;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) tx_busy_d1 <= 1'b0;
        else          tx_busy_d1 <= I_tx_busy;
    end
    wire tx_done = tx_busy_d1 & ~I_tx_busy;

    // 最近 4 个接收字节; 发新命令时清零, 避免上次应答的残留
    wire rx_clr = (state == S_IDLE) && I_start;
    reg [31:0] rx_hist;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) rx_hist <= 32'd0;
        else if (rx_clr) rx_hist <= 32'd0;
        else if (I_rx_valid) rx_hist <= {rx_hist[23:0], I_rx_data};
    end
    wire got_done = (rx_hist == 32'h646F6E65);         // "done"
    wire got_err  = (rx_hist[23:0] == 24'h455252);     // "ERR"

    //------------------------------------------------------------------------
    // 发送状态机
    //------------------------------------------------------------------------
    localparam S_IDLE   = 4'd0;
    localparam S_XSIGN  = 4'd1;
    localparam S_XDIG   = 4'd2;
    localparam S_COMMA1 = 4'd3;
    localparam S_YSIGN  = 4'd4;
    localparam S_YDIG   = 4'd5;
    localparam S_COMMA2 = 4'd6;
    localparam S_TYPE   = 4'd7;
    localparam S_CR     = 4'd8;
    localparam S_LF     = 4'd9;
    localparam S_WAIT   = 4'd10;

    reg [3:0]  state;
    reg [2:0]  d_idx;         // 当前发送的数字位 (0=个位 .. 4=万位)
    reg        tx_wait;       // 正在等当前字节发完
    reg [32:0] timeout_cnt;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            state       <= S_IDLE;
            d_idx       <= 3'd0;
            tx_wait     <= 1'b0;
            O_busy      <= 1'b0;
            O_done      <= 1'b0;
            O_err       <= 1'b0;
            O_tx_data   <= 8'd0;
            O_tx_send   <= 1'b0;
            timeout_cnt <= 33'd0;
            x_l         <= 16'sd0;
            y_l         <= 16'sd0;
            type_l      <= 8'd0;
        end else begin
            O_tx_send <= 1'b0;
            O_done    <= 1'b0;
            O_err     <= 1'b0;

            // 当前字节发送完成 -> 放开字段逻辑
            if (tx_wait && tx_done)
                tx_wait <= 1'b0;

            if (!tx_wait) begin
                case (state)
                    S_IDLE: begin
                        if (I_start) begin
                            x_l    <= I_x;
                            y_l    <= I_y;
                            type_l <= I_type;
                            O_busy <= 1'b1;
                            state  <= S_XSIGN;
                        end
                    end

                    S_XSIGN: begin
                        if (x_neg) begin
                            O_tx_data <= 8'h2D;   // '-'
                            O_tx_send <= 1'b1;
                            tx_wait   <= 1'b1;
                        end
                        d_idx <= x_first;
                        state <= S_XDIG;
                    end

                    S_XDIG: begin
                        O_tx_data <= {4'b0, F_xdig(d_idx)} + 8'h30;   // 数字 -> ASCII
                        O_tx_send <= 1'b1;
                        tx_wait   <= 1'b1;
                        if (d_idx == 3'd0)
                            state <= S_COMMA1;
                        else
                            d_idx <= d_idx - 3'd1;
                    end

                    S_COMMA1: begin
                        O_tx_data <= 8'h2C;   // ','
                        O_tx_send <= 1'b1;
                        tx_wait   <= 1'b1;
                        state     <= S_YSIGN;
                    end

                    S_YSIGN: begin
                        if (y_neg) begin
                            O_tx_data <= 8'h2D;
                            O_tx_send <= 1'b1;
                            tx_wait   <= 1'b1;
                        end
                        d_idx <= y_first;
                        state <= S_YDIG;
                    end

                    S_YDIG: begin
                        O_tx_data <= {4'b0, F_ydig(d_idx)} + 8'h30;
                        O_tx_send <= 1'b1;
                        tx_wait   <= 1'b1;
                        if (d_idx == 3'd0)
                            state <= S_COMMA2;
                        else
                            d_idx <= d_idx - 3'd1;
                    end

                    S_COMMA2: begin
                        O_tx_data <= 8'h2C;
                        O_tx_send <= 1'b1;
                        tx_wait   <= 1'b1;
                        state     <= S_TYPE;
                    end

                    S_TYPE: begin
                        O_tx_data <= type_l;
                        O_tx_send <= 1'b1;
                        tx_wait   <= 1'b1;
                        state     <= S_CR;
                    end

                    S_CR: begin
                        O_tx_data <= 8'h0D;   // '\r'
                        O_tx_send <= 1'b1;
                        tx_wait   <= 1'b1;
                        state     <= S_LF;
                    end

                    S_LF: begin
                        O_tx_data   <= 8'h0A;   // '\n'
                        O_tx_send   <= 1'b1;
                        tx_wait     <= 1'b1;
                        timeout_cnt <= 33'd0;
                        state       <= S_WAIT;
                    end

                    S_WAIT: begin
                        timeout_cnt <= timeout_cnt + 33'd1;
                        if (got_done) begin
                            O_done <= 1'b1;
                            O_busy <= 1'b0;
                            state  <= S_IDLE;
                        end else if (got_err || (timeout_cnt == TIMEOUT)) begin
                            O_err  <= 1'b1;
                            O_busy <= 1'b0;
                            state  <= S_IDLE;
                        end
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
