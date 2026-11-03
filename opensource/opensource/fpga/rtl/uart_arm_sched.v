//============================================================================
// uart_arm_sched.v -- 三色坐标批次 -> 机械臂串口命令 调度器 (100MHz 域)
//
// 职责:
//   1) 用 1bit toggle 从相机域同步 coord_batch 的快照(三级同步 + 边沿检测);
//   2) 按 红 -> 蓝 -> 绿 的顺序, 把快照里 valid 的条目逐条交给 uart_cmd;
//   3) uart_cmd 每条命令发完并等到机械臂回 "done"(或超时/ERR) 才发下一条,
//      天然与机械臂"一次只抓一个"的执行节奏对齐, 不会堆积。
//
// KEY4 触发(替代自动重发):
//   识别侧每 3 帧(≈50ms)就产生一组新坐标, 而机械臂抓一次要数秒, 不能每条都排队。
//   这里保留"最新一组"快照(batch_l 持续刷新), 但只在 I_trig(板载 KEY4 消抖后
//   跨到本域的脉冲)置位 armed 后才起发一批; 起批时把 armed 清零(消费触发),
//   发完即回到 IDLE 等下一次 KEY4 —— 一次按键 = 一次分拣, 旧数据自动丢弃。
//
// 协议(与机械臂固件 user/main.c 的 sscanf("%f,%f, %c") 匹配):
//   "px,py,T\r\n"   px 为框中心相对底边中点的像素坐标, py 同理向上为正,
//   T = 'R'/'B'/'G' 区分颜色。每条命令等待 "done" 应答。
//============================================================================
module uart_arm_sched #(
    // 单条等应答超时 80s @100MHz。
    // 机械臂走的是 0.3°/20ms 的平滑插值(≈15°/s)，一次取放(7 次 Arm_MoveTo +
    // 夹爪开合)要 30~40s，超时必须远大于它，否则会在机械臂还在动的时候
    // 就判超时并接着发下一条，导致"done"错配、丢命令。
    parameter [32:0] CMD_TIMEOUT = 33'd8_000_000_000
)(
    input  wire        I_clk,        // 100MHz 串口域
    input  wire        I_rst_n,

    // 来自相机域的批次快照
    input  wire [89:0] I_batch,
    input  wire        I_tgl,
    input  wire        I_trig,      // KEY4 触发脉冲(100MHz 域), 一次只发一批

    // 到 uart_tx / uart_rx
    output wire [7:0]  O_tx_data,
    output wire        O_tx_send,
    input  wire        I_tx_busy,
    input  wire [7:0]  I_rx_data,
    input  wire        I_rx_valid,

    // 调试/指示: 最近一条的结果脉冲
    output reg         O_done,
    output reg         O_err
);

    //------------------------------------------------------------------------
    // 1) toggle 跨时钟域
    //------------------------------------------------------------------------
    reg [2:0] tgl_s;      // 三级同步
    reg       tgl_d;      // 上一拍(用于边沿检测)

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            tgl_s <= 3'd0;
            tgl_d <= 1'b0;
        end else begin
            tgl_s <= { tgl_s[1:0], I_tgl };
            tgl_d <= tgl_s[2];
        end
    end

    wire tgl_edge = tgl_s[2] ^ tgl_d;

    //------------------------------------------------------------------------
    // 2) 边沿后再等一拍采数据(此时源端数据已稳定, 避免采到跳变中的多 bit)
    //------------------------------------------------------------------------
    reg        tgl_edge_d;
    reg [89:0] batch_l;
    reg        new_batch;
    reg        batch_done;      // 一批发完
    reg        armed;           // KEY4 触发, 起批时消费, 本批发完不再自动重发

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            tgl_edge_d <= 1'b0;
            batch_l    <= 90'd0;
            new_batch  <= 1'b0;
        end else begin
            tgl_edge_d <= tgl_edge;
            if (tgl_edge_d) begin
                batch_l   <= I_batch;
                new_batch <= 1'b1;
            end else if (batch_done) begin
                new_batch <= 1'b0;
            end
        end
    end

    //------------------------------------------------------------------------
    // 3) 条目解包
    //------------------------------------------------------------------------
    wire [29:0] e0 = batch_l[29:0];     // 红
    wire [29:0] e1 = batch_l[59:30];    // 蓝
    wire [29:0] e2 = batch_l[89:60];    // 绿

    reg  [1:0]  idx;
    wire [29:0] e_sel = (idx == 2'd1) ? e1 : (idx == 2'd2) ? e2 : e0;

    wire               e_valid = e_sel[0];
    wire [7:0]         e_type  = e_sel[8:1];
    wire signed [10:0] e_px    = e_sel[19:9];
    wire [9:0]         e_py    = e_sel[29:20];

    wire signed [15:0] px_ext  = { {5{e_px[10]}}, e_px };   // 符号扩展到 16bit
    wire signed [15:0] py_ext  = { 6'd0, e_py };            // py 恒 >= 0

    //------------------------------------------------------------------------
    // 4) uart_cmd 实例
    //------------------------------------------------------------------------
    reg  signed [15:0] cmd_x;
    reg  signed [15:0] cmd_y;
    reg  [7:0]         cmd_type;
    reg                cmd_start;

    wire cmd_busy, cmd_done, cmd_err;

    uart_cmd #(
        .TIMEOUT ( CMD_TIMEOUT )
    ) u_uart_cmd (
        .I_clk      ( I_clk      ),
        .I_rst_n    ( I_rst_n    ),
        .I_x        ( cmd_x      ),
        .I_y        ( cmd_y      ),
        .I_type     ( cmd_type   ),
        .I_start    ( cmd_start  ),
        .O_busy     ( cmd_busy   ),
        .O_done     ( cmd_done   ),
        .O_err      ( cmd_err    ),
        .O_tx_data  ( O_tx_data  ),
        .O_tx_send  ( O_tx_send  ),
        .I_tx_busy  ( I_tx_busy  ),
        .I_rx_data  ( I_rx_data  ),
        .I_rx_valid ( I_rx_valid )
    );

    //------------------------------------------------------------------------
    // 5) 调度状态机
    //------------------------------------------------------------------------
    localparam S_IDLE  = 3'd0;
    localparam S_FIND  = 3'd1;
    localparam S_START = 3'd2;
    localparam S_WAITB = 3'd3;
    localparam S_WAITD = 3'd4;

    reg [2:0] st;
    reg [2:0] wcnt;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            st         <= S_IDLE;
            idx        <= 2'd0;
            wcnt       <= 3'd0;
            cmd_x      <= 16'sd0;
            cmd_y      <= 16'sd0;
            cmd_type   <= 8'd0;
            cmd_start  <= 1'b0;
            batch_done <= 1'b0;
            armed      <= 1'b0;
            O_done     <= 1'b0;
            O_err      <= 1'b0;
        end else begin
            cmd_start  <= 1'b0;
            batch_done <= 1'b0;
            O_done     <= 1'b0;
            O_err      <= 1'b0;

            // KEY4 触发一次只发一批: I_trig 置位 armed, S_IDLE 起批时清零(消费)。
            // 本批发完后 armed 已为 0, 不会再自动重发, 等下一次 KEY4。
            if (I_trig) armed <= 1'b1;

            case (st)
                S_IDLE: begin
                    if (new_batch && armed) begin
                        armed <= 1'b0;
                        idx   <= 2'd0;
                        st    <= S_FIND;
                    end
                end

                // 依次找本轮第一个 valid 的条目, 全部无效则结束本批
                S_FIND: begin
                    if (idx == 2'd3) begin
                        batch_done <= 1'b1;
                        st         <= S_IDLE;
                    end else if (e_valid) begin
                        cmd_x    <= px_ext;
                        cmd_y    <= py_ext;
                        cmd_type <= e_type;
                        st       <= S_START;
                    end else begin
                        idx <= idx + 2'd1;
                    end
                end

                // 拉 1 拍 start, 下一拍 uart_cmd 采样到
                S_START: begin
                    cmd_start <= 1'b1;
                    wcnt      <= 3'd0;
                    st        <= S_WAITB;
                end

                // 等 busy 起来(正常情况下 1~2 拍); 起不来说明通道异常, 跳过本条
                S_WAITB: begin
                    if (cmd_busy) begin
                        st <= S_WAITD;
                    end else if (wcnt >= 3'd5) begin
                        idx <= idx + 2'd1;
                        st  <= S_FIND;
                    end else begin
                        wcnt <= wcnt + 3'd1;
                    end
                end

                // 等本条发送 + 机械臂执行完毕(busy 落); 超时由 uart_cmd 内部兜底
                S_WAITD: begin
                    if (!cmd_busy) begin
                        if (cmd_done) O_done <= 1'b1;
                        if (cmd_err)  O_err  <= 1'b1;
                        idx <= idx + 2'd1;
                        st  <= S_FIND;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
