//============================================================================
// line_buffer_33.v  --  33bit 行缓存（推断 BRAM 512x33，深度可容纳 320 组/行）
//
// 功能与逐拍时序完全复刻 line_buffer_demosaic（供 edge_detect 使用），仅把
// 存储体从 IP 生成的 blk_mem_gen_demosaic（2048x33，浪费）换成推断的
// 512x33 BRAM，供滑动窗口二值化等多行缓存场景节省 BRAM。
//
// 时序特性（与 line_buffer_demosaic 一致，勿改）：
//   - 每级延迟 1 行 + 1 组（321 拍 = 320 组 + 1 拍 tlast_lock 门控）
//   - 输出相对输入滞后 1 组（4 像素）列，级联 N 级后第 k 级滞后 k 组列，
//     需在调用方用 (N-k) 组移位寄存器对齐（见 edge_detect / binarize_sw）
//   - addra/addrb 复位于 I_tuser（帧首）
//
// BRAM 推断：reg [32:0] mem[0:511] + fehdl 属性强制 ERAM；写入数据同步
// 寄存（data_d0），读出为异步（assign O_data = mem[addrb]），与
// blk_mem_gen_demosaic 的 REGMODE_B=NOREG 等价。
//============================================================================

module line_buffer_33 #(
    parameter IMG_HEIGHT = 720,    // 图像高度（行）
    parameter IMG_WIDTH  = 1280    // 图像宽度（像素）
) (
    input  wire        I_clk,
    input  wire        I_rst_n,    // 异步复位，低有效
    input  wire        I_tuser,    // 帧开始脉冲（SOF）
    input  wire        I_valid,
    input  wire [32:0] I_data,
    output wire        O_valid,
    output wire [32:0] O_data
);

    localparam IMG_WIDTH_4x = (IMG_WIDTH >> 2);   // 每行 4 像素组数 = 320

    reg  [8:0]  addra;      // 写地址（0..319，9bit 覆盖 512 深）
    reg  [8:0]  addrb;      // 读地址
    reg  [32:0] data_d0;    // 输入数据延迟 1 拍
    reg         valid_d0;   // 输入有效延迟 1 拍
    reg         tlast_lock_d0;

    // 推断 BRAM：512 x 33（需 >= IMG_WIDTH_4x）
    reg [32:0] mem [0:511];  /* fehdl force_ram=1, ram_style="bram_fast" */

    //------------------------------------------------------------------------
    // 数据/有效延迟 1 拍
    //------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            data_d0  <= 33'd0;
            valid_d0 <= 1'b0;
        end else begin
            data_d0  <= I_data;
            valid_d0 <= I_valid;
        end
    end

    //------------------------------------------------------------------------
    // 写地址：valid_d0 时递增，帧首/复位清零
    //------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n || I_tuser) begin
            addra <= 9'd0;
        end else if (valid_d0) begin
            addra <= (addra == IMG_WIDTH_4x - 1) ? 9'd0 : addra + 9'd1;
        end
    end

    //------------------------------------------------------------------------
    // 行末锁定：首次 addra 达行尾后置 1（此后才允许读出，屏蔽首行垃圾）
    //------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n || I_tuser) begin
            tlast_lock_d0 <= 1'b0;
        end else if ((tlast_lock_d0 == 1'b0) & (addra == IMG_WIDTH_4x - 1)) begin
            tlast_lock_d0 <= 1'b1;
        end
    end

    wire rd_en = (I_valid && tlast_lock_d0) ? 1'b1 : 1'b0;

    //------------------------------------------------------------------------
    // 读地址：rd_en 时递增（滞后 addra 1 拍，产生固有 1 组列滞后）
    //------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n || I_tuser) begin
            addrb <= 9'd0;
        end else begin
            addrb <= rd_en ? ((addrb == IMG_WIDTH_4x - 1) ? 9'd0 : addrb + 9'd1) : addrb;
        end
    end

    assign O_valid = (valid_d0 && tlast_lock_d0) ? 1'b1 : 1'b0;

    //------------------------------------------------------------------------
    // BRAM 写入（同步，wea=valid_d0）与读出（异步，等价 NOREG）
    //------------------------------------------------------------------------
    always @(posedge I_clk) begin
        if (valid_d0)
            mem[addra] <= data_d0;
    end

    assign O_data = mem[addrb];

endmodule
