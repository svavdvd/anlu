


module video_in (
    input wire         I_rst_n,

    input wire         I_camera_clk,
    input wire         I_camera_frame_start,
    input wire         I_camera_valid,
    input wire[127:0]  I_camera_data,
    input wire         I_mipi_rx_error,

    input wire         I_ddr_clk,

    input wire         I_display_pause,
    input wire         I_video_out_rd_busy,
    output wire        O_video_in_wr_busy,
    output reg[1:0]    O_video_out_rp,     ///读指针

    output reg         O_ddr_user_wr_en,
    output reg[24:0]   O_ddr_user_addr,
    output wire[127:0] O_ddr_user_wr_data,
    input wire         I_ddr_user_ready
);

    reg         S_fifo_rst;
    reg[3:0]    S_fifo_rst_cnt;
    reg         S_frame_tgl;        /// 相机域 toggle，每帧翻转一次
    reg         S_frame_start_1d;   /// 相机域 frame_start 上一拍（取上升沿）
    reg[2:0]    S_frame_tgl_s;      /// 3 级同步到 ddr_clk 域
    wire        S_frame_start;      /// toggle 变化沿 = 帧起始脉冲
    reg[1:0]    S_video_in_wp;       ///写指针
    wire[8:0]   S_fifo_rd_num;
    wire        S_fifo_rd_en;
    wire        S_fifo_rd_valid;
    wire[127:0] S_fifo_rd_data;
    reg         S_ddr_wr_valid;
    reg[9:0]    S_ddr_wr_cnt;

    localparam IMAGE_BASE_ADDR_0 = 25'd0;
    localparam IMAGE_BASE_ADDR_1 = 25'd7000000;
    localparam IMAGE_BASE_ADDR_2 = 25'd14000000;
    localparam IMAGE_BASE_ADDR_3 = 25'd21000000;


    // 以完整DDR写突发的占用标志仲裁，避免连续视频写入长期阻塞读通道。
    assign O_video_in_wr_busy = S_ddr_wr_valid;

    w128_d512_fifo U_w128_d512_fifo(
        .rst        ( S_fifo_rst         ),

        .clkw       ( I_camera_clk       ),
        .we         ( I_camera_valid     ),
        .di         ( I_camera_data      ),
        .afull      (),
        .full_flag  (),
        .wrusedw    (),

        .clkr       ( I_ddr_clk          ),
        .re         ( S_fifo_rd_en       ),
        .dout       ( O_ddr_user_wr_data ),
        .rdusedw    ( S_fifo_rd_num      ),
        .valid      (),
        .empty_flag (),
        .aempty     ()
    );


    // frame_start 跨时钟域：toggle 握手。
    // 相机域每帧只翻转一次 toggle（电平），ddr 域 3 级同步后异或取变化沿。
    // 电平每帧只变一次，采样即使撞亚稳态也最多晚一拍，不会多出/丢掉边沿。
    always @(posedge I_camera_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_frame_tgl      <= 1'b0;
            S_frame_start_1d <= 1'b0;
        end else begin
            S_frame_start_1d <= I_camera_frame_start;
            if(I_camera_frame_start && !S_frame_start_1d)
                S_frame_tgl <= ~S_frame_tgl;
        end
    end

    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_frame_tgl_s <= 3'd0;
        else
            S_frame_tgl_s <= {S_frame_tgl_s[1:0], S_frame_tgl};
    end

    assign S_frame_start = S_frame_tgl_s[2] ^ S_frame_tgl_s[1];

    // FIFO 复位：帧起始后拉高若干拍，在数据到达前完成复位
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_fifo_rst <= 1'b0;
        else if(S_frame_start)
            S_fifo_rst <= 1'b1;
        else if(S_fifo_rst_cnt == 'd10)
            S_fifo_rst <= 1'b0;
    end

    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_fifo_rst_cnt <= 'd0;
        else if(S_fifo_rst)
            S_fifo_rst_cnt <= S_fifo_rst_cnt + 'd1;
        else
            S_fifo_rst_cnt <= 'd0;
    end

    /*
        如果前一帧图像有误少行了，那么地址回退为之前的，覆盖有问题的那帧图像
    */
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_video_in_wp <= 'd0;
        else
            if(I_display_pause)
                S_video_in_wp <= S_video_in_wp;
            else if(S_frame_start)
                begin
                    if(I_mipi_rx_error)
                        S_video_in_wp <= S_video_in_wp;
                    else
                        S_video_in_wp <= S_video_in_wp + 'd1;
                end
            else
                S_video_in_wp <= S_video_in_wp;
    end


    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            O_video_out_rp <= 'd0;
        else
            if(I_display_pause)
                O_video_out_rp <= O_video_out_rp;
            else if(S_frame_start)
                begin
                    case(S_video_in_wp)
                        'd0: O_video_out_rp <= 'd2;
                        'd1: O_video_out_rp <= 'd3;
                        'd2: O_video_out_rp <= 'd0;
                        'd3: O_video_out_rp <= 'd1;
                    endcase
                end
            else
                O_video_out_rp <= O_video_out_rp;
    end

    /*
        fifo的深度是512，实际的深度是510，因此需要注意每次读出的长度避免为256，否则连续两次读写会引起fifo读空，丢失数据
    */
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_ddr_wr_valid <= 1'b0;
        else
            if(S_ddr_wr_valid && I_ddr_user_ready && S_ddr_wr_cnt == 'd239)
                S_ddr_wr_valid <= 1'b0;
            else if(S_fifo_rd_num >= 'd240 && (!I_video_out_rd_busy) && (!S_fifo_rst))
                S_ddr_wr_valid <= 1'b1;
            else
                S_ddr_wr_valid <= S_ddr_wr_valid;
    end

    always @(posedge I_ddr_clk) begin
        if(S_ddr_wr_valid)
            begin
                if(I_ddr_user_ready)
                    S_ddr_wr_cnt <= S_ddr_wr_cnt + 'd1;
                else
                    S_ddr_wr_cnt <= S_ddr_wr_cnt;
            end
        else
            S_ddr_wr_cnt <= 'd0;
    end

    assign S_fifo_rd_en = S_ddr_wr_valid & I_ddr_user_ready;

    always @(posedge I_ddr_clk) begin
        O_ddr_user_wr_en <= S_fifo_rd_en;
    end

    /*
        如果前一帧图像有误少行了，那么地址回退为之前的，覆盖有问题的那帧图像
    */
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            O_ddr_user_addr <= 1'b0;
        else
            if(S_frame_start && (!I_mipi_rx_error))
                begin
                    case(S_video_in_wp)
                        'd0: O_ddr_user_addr <= IMAGE_BASE_ADDR_0;
                        'd1: O_ddr_user_addr <= IMAGE_BASE_ADDR_1;
                        'd2: O_ddr_user_addr <= IMAGE_BASE_ADDR_2;
                        'd3: O_ddr_user_addr <= IMAGE_BASE_ADDR_3;
                    endcase
                end
            else if(S_frame_start && I_mipi_rx_error)
                begin
                    case(S_video_in_wp)
                        'd0: O_ddr_user_addr <= IMAGE_BASE_ADDR_3;
                        'd1: O_ddr_user_addr <= IMAGE_BASE_ADDR_0;
                        'd2: O_ddr_user_addr <= IMAGE_BASE_ADDR_1;
                        'd3: O_ddr_user_addr <= IMAGE_BASE_ADDR_2;
                    endcase
                end
            else if(O_ddr_user_wr_en)
                O_ddr_user_addr <= O_ddr_user_addr + 'd8;
            else
                O_ddr_user_addr <= O_ddr_user_addr;
    end


endmodule
