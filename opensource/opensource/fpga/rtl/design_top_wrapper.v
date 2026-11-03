

module design_top_wrapper (
    input wire        I_sys_clk,
    input wire        I_rst_n,
      
    output wire       O_cam_scl,
    inout  wire       IO_cam_sda,
    output wire       O_cam_24m,
    output wire       O_cam_rst,
      
    inout wire [1:0]  I_button,
    input wire        I_key0,
    input wire        I_key2,
    input wire        I_key3,
    input wire        I_key4,

    //---- STM32 串口 (115200/8N1, 3.3V LVCMOS) ----
    output wire       O_uart_tx,     // FPGA TX -> STM32 PA10 (RX)
    input  wire       I_uart_rx,     // FPGA RX <- STM32 PA9  (TX)

    output wire       O_screen_pwm,
    output wire       O_tmds_ch0_p,
    output wire       O_tmds_ch1_p,
    output wire       O_tmds_ch2_p,
    output wire       O_tmds_clk_p,

    inout wire        IO_rx_clk_pad_n, 
    inout wire        IO_rx_clk_pad_p, 
    inout wire[3:0]   IO_rx_data_pad_n,
    inout wire[3:0]   IO_rx_data_pad_p,

    output wire[12:0] ddr_addr,
    output wire[ 1:0] ddr_ba,
    output wire[ 0:0] ddr_cke,
    output wire[ 0:0] ddr_odt,
    output wire[ 0:0] ddr_cs_n,
    output wire       ddr_ras_n,
    output wire       ddr_cas_n,
    output wire       ddr_we_n,
    output wire[ 0:0] ddr_ck_p,
    output wire[ 0:0] ddr_ck_n,
    inout wire[1:0]   ddr_dm,
    inout wire[15:0]  ddr_dq,
    inout wire[1:0]   ddr_dqs_p,
    inout wire[1:0]   ddr_dqs_n   
);


    wire        	S_100m_clk;
    wire        	S_24m_clk;
    wire        	S_aux_50m_clk;
	wire        	S_10m_clk;
		
	wire        	S_pll_lock;
    wire        	S_rst_n;
	
	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire 			S_ae_req; 
	
	wire        	S_csi_rx_clk;  
    wire        	S_hs_rx_valid;  
    wire[15:0]  	S_hs_rx_data;   
	wire[1:0]   	S_lane_error;
	
	wire        	S_csi_frame_start;       
	wire        	S_csi_frame_end;         
	wire        	S_csi_valid;             
	wire[31:0]  	S_csi_data;  
		
	wire        	S_raw10_frame_start; 
	wire        	S_raw10_frame_end;   
	wire        	S_raw10_valid;       
	wire [39:0] 	S_raw10_data;
		
	wire 			S_axis_tlast;  
	wire 			S_axis_tuser;  
	wire [39:0]		S_axis_tdata;  
	wire 			S_axis_tvalid; 
	
	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;
	
	wire			S_ISP_O_tready;
	wire [127:0]	S_ISP_O_tdata ;
	wire 			S_ISP_O_tlast ;
	wire 			S_ISP_O_tuser ;
	wire 			S_ISP_O_tvalid; 
	

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;


    wire        S_vi_128b_frame_start;
    wire        S_vi_128b_valid;      
    wire[127:0] S_vi_128b_data;    
    
    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy; 
    wire[1:0]   S_video_out_rp;     

    wire        S_ddr_user_wr_en;       
    wire        S_ddr_user_rd_en;       
    wire[24:0]  S_ddr_user_addr;        
    wire[127:0] S_ddr_user_wr_data;     
    wire        S_ddr_user_ready;       
    wire        S_ddr_user_rd_valid;    
    wire[127:0] S_ddr_user_rd_data;     

    wire        S_vi_ddr_wr_en;   
    wire[24:0]  S_vi_ddr_wr_addr; 
    wire[127:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;   
    wire[24:0]  S_vo_ddr_rd_addr; 
    wire        S_vo_ddr_rd_ready;
    wire        S_vo_dbg_fifo_rst;
    wire        S_vo_dbg_ddr_rd_valid;
    wire        S_vo_dbg_fifo_rd_en;
    wire        S_vo_dbg_fifo_empty;

    wire        S_init_calib_complete; 
    wire[24:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[127:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[15:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[127:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       
    wire        S_mc_dbg_fifo_afull;
    wire        S_mc_dbg_app_rdy;
    wire        S_mc_dbg_wdf_rdy;
    wire        S_mc_dbg_cmd_pop;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;

    wire[23:0]  S_video_rd_data;
    reg         S_dbg_raw_seen;
    reg         S_dbg_isp_seen;
    reg         S_dbg_ddr_wr_seen;
    reg         S_dbg_ddr_rd_seen;
    reg         S_dbg_cam_cfg_seen;
    reg         S_dbg_hs_seen;
    reg         S_dbg_csi_seen;
    reg         S_dbg_raw10_seen;
    reg         S_dbg_isp_fs_seen;
    reg         S_dbg_isp_valid_seen;
    reg         S_dbg_ddr_req_seen;
    reg         S_dbg_ddr_ready_seen;
    reg [1:0]   S_dbg_raw_seen_sync;
    reg [1:0]   S_dbg_isp_seen_sync;
    reg [1:0]   S_dbg_ddr_wr_seen_sync;
    reg [1:0]   S_dbg_ddr_rd_seen_sync;
    reg [1:0]   S_dbg_cam_cfg_seen_sync;
    reg [1:0]   S_dbg_hs_seen_sync;
    reg [1:0]   S_dbg_csi_seen_sync;
    reg [1:0]   S_dbg_raw10_seen_sync;
    reg [1:0]   S_dbg_isp_fs_seen_sync;
    reg [1:0]   S_dbg_isp_valid_seen_sync;
    reg [1:0]   S_dbg_ddr_req_seen_sync;
    reg [1:0]   S_dbg_ddr_ready_seen_sync;
    reg         S_dbg_video_nonzero_seen;
    reg         S_dbg_rd_req_seen;
    reg         S_dbg_rd_valid_seen;
    reg         S_dbg_rd_nonzero_seen;
    reg [1:0]   S_dbg_rd_req_seen_sync;
    reg [1:0]   S_dbg_rd_valid_seen_sync;
    reg [1:0]   S_dbg_rd_nonzero_seen_sync;
    reg         S_dbg_mc_init_seen;
    reg         S_dbg_mc_app_rdy_seen;
    reg         S_dbg_mc_wdf_rdy_seen;
    reg         S_dbg_mc_cmd_pop_seen;
    reg [1:0]   S_dbg_mc_init_seen_sync;
    reg [1:0]   S_dbg_mc_app_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_wdf_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_cmd_pop_seen_sync;
    wire[3:0]   S_hdmi_debug_status;
    wire        S_lane_error_any;


    assign O_screen_pwm = 1'b1;
    assign S_video_out_vsync = ~S_hdmi_vsync;
	
	
    assign S_rst_n 		= S_pll_lock;
    assign S_hdmi_rst_n = S_pll_lock;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw_seen <= 1'b0;
        else if(S_raw10_frame_start | S_axis_tvalid)
            S_dbg_raw_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_seen <= 1'b0;
        else if(S_ISP_O_tuser | S_ISP_O_tvalid)
            S_dbg_isp_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_wr_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_wr_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_rd_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_ddr_rd_seen <= 1'b1;
    end

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_cam_cfg_seen <= 1'b0;
        else if(S_cam_cfg_done)
            S_dbg_cam_cfg_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_hs_seen <= 1'b0;
        else if(S_hs_rx_valid)
            S_dbg_hs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_csi_seen <= 1'b0;
        else if(S_csi_frame_start | S_csi_valid)
            S_dbg_csi_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw10_seen <= 1'b0;
        else if(S_raw10_frame_start | S_raw10_valid)
            S_dbg_raw10_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_fs_seen <= 1'b0;
        else if(S_ISP_O_tuser)
            S_dbg_isp_fs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_valid_seen <= 1'b0;
        else if(S_ISP_O_tvalid)
            S_dbg_isp_valid_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_req_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_ready_seen <= 1'b0;
        else if(S_ddr_user_ready)
            S_dbg_ddr_ready_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_vo_ddr_rd_en)
            S_dbg_rd_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_rd_valid_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
            S_dbg_rd_nonzero_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_init_seen <= 1'b0;
        else if(S_init_calib_complete)
            S_dbg_mc_init_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_app_rdy_seen <= 1'b0;
        else if(S_mc_dbg_app_rdy)
            S_dbg_mc_app_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_wdf_rdy_seen <= 1'b0;
        else if(S_mc_dbg_wdf_rdy)
            S_dbg_mc_wdf_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_cmd_pop_seen <= 1'b0;
        else if(S_mc_dbg_cmd_pop)
            S_dbg_mc_cmd_pop_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_dbg_cam_cfg_seen_sync <= 2'b00;
            S_dbg_hs_seen_sync      <= 2'b00;
            S_dbg_csi_seen_sync     <= 2'b00;
            S_dbg_raw10_seen_sync   <= 2'b00;
            S_dbg_raw_seen_sync    <= 2'b00;
            S_dbg_isp_seen_sync    <= 2'b00;
            S_dbg_ddr_wr_seen_sync <= 2'b00;
            S_dbg_ddr_rd_seen_sync <= 2'b00;
            S_dbg_isp_fs_seen_sync  <= 2'b00;
            S_dbg_isp_valid_seen_sync <= 2'b00;
            S_dbg_ddr_req_seen_sync <= 2'b00;
            S_dbg_ddr_ready_seen_sync <= 2'b00;
            S_dbg_video_nonzero_seen <= 1'b0;
            S_dbg_rd_req_seen_sync <= 2'b00;
            S_dbg_rd_valid_seen_sync <= 2'b00;
            S_dbg_rd_nonzero_seen_sync <= 2'b00;
            S_dbg_mc_init_seen_sync <= 2'b00;
            S_dbg_mc_app_rdy_seen_sync <= 2'b00;
            S_dbg_mc_wdf_rdy_seen_sync <= 2'b00;
            S_dbg_mc_cmd_pop_seen_sync <= 2'b00;
        end
        else begin
            S_dbg_cam_cfg_seen_sync <= {S_dbg_cam_cfg_seen_sync[0], S_dbg_cam_cfg_seen};
            S_dbg_hs_seen_sync      <= {S_dbg_hs_seen_sync[0],      S_dbg_hs_seen};
            S_dbg_csi_seen_sync     <= {S_dbg_csi_seen_sync[0],     S_dbg_csi_seen};
            S_dbg_raw10_seen_sync   <= {S_dbg_raw10_seen_sync[0],   S_dbg_raw10_seen};
            S_dbg_raw_seen_sync    <= {S_dbg_raw_seen_sync[0],    S_dbg_raw_seen};
            S_dbg_isp_seen_sync    <= {S_dbg_isp_seen_sync[0],    S_dbg_isp_seen};
            S_dbg_ddr_wr_seen_sync <= {S_dbg_ddr_wr_seen_sync[0], S_dbg_ddr_wr_seen};
            S_dbg_ddr_rd_seen_sync <= {S_dbg_ddr_rd_seen_sync[0], S_dbg_ddr_rd_seen};
            S_dbg_isp_fs_seen_sync  <= {S_dbg_isp_fs_seen_sync[0],  S_dbg_isp_fs_seen};
            S_dbg_isp_valid_seen_sync <= {S_dbg_isp_valid_seen_sync[0], S_dbg_isp_valid_seen};
            S_dbg_ddr_req_seen_sync <= {S_dbg_ddr_req_seen_sync[0], S_dbg_ddr_req_seen};
            S_dbg_ddr_ready_seen_sync <= {S_dbg_ddr_ready_seen_sync[0], S_dbg_ddr_ready_seen};
            S_dbg_rd_req_seen_sync <= {S_dbg_rd_req_seen_sync[0], S_dbg_rd_req_seen};
            S_dbg_rd_valid_seen_sync <= {S_dbg_rd_valid_seen_sync[0], S_dbg_rd_valid_seen};
            S_dbg_rd_nonzero_seen_sync <= {S_dbg_rd_nonzero_seen_sync[0], S_dbg_rd_nonzero_seen};
            S_dbg_mc_init_seen_sync <= {S_dbg_mc_init_seen_sync[0], S_dbg_mc_init_seen};
            S_dbg_mc_app_rdy_seen_sync <= {S_dbg_mc_app_rdy_seen_sync[0], S_dbg_mc_app_rdy_seen};
            S_dbg_mc_wdf_rdy_seen_sync <= {S_dbg_mc_wdf_rdy_seen_sync[0], S_dbg_mc_wdf_rdy_seen};
            S_dbg_mc_cmd_pop_seen_sync <= {S_dbg_mc_cmd_pop_seen_sync[0], S_dbg_mc_cmd_pop_seen};
            if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
                S_dbg_video_nonzero_seen <= 1'b1;
        end
    end

    assign S_lane_error_any = |S_lane_error;

    assign S_vo_dbg_fifo_rst = 1'b0;
    assign S_vo_dbg_ddr_rd_valid = 1'b0;
    assign S_vo_dbg_fifo_rd_en = 1'b0;
    assign S_vo_dbg_fifo_empty = 1'b0;
    assign S_mc_dbg_fifo_afull = 1'b0;
    assign S_mc_dbg_app_rdy = 1'b0;
    assign S_mc_dbg_wdf_rdy = 1'b0;
    assign S_mc_dbg_cmd_pop = 1'b0;

    assign S_hdmi_debug_status = {
        (|S_ddr_user_rd_data),
        S_ddr_user_rd_valid,
        S_vo_ddr_rd_en,
        S_vi_ddr_wr_en
    };
	
	assign O_cam_rst 	= 1'b1;
    assign O_cam_24m 	= S_24m_clk;
	
	
	
	
    PLL u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset    	 ( ~I_rst_n          ),
        
        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk4_out    ( S_hdmi_pixel_clk  ),
        .clk5_out    ( S_hdmi_serial_clk ),

        .lock        ( S_pll_lock        )

    );


	

  
  ae_set u_ae_set (
      .I_clk(S_24m_clk),
      .I_rst(~S_rst_n),
      .I_btn({I_button,2'b11}),
      .I_cam_cfg_done(S_cam_cfg_done),
      .I_ae_cfg_done(S_ae_cfg_done),
      .O_ae_req(S_ae_req),
      .O_ae(S_ae),
      .O_ag(S_ag)
  );

    //--------------------------------------------------------------
    // 图像处理档位切换
    // 板载按键 key0 接在 Bank42 的 D5，外部上拉到 VCC、按下接地，
    // 因此是低有效，送消抖模块前先取反，使其表现为高有效。
    // 跨时钟域只传递 1bit 翻转标志(toggle)，各时钟域各自维护一份
    // 档位计数器。这样可避免多位格雷码在 4->0 回绕时出现 2bit
    // 同时翻转而导致的亚稳态采样错误。
    //--------------------------------------------------------------
    wire S_key0_pulse;

    key_remove_shakes u_key0_debounce (
        .I_clk         ( S_24m_clk    ),
        .I_rst_n       ( S_rst_n      ),
        .I_key_in      ( ~I_key0      ),
        .O_key_trig_out( S_key0_pulse )
    );

    reg S_mode_toggle;
    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_mode_toggle <= 1'b0;
        else if(S_key0_pulse)
            S_mode_toggle <= ~S_mode_toggle;
    end

    //---- 档位分发到 ISP 时钟域(112.5MHz) ----
    reg S_tgl_isp_1d;
    reg S_tgl_isp_2d;
    reg S_tgl_isp_3d;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_tgl_isp_1d <= 1'b0;
            S_tgl_isp_2d <= 1'b0;
            S_tgl_isp_3d <= 1'b0;
        end
        else begin
            S_tgl_isp_1d <= S_mode_toggle;
            S_tgl_isp_2d <= S_tgl_isp_1d;
            S_tgl_isp_3d <= S_tgl_isp_2d;
        end
    end

    wire S_mode_inc_isp;
    assign S_mode_inc_isp = S_tgl_isp_2d ^ S_tgl_isp_3d;

    reg [2:0] S_img_mode_isp;
    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_img_mode_isp <= 3'd0;
        else if(S_mode_inc_isp)
            S_img_mode_isp  <= (S_img_mode_isp  >= 3'd7) ? 3'd0 : S_img_mode_isp  + 3'd1;
    end

    //---- 档位分发到 HDMI 像素时钟域(75MHz) ----
    reg S_tgl_hdmi_1d;
    reg S_tgl_hdmi_2d;
    reg S_tgl_hdmi_3d;

    always @(posedge S_hdmi_pixel_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_tgl_hdmi_1d <= 1'b0;
            S_tgl_hdmi_2d <= 1'b0;
            S_tgl_hdmi_3d <= 1'b0;
        end
        else begin
            S_tgl_hdmi_1d <= S_mode_toggle;
            S_tgl_hdmi_2d <= S_tgl_hdmi_1d;
            S_tgl_hdmi_3d <= S_tgl_hdmi_2d;
        end
    end

    wire S_mode_inc_hdmi;
    assign S_mode_inc_hdmi = S_tgl_hdmi_2d ^ S_tgl_hdmi_3d;

    reg [2:0] S_img_mode_hdmi;
    always @(posedge S_hdmi_pixel_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_img_mode_hdmi <= 3'd0;
        else if(S_mode_inc_hdmi)
            S_img_mode_hdmi <= (S_img_mode_hdmi >= 3'd7) ? 3'd0 : S_img_mode_hdmi + 3'd1;
    end

    //--------------------------------------------------------------
    // 亮度(key2, A9)/对比度(key3, B9)按键调节
    // 两键均为外部上拉、按下接地（低有效），送消抖前取反为高有效。
    // 按键脉冲在 24MHz 域转成 1bit toggle，跨时钟域到 ISP 域后各自
    // 维护 3bit 档位计数器（0~7 循环），再由 img_proc 帧边界锁存。
    //--------------------------------------------------------------
    wire S_key2_pulse;
    wire S_key3_pulse;

    key_remove_shakes u_key2_debounce (
        .I_clk         ( S_24m_clk    ),
        .I_rst_n       ( S_rst_n      ),
        .I_key_in      ( ~I_key2      ),
        .O_key_trig_out( S_key2_pulse )
    );

    key_remove_shakes u_key3_debounce (
        .I_clk         ( S_24m_clk    ),
        .I_rst_n       ( S_rst_n      ),
        .I_key_in      ( ~I_key3      ),
        .O_key_trig_out( S_key3_pulse )
    );

    reg S_bright_toggle;
    reg S_contrast_toggle;

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_bright_toggle   <= 1'b0;
            S_contrast_toggle <= 1'b0;
        end else begin
            if(S_key2_pulse)   S_bright_toggle   <= ~S_bright_toggle;
            if(S_key3_pulse)   S_contrast_toggle <= ~S_contrast_toggle;
        end
    end

    //---- 亮度/对比度 toggle 跨时钟域到 ISP 域 ----
    reg S_bright_tgl_isp_1d;
    reg S_bright_tgl_isp_2d;
    reg S_bright_tgl_isp_3d;
    reg S_contrast_tgl_isp_1d;
    reg S_contrast_tgl_isp_2d;
    reg S_contrast_tgl_isp_3d;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_bright_tgl_isp_1d   <= 1'b0;
            S_bright_tgl_isp_2d   <= 1'b0;
            S_bright_tgl_isp_3d   <= 1'b0;
            S_contrast_tgl_isp_1d <= 1'b0;
            S_contrast_tgl_isp_2d <= 1'b0;
            S_contrast_tgl_isp_3d <= 1'b0;
        end else begin
            S_bright_tgl_isp_1d   <= S_bright_toggle;
            S_bright_tgl_isp_2d   <= S_bright_tgl_isp_1d;
            S_bright_tgl_isp_3d   <= S_bright_tgl_isp_2d;
            S_contrast_tgl_isp_1d <= S_contrast_toggle;
            S_contrast_tgl_isp_2d <= S_contrast_tgl_isp_1d;
            S_contrast_tgl_isp_3d <= S_contrast_tgl_isp_2d;
        end
    end

    wire S_bright_inc_isp;
    wire S_contrast_inc_isp;
    assign S_bright_inc_isp   = S_bright_tgl_isp_2d   ^ S_bright_tgl_isp_3d;
    assign S_contrast_inc_isp = S_contrast_tgl_isp_2d ^ S_contrast_tgl_isp_3d;

    reg [2:0] S_bright_level;
    reg [2:0] S_contrast_level;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_bright_level   <= 3'd2;   // 复位默认亮度 1.25x（img_proc 档位表第 2 档）
            S_contrast_level <= 3'd1;   // 复位默认对比度 1.00x（档位表第 1 档）
        end else begin
            if(S_bright_inc_isp)
                S_bright_level   <= (S_bright_level   >= 3'd7) ? 3'd0 : S_bright_level   + 3'd1;
            if(S_contrast_inc_isp)
                S_contrast_level <= (S_contrast_level >= 3'd7) ? 3'd0 : S_contrast_level + 3'd1;
        end
    end

    uicfgcs500_720p #(
      .CLK_DIV(24000000 / 100000 - 1)
  ) u_uicfgcs500 (
      .I_clk(S_24m_clk),  //系统时钟输入
      .I_rst_n(S_rst_n),  //系统复位输入
      .I_ae_req(S_ae_req),
      .I_ae(S_ae),
      .I_ag(S_ag),
      .O_cam_scl(O_cam_scl),  //I2C总线，SCL时钟
      .IO_cam_sda(IO_cam_sda),  //I2C总线，SDA数据
      .O_cfg_done(S_cam_cfg_done),  //摄像头寄存器初始化完成
      .O_ae_cfg_done(S_ae_cfg_done)  //AE配置完成
  );
	
    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        .HS_EQUALIZER          ( "0dB"   ),//"0dB","1.5dB","3dB","4.5dB","6dB","7.5dB","9dB","10.5dB"
        .HS_VGA_GAIN           ( "8dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),
     
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
      
        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),
      
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
      
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
      
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );
	
	
 //  cwc cwc_inst
 //(
 //    .probe0(S_hs_rx_valid),
 //    .probe1(S_hs_rx_data),
 //    .probe2(S_lane_error),
 //    .clk(S_csi_rx_clk)
 //);

 //csi 解码为RAW数据
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        )
);


//解码为RAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data)
  );


//将数据转为stream流
uial2axis #(
.IMG_WIDTH(1280),
.IMG_HEIGHT(720),
.INPUT_DATA_WIDTH(40)
) 
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw10_data       ),
.I_data_valid(S_raw10_valid      ),
.I_data_start(S_raw10_frame_start),
.I_data_end  (S_raw10_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast )
);
	

// 原厂HD模式直接输出1280x720 RAW10，不再进行FPGA固定窗口裁剪。
assign S_raw_tdata  = S_axis_tdata;
assign S_raw_tlast  = S_axis_tlast;
assign S_raw_tvalid = S_axis_tvalid;
assign S_raw_tuser  = S_axis_tuser;


//cwc1 cwc1_Inst
//  (
//      .probe0(S_axis_tuser),
//      .probe1(S_axis_tvalid),
//      .probe2(S_axis_tlast),
//      .probe3(S_axis_tdata),
//      .probe4(S_ISP_O_tuser),
//      .probe5(S_ISP_O_tvalid),
//      .probe6(S_ISP_O_tlast),
//      .probe7(S_ISP_O_tdata),
//      .probe8 (S_raw_tdata ),
//      .probe9 (S_raw_tlast ),
//      .probe10(S_raw_tvalid),
//      .probe11(S_raw_tuser ),
//      .clk(S_csi_rx_clk)
//  );


    //--------------------------------------------------------------
    // 红/蓝/绿物体检测结果(mode 7)。当前仅"算好保留"用于调试/后续分拣,
    // 未接引脚; 需要对外输出时再加顶层端口。
    // 质心 = S_red_sum_x / S_red_count, S_red_sum_y / S_red_count (蓝/绿色同理)。
    //--------------------------------------------------------------
    wire        S_red_bbox_valid;  //synthesis keep
    wire [10:0] S_red_min_x;       //synthesis keep
    wire [9:0]  S_red_min_y;       //synthesis keep
    wire [10:0] S_red_max_x;       //synthesis keep
    wire [9:0]  S_red_max_y;       //synthesis keep
    wire [19:0] S_red_count;       //synthesis keep
    wire [31:0] S_red_sum_x;       //synthesis keep
    wire [31:0] S_red_sum_y;       //synthesis keep
    wire        S_red_frame_done;  //synthesis keep

    wire        S_blue_bbox_valid; //synthesis keep
    wire [10:0] S_blue_min_x;      //synthesis keep
    wire [9:0]  S_blue_min_y;      //synthesis keep
    wire [10:0] S_blue_max_x;      //synthesis keep
    wire [9:0]  S_blue_max_y;      //synthesis keep
    wire [19:0] S_blue_count;      //synthesis keep
    wire [31:0] S_blue_sum_x;      //synthesis keep
    wire [31:0] S_blue_sum_y;      //synthesis keep
    wire        S_blue_frame_done; //synthesis keep

    wire        S_green_bbox_valid; //synthesis keep
    wire [10:0] S_green_min_x;      //synthesis keep
    wire [9:0]  S_green_min_y;      //synthesis keep
    wire [10:0] S_green_max_x;      //synthesis keep
    wire [9:0]  S_green_max_y;      //synthesis keep
    wire [19:0] S_green_count;      //synthesis keep
    wire [31:0] S_green_sum_x;      //synthesis keep
    wire [31:0] S_green_sum_y;      //synthesis keep
    wire        S_green_frame_done; //synthesis keep

//ISP算法顶层模块
isp_top u_isp_top (
.axi4s_video_aclk(S_csi_rx_clk),
.I_rst_n         (S_rst_n),
.I_tlast         (S_raw_tlast	),
.I_tuser         (S_raw_tuser	),
.I_tdata         (S_raw_tdata	),
.I_tvalid        (S_raw_tvalid	),
.I_tdest         (),
.I_img_mode      (S_img_mode_isp),
.I_brightness    (S_bright_level),
.I_contrast      (S_contrast_level),
.O_tready        (S_ISP_O_tready),
.O_tdata         (S_ISP_O_tdata ),
.O_tlast         (S_ISP_O_tlast ),
.O_tuser         (S_ISP_O_tuser ),
.O_tvalid        (S_ISP_O_tvalid),
.I_tready        (),

.O_red_bbox_valid (S_red_bbox_valid),
.O_red_min_x      (S_red_min_x     ),
.O_red_min_y      (S_red_min_y     ),
.O_red_max_x      (S_red_max_x     ),
.O_red_max_y      (S_red_max_y     ),
.O_red_count      (S_red_count     ),
.O_red_sum_x      (S_red_sum_x     ),
.O_red_sum_y      (S_red_sum_y     ),
.O_red_frame_done (S_red_frame_done),

.O_blue_bbox_valid (S_blue_bbox_valid),
.O_blue_min_x      (S_blue_min_x     ),
.O_blue_min_y      (S_blue_min_y     ),
.O_blue_max_x      (S_blue_max_x     ),
.O_blue_max_y      (S_blue_max_y     ),
.O_blue_count      (S_blue_count     ),
.O_blue_sum_x      (S_blue_sum_x     ),
.O_blue_sum_y      (S_blue_sum_y     ),
.O_blue_frame_done (S_blue_frame_done),

.O_green_bbox_valid (S_green_bbox_valid),
.O_green_min_x      (S_green_min_x     ),
.O_green_min_y      (S_green_min_y     ),
.O_green_max_x      (S_green_max_x     ),
.O_green_max_y      (S_green_max_y     ),
.O_green_count      (S_green_count     ),
.O_green_sum_x      (S_green_sum_x     ),
.O_green_sum_y      (S_green_sum_y     ),
.O_green_frame_done (S_green_frame_done)
  );


    video_in u_video_in(
        .I_rst_n              (  S_rst_n             ),

        .I_camera_clk         ( S_csi_rx_clk          ),
        .I_camera_frame_start ( S_ISP_O_tuser  ),
        .I_camera_valid       ( S_ISP_O_tvalid ),
        .I_camera_data        ( S_ISP_O_tdata  ),
        .I_mipi_rx_error      ( 1'b0			      ),

        .I_ddr_clk            ( S_ddr_clk             ),
		.I_display_pause      ( 1'b0                  ),
        .I_video_out_rd_busy  ( S_video_out_rd_busy   ),
        .O_video_in_wr_busy   ( S_video_in_wr_busy    ),
        .O_video_out_rp       ( S_video_out_rp        ),

        .O_ddr_user_wr_en     ( S_vi_ddr_wr_en        ),
        .O_ddr_user_addr      ( S_vi_ddr_wr_addr      ),
        .O_ddr_user_wr_data   ( S_vi_ddr_wr_data      ),
        .I_ddr_user_ready     ( S_ddr_user_ready      )
    );



    assign S_ddr_user_wr_en    = S_vi_ddr_wr_en;

    assign S_ddr_user_rd_en    = S_vo_ddr_rd_en;

    assign S_ddr_user_addr     = S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                 S_vo_ddr_rd_en ? S_vo_ddr_rd_addr : 'd0;

    assign S_ddr_user_wr_data  = S_vi_ddr_wr_data;


    assign S_video_out_rst_n = S_hdmi_rst_n;

    video_out u_video_out(
        .I_rst_n             ( S_video_out_rst_n    ),
        .I_ddr_clk           ( S_ddr_clk            ),
 
        .O_video_out_rd_busy ( S_video_out_rd_busy  ),
        .I_video_in_wr_busy  ( S_video_in_wr_busy   ),
        .I_video_out_rp      ( S_video_out_rp       ),
 
        .O_ddr_user_rd_en    ( S_vo_ddr_rd_en       ),
        .O_ddr_user_addr     ( S_vo_ddr_rd_addr     ),
        .I_ddr_user_ready    ( S_ddr_user_ready     ),
        .I_ddr_user_rd_valid ( S_ddr_user_rd_valid  ),
        .I_ddr_user_rd_data  ( S_ddr_user_rd_data   ),

        .I_dsi_clk           ( S_hdmi_pixel_clk     ),
        .I_video_vsync       ( S_video_out_vsync    ),
        .I_video_rd_en       ( S_hdmi_window_rd_en  ),
        .O_vdieo_data        ( S_video_rd_data      )
    );



    mc_to_user_interface u_mc_to_user_interface(
        .I_clk                   ( S_ddr_clk               ),
        .I_rst_n                 ( S_rst_n                 ),

        .I_ddr_user_wr_en        ( S_ddr_user_wr_en        ),
        .I_ddr_user_rd_en        ( S_ddr_user_rd_en        ),
        .I_ddr_user_addr         ( S_ddr_user_addr         ),
        .I_ddr_user_wr_data      ( S_ddr_user_wr_data      ),
        .O_ddr_user_ready        ( S_ddr_user_ready        ),
        .O_ddr_user_rd_valid     ( S_ddr_user_rd_valid     ),
        .O_ddr_user_rd_data      ( S_ddr_user_rd_data      ),
            
        .O_mc_app_en             ( S_mc_app_en             ),
        .O_mc_app_addr           ( S_mc_app_addr           ),
        .O_mc_app_cmd            ( S_mc_app_cmd            ),
        .I_mc_app_rdy            ( S_mc_app_rdy            ),
        .O_mc_app_wdf_wren       ( S_mc_app_wdf_wren       ),
        .O_mc_app_wdf_data       ( S_mc_app_wdf_data       ),
        .O_mc_app_wdf_end        ( S_mc_app_wdf_end        ),
        .O_mc_app_wdf_mask       ( S_mc_app_wdf_mask       ),
        .I_mc_app_wdf_rdy        ( S_mc_app_wdf_rdy        ),
        .I_mc_app_rd_data        ( S_mc_app_rd_data        ),
        .I_mc_app_rd_data_end    ( S_mc_app_rd_data_end    ),
        .I_mc_app_rd_data_valid  ( S_mc_app_rd_data_valid  )
    );



    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper(
        .I_sys_clk              ( I_sys_clk              ),
        .I_sys_rst_n            ( S_rst_n                ),

        .O_ddr_clk              ( S_ddr_clk              ),
        .O_init_calib_complete  ( S_init_calib_complete  ),
        .I_mc_app_addr          ( S_mc_app_addr          ),
        .I_mc_app_cmd           ( S_mc_app_cmd           ),
        .I_mc_app_en            ( S_mc_app_en            ),
        .I_mc_app_wdf_data      ( S_mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( S_mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( S_mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( S_mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( S_mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( S_mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( S_mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( S_mc_app_wdf_rdy       ),

        .ddr_addr               ( ddr_addr               ),
        .ddr_ba                 ( ddr_ba                 ),
        .ddr_cke                ( ddr_cke                ),
        .ddr_odt                ( ddr_odt                ),
        .ddr_cs_n               ( ddr_cs_n               ),
        .ddr_ras_n              ( ddr_ras_n              ),
        .ddr_cas_n              ( ddr_cas_n              ),
        .ddr_we_n               ( ddr_we_n               ),
        .ddr_ck_p               ( ddr_ck_p               ),
        .ddr_ck_n               ( ddr_ck_n               ),
        .ddr_dm                 ( ddr_dm                 ),
        .ddr_dq                 ( ddr_dq                 ),
        .ddr_dqs_p              ( ddr_dqs_p              ),
        .ddr_dqs_n              ( ddr_dqs_n              )
    );



    uivtc #(
        .H_ActiveSize ( 1280 ),
        .H_FrameSize  ( 1650 ),
        .H_SyncStart  ( 1390 ),
        .H_SyncEnd    ( 1430 ),
        .V_ActiveSize ( 720  ),
        .V_FrameSize  ( 750  ),
        .V_SyncStart  ( 725  ),
        .V_SyncEnd    ( 730  )
    )u_hdmi_vtc(
        .I_vtc_rstn    ( S_hdmi_rst_n    ),
        .I_vtc_clk     ( S_hdmi_pixel_clk ),
        .O_vtc_vs      ( S_hdmi_vsync     ),
        .O_vtc_hs      ( S_hdmi_hsync     ),
        .O_vtc_de_valid( S_hdmi_de        ),
        .O_vtc_user    ( S_hdmi_user      ),
        .O_vtc_last    ( S_hdmi_last      )
    );

    //--------------------------------------------------------------
    // 实时帧率(fps) 测量
    //   相机域: 每帧(S_ISP_O_tuser 上升沿)翻一次 1bit toggle;
    //   显示域: fps_meter 用 74.25MHz 像素时钟做 1 秒统计窗口, 输出两位 BCD。
    //   跨域是 1bit 电平, 三级同步后取双边沿即为"每帧一个"事件,
    //   相机域(MIPI_RX_BYTE_CLK)与显示域(HDMI_PIXEL_CLK)在 SDC 中已是异步组,
    //   属结构性伪路径, 不需要额外约束。
    //--------------------------------------------------------------
    reg S_fps_ftgl;
    reg S_fps_tuser_d;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_fps_ftgl    <= 1'b0;
            S_fps_tuser_d <= 1'b0;
        end else begin
            S_fps_tuser_d <= S_ISP_O_tuser;
            if(S_ISP_O_tuser && !S_fps_tuser_d)
                S_fps_ftgl <= ~S_fps_ftgl;
        end
    end

    wire [3:0] S_fps_tens;
    wire [3:0] S_fps_ones;

    fps_meter #(
        .CLK_HZ     ( 74_250_000   )
    ) u_fps_meter (
        .I_clk          ( S_hdmi_pixel_clk ),
        .I_rst_n        ( S_hdmi_rst_n     ),
        .I_frame_toggle ( S_fps_ftgl       ),
        .O_tens         ( S_fps_tens       ),
        .O_ones         ( S_fps_ones       )
    );

    hdmi_mixer #(
        .H_OFFSET   ( 0    ),
        .V_OFFSET   ( 0    ),
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720  ),
        .DEBUG_MODE ( 0    )
    )u_hdmi_mixer(
        .I_clk           ( S_hdmi_pixel_clk   ),
        .I_rst_n         ( S_hdmi_rst_n       ),
        .I_video_vsync   ( S_hdmi_vsync       ),
        .I_video_hsync   ( S_hdmi_hsync       ),
        .I_video_de      ( S_hdmi_de          ),
        .I_video_user    ( S_hdmi_user        ),
        .I_video_last    ( S_hdmi_last        ),
        .I_debug_status  ( S_hdmi_debug_status),
        .I_img_mode      ( S_img_mode_hdmi    ),
        .I_fps_tens      ( S_fps_tens         ),
        .I_fps_ones      ( S_fps_ones         ),
        .O_video_rd_en   ( S_hdmi_window_rd_en),
        .I_video_rd_data ( S_video_rd_data    ),
        .O_hdmi_vsync    ( S_hdmi_out_vsync   ),
        .O_hdmi_hsync    ( S_hdmi_out_hsync   ),
        .O_hdmi_de       ( S_hdmi_out_de      ),
        .O_hdmi_data     ( S_hdmi_out_data    )
    );

    hdmi_tx u_hdmi_tx(
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        .I_rst              ( ~S_hdmi_rst_n     ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_hdmi_out_vsync  ),
        .I_video_in_de      ( S_hdmi_out_de     ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( S_hdmi_out_data   ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p      ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p      ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p      ),
        .O_tmds_clk_p       ( O_tmds_clk_p      )
    );


    //--------------------------------------------------------------
    // STM32 串口 (115200/8N1)
    //   与 STM32 USART1 配对: FPGA.O_uart_tx -> STM32.PA10(RX),
    //                          FPGA.I_uart_rx <- STM32.PA9(TX), 两板共地。
    //   时钟 S_100m_clk=100MHz, 波特率 115200, 误差约 0.006%。
    //   识别模式(mode 7)下, 由 coord_batch + uart_arm_sched 把三色坐标
    //   格式化 "px,py,T\r\n" 逐条发给机械臂, 每条等 "done" 应答。
    //--------------------------------------------------------------
    wire [7:0] S_uart_tx_data;
    wire       S_uart_tx_send;
    wire       S_uart_tx_busy;
    wire [7:0] S_uart_rx_data;
    wire       S_uart_rx_valid;

    uart_tx u_uart_tx (
        .I_clk   ( S_100m_clk      ),
        .I_rst_n ( S_rst_n         ),
        .I_din   ( S_uart_tx_data  ),
        .I_send  ( S_uart_tx_send  ),
        .O_tx    ( O_uart_tx       ),
        .O_busy  ( S_uart_tx_busy  )
    );

    uart_rx u_uart_rx (
        .I_clk   ( S_100m_clk      ),
        .I_rst_n ( S_rst_n         ),
        .I_rx    ( I_uart_rx       ),
        .O_dout  ( S_uart_rx_data  ),
        .O_valid ( S_uart_rx_valid )
    );

    //--------------------------------------------------------------
    // KEY4 (板载 C7, 外部上拉、按下接地): 触发一次"三色坐标批次"发送。
    //   24MHz 域消抖 -> 1bit toggle -> 三级同步到 100MHz 串口域 -> 边沿得到
    //   1 拍触发脉冲, 交给 uart_arm_sched 的 I_trig。消抖/跨域与 key0/key2/key3
    //   同一套路。
    //--------------------------------------------------------------
    wire S_key4_pulse;

    key_remove_shakes u_key4_debounce (
        .I_clk         ( S_24m_clk    ),
        .I_rst_n       ( S_rst_n      ),
        .I_key_in      ( ~I_key4      ),
        .O_key_trig_out( S_key4_pulse )
    );

    reg S_key4_toggle;
    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_key4_toggle <= 1'b0;
        else if(S_key4_pulse)
            S_key4_toggle <= ~S_key4_toggle;
    end

    reg S_key4_tgl_1d;
    reg S_key4_tgl_2d;
    reg S_key4_tgl_3d;
    always @(posedge S_100m_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_key4_tgl_1d <= 1'b0;
            S_key4_tgl_2d <= 1'b0;
            S_key4_tgl_3d <= 1'b0;
        end else begin
            S_key4_tgl_1d <= S_key4_toggle;
            S_key4_tgl_2d <= S_key4_tgl_1d;
            S_key4_tgl_3d <= S_key4_tgl_2d;
        end
    end

    wire S_key4_trig;
    assign S_key4_trig = S_key4_tgl_2d ^ S_key4_tgl_3d;

    //--------------------------------------------------------------
    // 识别模式(mode 7): 三色坐标 -> 机械臂串口
    //
    //   object_detect 是时分复用, 每帧只更新一色。coord_batch 在绿色 pass
    //   收尾时把红/蓝/绿三色坐标打成一组 90bit 快照, 翻 1bit tgl 跨到
    //   100MHz 域; uart_arm_sched 按 红->蓝->绿 依次发 "px,py,T\r\n",
    //   每条等机械臂回 "done" 再发下一条, 与机械臂"一次抓一个"的节奏对齐。
    //
    //   协议与机械臂固件 user/main.c 的 sscanf("%f,%f, %c") 一致:
    //     px/py = 框中心相对"图像底边中点"的像素坐标(x 右正, y 上正)
    //     T     = 'R' / 'B' / 'G'
    //   注意: STM32 侧 DROP_TABLE 需包含这三个 type 字符, 否则回 ERR。
    //--------------------------------------------------------------
    // bbox 更新脉冲: 检测器 frame_done 延迟 1 拍(此时 min_x..max_y 已是新值)
    reg S_arm_upd_r, S_arm_upd_b, S_arm_upd_g;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_arm_upd_r <= 1'b0;
            S_arm_upd_b <= 1'b0;
            S_arm_upd_g <= 1'b0;
        end else begin
            S_arm_upd_r <= S_red_frame_done;
            S_arm_upd_b <= S_blue_frame_done;
            S_arm_upd_g <= S_green_frame_done;
        end
    end

    wire [89:0] S_arm_batch;
    wire        S_arm_tgl;
    wire        S_arm_en = (S_img_mode_isp == 3'd7);   // 仅识别模式发送

    coord_batch u_coord_batch (
        .I_clk     ( S_csi_rx_clk      ),
        .I_rst_n   ( S_rst_n           ),
        .I_en      ( S_arm_en          ),

        .I_r_upd   ( S_arm_upd_r       ),
        .I_r_valid ( S_red_bbox_valid  ),
        .I_r_minx  ( S_red_min_x       ),
        .I_r_miny  ( S_red_min_y       ),
        .I_r_maxx  ( S_red_max_x       ),
        .I_r_maxy  ( S_red_max_y       ),

        .I_b_upd   ( S_arm_upd_b       ),
        .I_b_valid ( S_blue_bbox_valid ),
        .I_b_minx  ( S_blue_min_x      ),
        .I_b_miny  ( S_blue_min_y      ),
        .I_b_maxx  ( S_blue_max_x      ),
        .I_b_maxy  ( S_blue_max_y      ),

        .I_g_upd   ( S_arm_upd_g        ),
        .I_g_valid ( S_green_bbox_valid ),
        .I_g_minx  ( S_green_min_x      ),
        .I_g_miny  ( S_green_min_y      ),
        .I_g_maxx  ( S_green_max_x      ),
        .I_g_maxy  ( S_green_max_y      ),

        .O_batch   ( S_arm_batch        ),
        .O_tgl     ( S_arm_tgl          )
    );

    uart_arm_sched #(
        .CMD_TIMEOUT ( 33'd8_000_000_000 )   // 单条等应答 80s(机械臂一次取放约 30~40s)
    ) u_uart_arm_sched (
        .I_clk      ( S_100m_clk      ),
        .I_rst_n    ( S_rst_n         ),
        .I_batch    ( S_arm_batch     ),
        .I_tgl      ( S_arm_tgl       ),
        .I_trig     ( S_key4_trig     ),
        .O_tx_data  ( S_uart_tx_data  ),
        .O_tx_send  ( S_uart_tx_send  ),
        .I_tx_busy  ( S_uart_tx_busy  ),
        .I_rx_data  ( S_uart_rx_data  ),
        .I_rx_valid ( S_uart_rx_valid ),
        .O_done     (                 ),
        .O_err      (                 )
    );



endmodule


