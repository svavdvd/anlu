module isp_top (
    input 			axi4s_video_aclk,
    input 			I_rst_n			,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    input [2:0] 	I_img_mode		,
    input [2:0] 	I_brightness	,
    input [2:0] 	I_contrast		,
    input 			O_tready		,
    output [127:0] 	O_tdata			,
    output 			O_tlast			,
    output 			O_tuser			,
    output 			O_tvalid		,
    output 			I_tready		,

    // 红色物体检测结果(mode 7), 透传 img_proc
    output 			O_red_bbox_valid,
    output [10:0] 	O_red_min_x		,
    output [9:0] 	O_red_min_y		,
    output [10:0] 	O_red_max_x		,
    output [9:0] 	O_red_max_y		,
    output [19:0] 	O_red_count		,
    output [31:0] 	O_red_sum_x		,
    output [31:0] 	O_red_sum_y		,
    output 			O_red_frame_done,

    // 蓝色物体检测结果(mode 7), 透传 img_proc
    output 			O_blue_bbox_valid,
    output [10:0] 	O_blue_min_x	,
    output [9:0] 	O_blue_min_y	,
    output [10:0] 	O_blue_max_x	,
    output [9:0] 	O_blue_max_y	,
    output [19:0] 	O_blue_count	,
    output [31:0] 	O_blue_sum_x	,
    output [31:0] 	O_blue_sum_y	,
    output 			O_blue_frame_done,

    // 绿色物体检测结果(mode 7), 透传 img_proc
    output 			O_green_bbox_valid,
    output [10:0] 	O_green_min_x	,
    output [9:0] 	O_green_min_y	,
    output [10:0] 	O_green_max_x	,
    output [9:0] 	O_green_max_y	,
    output [19:0] 	O_green_count	,
    output [31:0] 	O_green_sum_x	,
    output [31:0] 	O_green_sum_y	,
    output 			O_green_frame_done
);

wire         m_aixs_tvalid;  //synthesis keep
  wire [127:0] m_aixs_tdata;  //synthesis keep
  wire         m_aixs_tuser;  //synthesis keep
  wire         m_aixs_tlast;  //synthesis keep
  wire         m_aixs_tready;  //synthesis keep

  wire [ 95:0] m_aixs_tdata_96;  //synthesis keep

  wire [39:0] blc_O_tdata;  //synthesis keep
  wire [ 9:0] blc_O_tdest;  //synthesis keep
  wire        blc_O_tlast;  //synthesis keep
  wire        blc_O_tuser;  //synthesis keep
  wire        blc_O_tvalid; //synthesis keep
  wire        blc_O_tready; //synthesis keep


  wire         awb_O_tlast;  //synthesis keep
  wire         awb_O_tuser;  //synthesis keep
  wire [ 95:0] awb_O_tdata;  //synthesis keep
  wire         awb_O_tvalid;  //synthesis keep
  wire         awb_O_tready;  //synthesis keep

  wire         awb_O_tuser_128;  //synthesis keep
  wire         awb_O_tvalid_128;  //synthesis keep
  wire [127:0] awb_O_tdata_128;  //synthesis keep

  wire         img_O_tvalid;  //synthesis keep
  wire         img_O_tuser;   //synthesis keep
  wire         img_O_tlast;   //synthesis keep
  wire [ 95:0] img_O_tdata;   //synthesis keep

  assign awb_O_tready = O_tready;
  assign O_tdata  = awb_O_tdata_128;
  assign O_tlast  = awb_O_tlast    ;
  assign O_tuser  = awb_O_tuser_128    ;
  assign O_tvalid = awb_O_tvalid_128   ;

  //assign m_aixs_tready = O_tready;
  //assign O_tdata  = m_aixs_tdata;
  //assign O_tlast  = m_aixs_tlast    ;
  //assign O_tuser  = m_aixs_tuser    ;
  //assign O_tvalid = m_aixs_tvalid   ;


BLC #(
    .Black_level_offset_r0 (15),
    .Black_level_offset_r1 (15),
    .Black_level_offset_r2 (15),
    .Black_level_offset_r3 (15)
)
u_blc
(
    .I_clk    (axi4s_video_aclk),
    .I_rst_n  (I_rst_n),
    .I_tlast  (I_tlast),
    .I_tuser  (I_tuser),
    .I_tdata  (I_tdata),
    .I_tvalid (I_tvalid),
    .I_tdest  (I_tdest),
    .I_tready (I_tready),
    .O_tlast  (blc_O_tlast),
    .O_tuser  (blc_O_tuser),
    .O_tdata  (blc_O_tdata),
    .O_tvalid (blc_O_tvalid),
    .O_tdest  (blc_O_tdest),
    .O_tready (blc_O_tready)
);


demosaic #(
    .IMG_HEIGHT          (720 ), // 图像高度
    .IMG_WIDTH           (1280), // 图像宽度
    .data_complete_delay (50  ),
    .BAYER_MODE          ("BGGR")
)
u_demosaic
(
    .I_clk   			(axi4s_video_aclk)	,   // 时钟信号
    .I_rst_n 			(I_rst_n)	,   // 复位信号，低有效
    .axi4s_video_tdata 	(blc_O_tdata)	,  // AXI4-Stream视频数据
    .axi4s_video_tdest 	(blc_O_tdest)	,
    .axi4s_video_tlast 	(blc_O_tlast)	,  // 行结束信号
    .axi4s_video_tvalid	(blc_O_tvalid)	,  // 数据有效信号
    .axi4s_video_tuser 	(blc_O_tuser)	,  // 帧开始信号
    .axi4s_video_tready	(blc_O_tready)	,  // 从模块准备好接受数据
    .O_tlast  			(m_aixs_tlast)	,  // 输出行结束信号
    .O_tuser  			(m_aixs_tuser)	,  // 输出帧开始信号
    .O_tdata  			(m_aixs_tdata)	,  // 输出数据
    .O_tvalid 			(m_aixs_tvalid)	,  // 输出数据有效信号
    .O_tready   		(m_aixs_tready)	   // 输出数据准备好信号
);

data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(m_aixs_tdata_96)
);

    //========================================================================
    // 三色物体检测：单实例时分复用(红->蓝->绿逐帧轮转)。
    //
    // 数据源：三色均用 AWB 之后(awb_O_tdata)。绿最初用 AWB 之前(pre-AWB)，但
    //   pre-AWB 下白/反光高光天然偏绿(传感器对 G 更敏感)，会与绿物体混淆，
    //   故统一改为后 AWB：后 AWB 把白/高光拉回中性(R≈G≈B, S≈0)，只留下真绿。
    //   下面的 pre-AWB 延迟链(pre_data_delay/pre_aligned_tdata_96)保留但不再使用，
    //   便于需要时把绿回退到前 AWB。
    //   bbox/质心是帧锁存结果，三色均直通顶层对应颜色输出口。
    //========================================================================
    localparam AWB_PIPE = 55;   // awb.v: PIPE(53) + 2 级输出寄存器 = 55 拍

    // ---- pre-AWB 数据延迟 AWB_PIPE 拍，与 awb_O_tdata(显示流)逐像素对齐 ----
    reg  [95:0] pre_data_delay [0:AWB_PIPE-1];
    integer pre_delay_i;
    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            for (pre_delay_i = 0; pre_delay_i < AWB_PIPE; pre_delay_i = pre_delay_i + 1)
                pre_data_delay[pre_delay_i] <= 96'd0;
        end else begin
            pre_data_delay[0] <= m_aixs_tdata_96;
            for (pre_delay_i = 1; pre_delay_i < AWB_PIPE; pre_delay_i = pre_delay_i + 1)
                pre_data_delay[pre_delay_i] <= pre_data_delay[pre_delay_i-1];
        end
    end
    wire [95:0] pre_aligned_tdata_96 = pre_data_delay[AWB_PIPE-1];

    // ---- 三色 box_hit/掩码(供绘制) ----
    wire [3:0] det_red_box_hit;
    wire [3:0] det_red_mask;
    wire [3:0] det_blue_box_hit;
    wire [3:0] det_blue_mask;
    wire [3:0] det_green_box_hit;
    wire [3:0] det_green_mask;

    object_detect_3color #(
        .IMG_WIDTH  (1280),
        .IMG_HEIGHT (720 )
    ) u_object_detect (
        .I_clk    (axi4s_video_aclk),
        .I_rst_n  (I_rst_n         ),
        .I_tdata  (awb_O_tdata     ),
        .I_tdata_alt (pre_aligned_tdata_96),
        .I_tvalid (awb_O_tvalid    ),
        .I_tuser  (awb_O_tuser     ),
        .I_tlast  (awb_O_tlast     ),

        // 红/蓝结果直通顶层
        .O_red_bbox_valid (O_red_bbox_valid),
        .O_red_box_hit    (det_red_box_hit ),
        .O_red_mask       (det_red_mask    ),
        .O_red_min_x      (O_red_min_x     ),
        .O_red_min_y      (O_red_min_y     ),
        .O_red_max_x      (O_red_max_x     ),
        .O_red_max_y      (O_red_max_y     ),
        .O_red_count      (O_red_count     ),
        .O_red_sum_x      (O_red_sum_x     ),
        .O_red_sum_y      (O_red_sum_y     ),
        .O_red_frame_done (O_red_frame_done),

        .O_blue_bbox_valid (O_blue_bbox_valid),
        .O_blue_box_hit    (det_blue_box_hit ),
        .O_blue_mask       (det_blue_mask    ),
        .O_blue_min_x      (O_blue_min_x     ),
        .O_blue_min_y      (O_blue_min_y     ),
        .O_blue_max_x      (O_blue_max_x     ),
        .O_blue_max_y      (O_blue_max_y     ),
        .O_blue_count      (O_blue_count     ),
        .O_blue_sum_x      (O_blue_sum_x     ),
        .O_blue_sum_y      (O_blue_sum_y     ),
        .O_blue_frame_done (O_blue_frame_done),

        // 绿
        .O_green_bbox_valid (O_green_bbox_valid),
        .O_green_box_hit    (det_green_box_hit ),
        .O_green_mask       (det_green_mask    ),
        .O_green_min_x      (O_green_min_x     ),
        .O_green_min_y      (O_green_min_y     ),
        .O_green_max_x      (O_green_max_x     ),
        .O_green_max_y      (O_green_max_y     ),
        .O_green_count      (O_green_count     ),
        .O_green_sum_x      (O_green_sum_x     ),
        .O_green_sum_y      (O_green_sum_y     ),
        .O_green_frame_done (O_green_frame_done)
    );

    // 三色框/掩码均已对齐显示流(awb_O_tdata)，直通 img_proc
    wire [3:0] img_red_box_hit   = det_red_box_hit;
    wire [3:0] img_red_mask      = det_red_mask;
    wire [3:0] img_blue_box_hit  = det_blue_box_hit;
    wire [3:0] img_blue_mask     = det_blue_mask;
    wire [3:0] img_green_box_hit = det_green_box_hit;
    wire [3:0] img_green_mask    = det_green_mask;

    //========================================================================
    // 识别档(mode 7) 现实世界坐标：以显示屏(画面)底边中点为原点,
    //   x 轴向右为正, y 轴向上为正, 单位 cm, 显示 2 位小数(内部 1/100cm 定点)
    //   取值点 = 三色方块外接框中心(俯视 2D 平面, 忽略近大远小)
    //
    // 【标定 COORD_K_Q4】每像素对应多少个 0.01cm, Q4 定点(真实值 = 值 / 16)
    //     COORD_K_Q4 = 视野宽度(cm) x 100 x 16 / 1280 = 视野宽度(cm) x 1.25
    //     40cm -> 50(默认)    30cm -> 38    50cm -> 63    25cm -> 31
    //   标定方法：白纸上摆直尺, 量出画面左/右边缘之间的实际宽度, 代入上式重编译。
    //   注：坐标换算每帧只算一次(在该色 bbox 锁存后一拍), 不进像素关键路径。
    //========================================================================
    localparam integer COORD_K_Q4 = 50;

    // 三色 bbox 更新脉冲(检测器 frame_done 延迟 1 拍, 此时 min_x..max_y 已是新值)
    reg coord_upd_r, coord_upd_b, coord_upd_g;
    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            coord_upd_r <= 1'b0;
            coord_upd_b <= 1'b0;
            coord_upd_g <= 1'b0;
        end else begin
            coord_upd_r <= O_red_frame_done;
            coord_upd_b <= O_blue_frame_done;
            coord_upd_g <= O_green_frame_done;
        end
    end

    wire        lab_r_v, lab_b_v, lab_g_v;
    wire        lab_r_n, lab_b_n, lab_g_n;
    wire [10:0] lab_r_x, lab_b_x, lab_g_x;
    wire [9:0]  lab_r_y, lab_b_y, lab_g_y;
    wire [15:0] lab_r_xd, lab_b_xd, lab_g_xd;
    wire [15:0] lab_r_yd, lab_b_yd, lab_g_yd;

    coord_one #(
        .IMG_WIDTH  (1280       ),
        .IMG_HEIGHT (720        ),
        .K_Q4       (COORD_K_Q4 )
    ) u_coord_red (
        .I_clk     (axi4s_video_aclk),
        .I_rst_n   (I_rst_n         ),
        .I_upd     (coord_upd_r     ),
        .I_bvalid  (O_red_bbox_valid),
        .I_minx    (O_red_min_x     ),
        .I_miny    (O_red_min_y     ),
        .I_maxx    (O_red_max_x     ),
        .I_maxy    (O_red_max_y     ),
        .O_lab_valid(lab_r_v        ),
        .O_lab_x   (lab_r_x         ),
        .O_lab_y   (lab_r_y         ),
        .O_xneg    (lab_r_n         ),
        .O_xdig    (lab_r_xd        ),
        .O_ydig    (lab_r_yd        )
    );

    coord_one #(
        .IMG_WIDTH  (1280       ),
        .IMG_HEIGHT (720        ),
        .K_Q4       (COORD_K_Q4 )
    ) u_coord_blue (
        .I_clk     (axi4s_video_aclk ),
        .I_rst_n   (I_rst_n          ),
        .I_upd     (coord_upd_b      ),
        .I_bvalid  (O_blue_bbox_valid),
        .I_minx    (O_blue_min_x     ),
        .I_miny    (O_blue_min_y     ),
        .I_maxx    (O_blue_max_x     ),
        .I_maxy    (O_blue_max_y     ),
        .O_lab_valid(lab_b_v         ),
        .O_lab_x   (lab_b_x          ),
        .O_lab_y   (lab_b_y          ),
        .O_xneg    (lab_b_n          ),
        .O_xdig    (lab_b_xd         ),
        .O_ydig    (lab_b_yd         )
    );

    coord_one #(
        .IMG_WIDTH  (1280        ),
        .IMG_HEIGHT (720         ),
        .K_Q4       (COORD_K_Q4  )
    ) u_coord_green (
        .I_clk     (axi4s_video_aclk  ),
        .I_rst_n   (I_rst_n           ),
        .I_upd     (coord_upd_g       ),
        .I_bvalid  (O_green_bbox_valid),
        .I_minx    (O_green_min_x     ),
        .I_miny    (O_green_min_y     ),
        .I_maxx    (O_green_max_x     ),
        .I_maxy    (O_green_max_y     ),
        .O_lab_valid(lab_g_v          ),
        .O_lab_x   (lab_g_x           ),
        .O_lab_y   (lab_g_y           ),
        .O_xneg    (lab_g_n           ),
        .O_xdig    (lab_g_xd          ),
        .O_ydig    (lab_g_yd          )
    );

    // 打包: {valid, xneg, lab_x, lab_y, xdig, ydig} = 55bit
    wire [54:0] lab_red   = {lab_r_v, lab_r_n, lab_r_x, lab_r_y, lab_r_xd, lab_r_yd};
    wire [54:0] lab_blue  = {lab_b_v, lab_b_n, lab_b_x, lab_b_y, lab_b_xd, lab_b_yd};
    wire [54:0] lab_green = {lab_g_v, lab_g_n, lab_g_x, lab_g_y, lab_g_xd, lab_g_yd};

awb #(
    .IMG_HEIGHT(720 ),
    .IMG_WIDTH (1280)
) u_awb (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (m_aixs_tlast),
    .I_tuser (m_aixs_tuser),
    .I_tdata (m_aixs_tdata_96),
    .I_tvalid(m_aixs_tvalid),
    .I_tready(m_aixs_tready),
    .O_tlast (awb_O_tlast ),
    .O_tuser (awb_O_tuser ),
    .O_tdata (awb_O_tdata ),
    .O_tvalid(awb_O_tvalid),
    .O_tready(awb_O_tready)
);

img_proc u_img_proc (
    .I_clk    (axi4s_video_aclk),
    .I_rst_n  (I_rst_n         ),
    .I_mode   (I_img_mode      ),
    .I_brightness(I_brightness ),
    .I_contrast  (I_contrast   ),
    .I_tdata  (awb_O_tdata     ),
    .I_tvalid (awb_O_tvalid    ),
    .I_tuser  (awb_O_tuser     ),
    .I_tlast  (awb_O_tlast     ),
    .O_tdata  (img_O_tdata     ),
    .O_tvalid (img_O_tvalid    ),
    .O_tuser  (img_O_tuser     ),
    .O_tlast  (img_O_tlast     ),

    // 检测在 isp_top 完成：红/蓝/绿均吃 post-AWB 数据，三色框/掩码均已对齐 I_tdata
    .I_red_box_hit   (img_red_box_hit  ),
    .I_red_mask      (img_red_mask     ),
    .I_blue_box_hit  (img_blue_box_hit ),
    .I_blue_mask     (img_blue_mask    ),
    .I_green_box_hit (img_green_box_hit),
    .I_green_mask    (img_green_mask   ),

    // 三色物理坐标标签(mode 7 画在框旁边)
    .I_red_lab       (lab_red          ),
    .I_blue_lab      (lab_blue         ),
    .I_green_lab     (lab_green        )
);

    data_96bit_to_128bit u_data_96bit_to_128bit(
        .I_clk              ( axi4s_video_aclk  ),
        .I_rst_n            ( I_rst_n           ),

        .I_96b_frame_start  ( img_O_tuser	    ),
        .I_96b_valid        ( img_O_tvalid 		),
        .I_96b_data         ( img_O_tdata		),

        .O_128b_frame_start ( awb_O_tuser_128  	),
        .O_128b_valid       ( awb_O_tvalid_128 	),
        .O_128b_data        ( awb_O_tdata_128  	)
    );



//data96_128 u_data96_128 (
//    .I_tdata(awb_O_tdata),
//    .O_tdata(awb_O_tdata_128)
//);

endmodule
