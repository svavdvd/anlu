//============================================================================
// red_object_detect.v -- 红色物体检测(HSV 阈值法) + 多连通域外接框 + 质心统计
//
// 数据格式: 96bit = 4 像素 x RGB888, px0=[23:0] .. px3=[95:72], 与 img_proc 一致。
//
// 红色判定(逐像素, HSV 空间): 把 RGB 转到 HSV 后按 H/S/V 三个阈值判红。
//   V (亮度)   = max(R,G,B)
//   S (饱和度) = (max-min)/max          —— 归一化, 与亮度无关
//   H (色相)   = 不显式算角度, 用"最大通道 + |G-B|/delta"等效判据
//
//   判红条件(全部满足):
//     [V] max >= RED_V_MIN                           亮度下限, 抗暗部噪声
//     [S] delta*RED_S_DEN >= max*RED_S_NUM           归一化饱和度下限, 拒白/灰/粉
//     [H] max==R (色相落在 300°~60° 扇区) 且
//         |G-B|*RED_HUE_DEN <= delta*RED_HUE_NUM     H 环绕 0°, 窗口 ±60°*NUM/DEN
//
// 多连通域检测(替代早期"全画面单一外接框"):
//   1) RLE 前端: 每拍 4 像素判红掩码 -> 提取「闭合横 run」[xs, xe](行内连续红段),
//      跨拍的 run 用 open_run/open_start 跟踪, 行尾强制闭合。
//   2) run 事件 FIFO: 每拍最多 2 个闭合 run(行尾额外 1 个), 下游逐拍消费。
//   3) blob 跟踪表(MAX_BLOBS 个): 一个 run 落在第 y 行, 若与某个「上一行(lasty
//      == y-1)且 x 有重叠」的活跃 blob 相连, 就并入它(多个相连 blob 一起合并);
//      否则新建 blob。blob 记 minx/maxx/miny/maxy/count/sumx/sumy。
//   4) 收尾(finalize): blob 连续 1 行没被触碰(lasty < 当前行-1)即闭合, 面积
//      >= RED_MIN_PIXELS 的写入结果表; 末行/消隐期兜底收尾所有残留。
//   5) 帧边界(tuser): 结果表锁存到 out_*(供下一帧画框), 同时清空结果表与跟踪表。
//
// 输出: O_box_hit = 命中「任意一个」框边界的掩码(多个框同时画, 绿框);
//       O_min_x..O_sum_y = 面积最大的那个框的框+质心(兼容旧单框接口, 供机械臂);
//       O_red_mask = 判红像素掩码(调试涂洋红)。
//
// 坐标: h_cnt(0..319)/v_cnt(0..719), 每拍 4 像素, x = h_cnt*4 + {0,1,2,3}。
//   计数器由 tvalid/tlast/tuser 驱动, 与 I_tdata 同拍。
//   框绘制有 1 帧延迟(结果表 tuser 锁存), 与旧版一致。
//
// 资源: 无 DSP、无 BRAM, 少量比较器 + 加法器 + MAX_BLOBS 组小寄存器。
//============================================================================

module red_object_detect #(
    parameter IMG_WIDTH         = 1280,     // 图像宽度(像素)
    parameter IMG_HEIGHT        = 720,      // 图像高度(行)
    parameter [7:0] RED_V_MIN       = 8'd40,  // V(亮度 max) 下限, 抗暗部噪声
    parameter [3:0] RED_S_NUM       = 4'd1,   // 归一化饱和度下限 S = delta/max >= NUM/DEN
    parameter [3:0] RED_S_DEN       = 4'd2,   // 1/2 -> S >= 0.5, 拒白/灰/粉
    parameter [3:0] RED_HUE_NUM     = 4'd1,   // hue 窗口: |G-B| <= delta * NUM/DEN
    parameter [3:0] RED_HUE_DEN     = 4'd3,   // 1/3 -> H 在 0° ± 20° 内(拒橙/黄)
    parameter [19:0] RED_MIN_PIXELS = 20'd100, // 单个连通域最小面积(像素), 小于不框
    parameter MAX_BLOBS = 8,                  // 最多同时跟踪的活跃连通域数
    parameter MAX_BOXES = 8                   // 最多输出的框数
)(
    input  wire        I_clk,
    input  wire        I_rst_n,          // 异步复位, 低有效
    input  wire [95:0] I_tdata,          // 4 像素 x RGB888
    input  wire        I_tvalid,
    input  wire        I_tuser,          // 帧开始(SOF)
    input  wire        I_tlast,          // 行结束(EOL)

    output reg         O_box_valid,      // 高: 上一帧存在 >=1 个有效红区
    output wire [3:0]  O_box_hit,        // 当前拍 4 像素落在「任一」框边界(供绘制)
    output wire [3:0]  O_red_mask,       // 当前拍 4 像素判红掩码(调试: 涂色)
    output reg  [10:0] O_min_x,          // 面积最大的红区外接框(含端点)
    output reg  [9:0]  O_min_y,
    output reg  [10:0] O_max_x,
    output reg  [9:0]  O_max_y,
    output reg  [19:0] O_count,          // 最大红区像素数
    output reg  [31:0] O_sum_x,          // 最大红区 x 坐标和
    output reg  [31:0] O_sum_y,          // 最大红区 y 坐标和
    output reg         O_frame_done      // 帧边界 1 拍脉冲, 结果更新
);

    localparam IMG_WIDTH_4X = IMG_WIDTH >> 2;   // 每行 4 像素组数 = 320
    localparam FIFO_DEPTH   = 32;               // run 事件 FIFO 深度(正常场景远用不满)

    //------------------------------------------------------------------------
    // 逐像素 RGB->HSV 判定(组合逻辑, 无除法 —— 除法折叠进乘法比较)
    //   V = max, delta = max-min;
    //   饱和度: delta*RED_S_DEN >= max*RED_S_NUM  (等价于 delta/max >= NUM/DEN)
    //   色相  : |G-B|*RED_HUE_DEN <= delta*RED_HUE_NUM (等价于 |H| <= 60°*NUM/DEN)
    //   max==R 限定色相落在红扇区(R 最大), 与 |G-B| 条件一起圈定红窗口。
    //------------------------------------------------------------------------
    function F_red;
        input [7:0] r;
        input [7:0] g;
        input [7:0] b;
        reg [7:0] mx;
        reg [7:0] mn;
        reg [7:0] delta;
        reg [8:0] gb_abs;
        reg        r_is_max;
        reg [15:0] sat_lhs, sat_rhs, hue_lhs, hue_rhs;
        begin
            mx      = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
            mn      = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
            delta   = mx - mn;                                  // max - min
            gb_abs  = (g >= b) ? ({1'b0, g} - {1'b0, b}) : ({1'b0, b} - {1'b0, g});
            r_is_max = (r >= g) && (r >= b);                    // H 落在红扇区(R 最大)
            sat_lhs = {8'd0, delta} * {12'd0, RED_S_DEN};
            sat_rhs = {8'd0, mx}    * {12'd0, RED_S_NUM};
            hue_lhs = {7'd0, gb_abs} * {12'd0, RED_HUE_DEN};
            hue_rhs = {8'd0, delta} * {12'd0, RED_HUE_NUM};
            F_red = (mx >= RED_V_MIN) &&                        // [V] 亮度下限
                    (sat_lhs >= sat_rhs) &&                     // [S] 归一化饱和度下限
                    r_is_max &&                                  // [H] 红扇区
                    (hue_lhs <= hue_rhs);                        // [H] 红窗口(|G-B| 小)
        end
    endfunction

    wire red0 = F_red(I_tdata[23:16], I_tdata[15:8],  I_tdata[7:0]);
    wire red1 = F_red(I_tdata[47:40], I_tdata[39:32], I_tdata[31:24]);
    wire red2 = F_red(I_tdata[71:64], I_tdata[63:56], I_tdata[55:48]);
    wire red3 = F_red(I_tdata[95:88], I_tdata[87:80], I_tdata[79:72]);

    assign O_red_mask = {red3, red2, red1, red0};   // 调试: 判红掩码透出, 供 img_proc 涂色

    //------------------------------------------------------------------------
    // 像素坐标计数(h_cnt 0..319, v_cnt 0..719), 与 I_tdata 同拍
    //------------------------------------------------------------------------
    reg [9:0] h_cnt;
    reg [9:0] v_cnt;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n || I_tuser) begin
            h_cnt <= 10'd0;
            v_cnt <= 10'd0;
        end else if (I_tvalid) begin
            h_cnt <= (h_cnt == IMG_WIDTH_4X - 1) ? 10'd0 : h_cnt + 10'd1;
            v_cnt <= (h_cnt == IMG_WIDTH_4X - 1)
                     ? ((v_cnt == IMG_HEIGHT - 1) ? 10'd0 : v_cnt + 10'd1)
                     : v_cnt;
        end
    end

    wire [10:0] x0 = {h_cnt, 2'b00};      // h_cnt*4
    wire [10:0] x1 = x0 + 11'd1;
    wire [10:0] x2 = x0 + 11'd2;
    wire [10:0] x3 = x0 + 11'd3;

    //========================================================================
    // 1. RLE 前端: 每拍 4 像素判红掩码 -> 闭合横 run [xs, xe]
    //========================================================================
    reg        open_run;      // 上一拍末尾(x3)为红且同行, run 延续到本拍
    reg [10:0] open_start;    // 该延续 run 的起始 x

    // 各候选「闭合 run」的起止:
    //   c_carry : 延续 run 在本拍第一个像素前闭合 -> [open_start, x0-1]
    //   c0..c2  : 本拍内起止的 run
    //   c3      : 行尾(tlast)红到 x3 的 run 强制闭合到 x3
    wire c_carry = open_run && !red0;
    wire c0 = red0 && !red1;
    wire c1 = red1 && !red2;
    wire c2 = red2 && !red3;
    wire c3 = red3 && I_tlast;

    wire [10:0] start0 = open_run ? open_start : x0;
    wire [10:0] start1 = red0 ? (open_run ? open_start : x0) : x1;
    wire [10:0] start2 = red1 ? (red0 ? (open_run ? open_start : x0) : x1) : x2;
    wire [10:0] start3 = red2 ? (red1 ? (red0 ? (open_run ? open_start : x0) : x1) : x2) : x3;

    wire nopen       = red3 && !I_tlast;   // 本拍 run 延续到下一拍(行尾不延续)
    wire [10:0] nopen_start = start3;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n || I_tuser) begin
            open_run   <= 1'b0;
            open_start <= 11'd0;
        end else if (I_tvalid) begin
            open_run   <= nopen;
            open_start <= nopen_start;
        end
    end

    // 按左->右顺序把闭合 run 塞进最多 3 个事件槽
    reg [2:0]  num_ev;
    reg [10:0] ev_xs [0:2];
    reg [10:0] ev_xe [0:2];
    always @(*) begin
        num_ev = 3'd0;
        ev_xs[0] = 11'd0; ev_xe[0] = 11'd0;
        ev_xs[1] = 11'd0; ev_xe[1] = 11'd0;
        ev_xs[2] = 11'd0; ev_xe[2] = 11'd0;
        if (c_carry) begin ev_xs[num_ev] = open_start; ev_xe[num_ev] = x0 - 11'd1; num_ev = num_ev + 3'd1; end
        if (c0)      begin ev_xs[num_ev] = start0;     ev_xe[num_ev] = x0;          num_ev = num_ev + 3'd1; end
        if (c1)      begin ev_xs[num_ev] = start1;     ev_xe[num_ev] = x1;          num_ev = num_ev + 3'd1; end
        if (c2)      begin ev_xs[num_ev] = start2;     ev_xe[num_ev] = x2;          num_ev = num_ev + 3'd1; end
        if (c3)      begin ev_xs[num_ev] = start3;     ev_xe[num_ev] = x3;          num_ev = num_ev + 3'd1; end
    end

    wire ev_valid0 = (num_ev >= 3'd1);
    wire ev_valid1 = (num_ev >= 3'd2);
    wire ev_valid2 = (num_ev >= 3'd3);

    //========================================================================
    // 2. run 事件 FIFO(每拍最多推 3 个, 逐拍消费 1 个)
    //    事件格式: {y[9:0], xe[10:0], xs[10:0]} = 32bit
    //========================================================================
    reg [31:0] fifo_mem [0:FIFO_DEPTH-1];
    reg [4:0]  fifo_wp, fifo_rp;
    reg [5:0]  fifo_cnt;

    wire fifo_empty = (fifo_cnt == 6'd0);
    wire fifo_not_empty = !fifo_empty;

    // 本拍事件打包(y 用当前行 v_cnt)
    wire [31:0] ev_pack0 = {v_cnt, ev_xe[0], ev_xs[0]};
    wire [31:0] ev_pack1 = {v_cnt, ev_xe[1], ev_xs[1]};
    wire [31:0] ev_pack2 = {v_cnt, ev_xe[2], ev_xs[2]};

    // 推送使能(门控 tvalid 且非 tuser, 并按 FIFO 剩余空间兜底丢弃靠后事件)
    wire push0 = ev_valid0 && I_tvalid && !I_tuser && (fifo_cnt < FIFO_DEPTH);
    wire push1 = ev_valid1 && I_tvalid && !I_tuser && (fifo_cnt + push0 < FIFO_DEPTH);
    wire push2 = ev_valid2 && I_tvalid && !I_tuser && (fifo_cnt + push0 + push1 < FIFO_DEPTH);

    wire [4:0] waddr0 = fifo_wp;
    wire [4:0] waddr1 = fifo_wp + push0;
    wire [4:0] waddr2 = fifo_wp + push0 + push1;

    // 消费一个事件(合并); tuser 拍不消费
    wire do_merge = fifo_not_empty && !I_tuser;

    wire [31:0] fifo_dout = fifo_mem[fifo_rp];
    wire [9:0]  run_y  = fifo_dout[31:22];
    wire [10:0] run_xs = fifo_dout[10:0];
    wire [10:0] run_xe = fifo_dout[21:11];

    wire [11:0] ev_len  = ({1'b0, run_xe} - {1'b0, run_xs}) + 12'd1;   // run 长度(<=1280)
    wire [31:0] ev_sumx = (({21'd0, run_xs} + {21'd0, run_xe}) * {20'd0, ev_len}) >> 1;
    wire [31:0] ev_sumy = {22'd0, run_y} * {20'd0, ev_len};

    //========================================================================
    // 3. blob 跟踪表 + 合并(组合逻辑, 单拍完成一个 run 事件)
    //========================================================================
    reg [MAX_BLOBS-1:0] b_active;
    reg [10:0] b_minx  [0:MAX_BLOBS-1];
    reg [10:0] b_maxx  [0:MAX_BLOBS-1];
    reg [9:0]  b_miny  [0:MAX_BLOBS-1];
    reg [9:0]  b_maxy  [0:MAX_BLOBS-1];
    reg [9:0]  b_lasty [0:MAX_BLOBS-1];
    reg [19:0] b_count [0:MAX_BLOBS-1];
    reg [31:0] b_sumx  [0:MAX_BLOBS-1];
    reg [31:0] b_sumy  [0:MAX_BLOBS-1];

    integer bi;

    // 匹配: 上一行且 x 有重叠的活跃 blob
    reg [MAX_BLOBS-1:0] match;
    reg        any_match;
    reg [3:0]  first_match;
    reg [3:0]  free_slot;
    reg        have_free;
    reg [10:0] m_minx, m_maxx;
    reg [9:0]  m_miny;
    reg [19:0] m_count;
    reg [31:0] m_sumx, m_sumy;
    reg        found;

    always @(*) begin
        match = {MAX_BLOBS{1'b0}};
        for (bi = 0; bi < MAX_BLOBS; bi = bi + 1) begin
            match[bi] = b_active[bi] && (b_lasty[bi] == (run_y - 10'd1)) &&
                        (b_minx[bi] <= run_xe) && (b_maxx[bi] >= run_xs);
        end
        any_match = |match;

        first_match = 4'd0;
        found = 1'b0;
        for (bi = 0; bi < MAX_BLOBS; bi = bi + 1) begin
            if (match[bi] && !found) begin
                first_match = bi[3:0];
                found = 1'b1;
            end
        end

        free_slot = 4'd0;
        have_free = 1'b0;
        for (bi = 0; bi < MAX_BLOBS; bi = bi + 1) begin
            if (!b_active[bi] && !have_free) begin
                free_slot = bi[3:0];
                have_free = 1'b1;
            end
        end

        m_minx = 11'h7FF; m_maxx = 11'd0; m_miny = 10'h3FF;
        m_count = 20'd0; m_sumx = 32'd0; m_sumy = 32'd0;
        for (bi = 0; bi < MAX_BLOBS; bi = bi + 1) begin
            if (match[bi]) begin
                if (b_minx[bi] < m_minx) m_minx = b_minx[bi];
                if (b_maxx[bi] > m_maxx) m_maxx = b_maxx[bi];
                if (b_miny[bi] < m_miny) m_miny = b_miny[bi];
                m_count = m_count + b_count[bi];
                m_sumx  = m_sumx  + b_sumx[bi];
                m_sumy  = m_sumy  + b_sumy[bi];
            end
        end
    end

    wire [10:0] mg_minx  = (run_xs < m_minx) ? run_xs : m_minx;
    wire [10:0] mg_maxx  = (run_xe > m_maxx) ? run_xe : m_maxx;
    wire [9:0]  mg_miny  = m_miny;
    wire [9:0]  mg_maxy  = run_y;
    wire [19:0] mg_count = m_count + {8'd0, ev_len};
    wire [31:0] mg_sumx  = m_sumx + ev_sumx;
    wire [31:0] mg_sumy  = m_sumy + ev_sumy;

    // 合并/新建后的下一拍跟踪表
    reg [MAX_BLOBS-1:0] nx_active;
    reg [10:0] nx_minx  [0:MAX_BLOBS-1];
    reg [10:0] nx_maxx  [0:MAX_BLOBS-1];
    reg [9:0]  nx_miny  [0:MAX_BLOBS-1];
    reg [9:0]  nx_maxy  [0:MAX_BLOBS-1];
    reg [9:0]  nx_lasty [0:MAX_BLOBS-1];
    reg [19:0] nx_count [0:MAX_BLOBS-1];
    reg [31:0] nx_sumx  [0:MAX_BLOBS-1];
    reg [31:0] nx_sumy  [0:MAX_BLOBS-1];

    always @(*) begin
        nx_active = b_active;
        for (bi = 0; bi < MAX_BLOBS; bi = bi + 1) begin
            nx_minx[bi]  = b_minx[bi];
            nx_maxx[bi]  = b_maxx[bi];
            nx_miny[bi]  = b_miny[bi];
            nx_maxy[bi]  = b_maxy[bi];
            nx_lasty[bi] = b_lasty[bi];
            nx_count[bi] = b_count[bi];
            nx_sumx[bi]  = b_sumx[bi];
            nx_sumy[bi]  = b_sumy[bi];
        end
        if (any_match) begin
            // 并入 first_match, 其余匹配槽清掉(把被这个 run 桥接的两个 blob 合并成一个)
            nx_active[first_match] = 1'b1;
            nx_minx[first_match]  = mg_minx;
            nx_maxx[first_match]  = mg_maxx;
            nx_miny[first_match]  = mg_miny;
            nx_maxy[first_match]  = mg_maxy;
            nx_lasty[first_match] = run_y;
            nx_count[first_match] = mg_count;
            nx_sumx[first_match]  = mg_sumx;
            nx_sumy[first_match]  = mg_sumy;
            for (bi = 0; bi < MAX_BLOBS; bi = bi + 1) begin
                if (match[bi] && (bi[3:0] != first_match))
                    nx_active[bi] = 1'b0;
            end
        end else if (have_free) begin
            nx_active[free_slot] = 1'b1;
            nx_minx[free_slot]  = run_xs;
            nx_maxx[free_slot]  = run_xe;
            nx_miny[free_slot]  = run_y;
            nx_maxy[free_slot]  = run_y;
            nx_lasty[free_slot] = run_y;
            nx_count[free_slot] = {8'd0, ev_len};
            nx_sumx[free_slot]  = ev_sumx;
            nx_sumy[free_slot]  = ev_sumy;
        end
    end

    //========================================================================
    // 4. 收尾条件(组合): blob 连续 1 行没被触碰即闭合; 末行/消隐期兜底全收
    //========================================================================
    reg [MAX_BLOBS-1:0] finalizable;
    integer fi;
    always @(*) begin
        for (fi = 0; fi < MAX_BLOBS; fi = fi + 1) begin
            // 闭合判定:
            //  (1) 有效期内: 连续 1 行没被触碰(lasty < 当前行-1; v_cnt>0 防下溢)
            //  (2) 场消隐: !tvalid 且 v_cnt 已回卷到 0, 收尾所有残留(含触及末行的 blob)
            //      —— 末行最后一个 beat 会把 v_cnt 回卷到 0, 因此消隐期 v_cnt==0 而非 719
            finalizable[fi] = b_active[fi] && (
                (I_tvalid && (v_cnt > 10'd0) && (b_lasty[fi] < (v_cnt - 10'd1)))
                || (!I_tvalid && (v_cnt == 10'd0))
            );
        end
    end

    //========================================================================
    // 5. 结果表 / 输出表
    //========================================================================
    reg [MAX_BOXES-1:0] res_valid;
    reg [10:0] res_minx  [0:MAX_BOXES-1];
    reg [10:0] res_maxx  [0:MAX_BOXES-1];
    reg [9:0]  res_miny  [0:MAX_BOXES-1];
    reg [9:0]  res_maxy  [0:MAX_BOXES-1];
    reg [19:0] res_count [0:MAX_BOXES-1];
    reg [31:0] res_sumx  [0:MAX_BOXES-1];
    reg [31:0] res_sumy  [0:MAX_BOXES-1];
    reg [3:0]  res_wr_ptr;

    reg [MAX_BOXES-1:0] out_valid;
    reg [10:0] out_minx  [0:MAX_BOXES-1];
    reg [10:0] out_maxx  [0:MAX_BOXES-1];
    reg [9:0]  out_miny  [0:MAX_BOXES-1];
    reg [9:0]  out_maxy  [0:MAX_BOXES-1];

    // 面积最大的结果框(组合, 供 O_min_x..O_sum_y 锁存)
    reg [10:0] lg_minx, lg_maxx;
    reg [9:0]  lg_miny, lg_maxy;
    reg [19:0] lg_count;
    reg [31:0] lg_sumx, lg_sumy;
    reg        lg_found;
    integer li;
    always @(*) begin
        lg_found = 1'b0;
        lg_minx = 11'd0; lg_maxx = 11'd0; lg_miny = 10'd0; lg_maxy = 10'd0;
        lg_count = 20'd0; lg_sumx = 32'd0; lg_sumy = 32'd0;
        for (li = 0; li < MAX_BOXES; li = li + 1) begin
            if (res_valid[li] && (!lg_found || (res_count[li] > lg_count))) begin
                lg_found = 1'b1;
                lg_minx = res_minx[li]; lg_maxx = res_maxx[li];
                lg_miny = res_miny[li]; lg_maxy = res_maxy[li];
                lg_count = res_count[li]; lg_sumx = res_sumx[li]; lg_sumy = res_sumy[li];
            end
        end
    end

    reg [2:0] scan_idx;   // MAX_BLOBS=8 -> 3bit 回卷(0..7)

    //========================================================================
    // 6. 主时序: FIFO 推/弹 + 合并 + 收尾 + 帧边界锁存
    //========================================================================
    integer ri;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            for (ri = 0; ri < FIFO_DEPTH; ri = ri + 1) fifo_mem[ri] <= 32'd0;
            fifo_wp  <= 5'd0;
            fifo_rp  <= 5'd0;
            fifo_cnt <= 6'd0;

            b_active <= {MAX_BLOBS{1'b0}};
            for (ri = 0; ri < MAX_BLOBS; ri = ri + 1) begin
                b_minx[ri]  <= 11'd0;
                b_maxx[ri]  <= 11'd0;
                b_miny[ri]  <= 10'd0;
                b_maxy[ri]  <= 10'd0;
                b_lasty[ri] <= 10'd0;
                b_count[ri] <= 20'd0;
                b_sumx[ri]  <= 32'd0;
                b_sumy[ri]  <= 32'd0;
            end

            res_valid <= {MAX_BOXES{1'b0}};
            res_wr_ptr <= 4'd0;
            out_valid <= {MAX_BOXES{1'b0}};
            scan_idx  <= 3'd0;

            O_box_valid  <= 1'b0;
            O_min_x      <= 11'd0;
            O_min_y      <= 10'd0;
            O_max_x      <= 11'd0;
            O_max_y      <= 10'd0;
            O_count      <= 20'd0;
            O_sum_x      <= 32'd0;
            O_sum_y      <= 32'd0;
            O_frame_done <= 1'b0;
        end else begin
            O_frame_done <= 1'b0;   // 默认 0, 仅 tuser 拍置 1

            if (I_tuser) begin
                // ---- 帧边界: 锁存结果 -> 输出, 清结果表与跟踪表 ----
                out_valid <= res_valid;
                for (ri = 0; ri < MAX_BOXES; ri = ri + 1) begin
                    out_minx[ri] <= res_minx[ri];
                    out_maxx[ri] <= res_maxx[ri];
                    out_miny[ri] <= res_miny[ri];
                    out_maxy[ri] <= res_maxy[ri];
                end

                O_box_valid  <= |res_valid;
                O_min_x      <= lg_minx;
                O_min_y      <= lg_miny;
                O_max_x      <= lg_maxx;
                O_max_y      <= lg_maxy;
                O_count      <= lg_count;
                O_sum_x      <= lg_sumx;
                O_sum_y      <= lg_sumy;
                O_frame_done <= 1'b1;

                res_valid   <= {MAX_BOXES{1'b0}};
                res_wr_ptr  <= 4'd0;
                b_active    <= {MAX_BLOBS{1'b0}};

                fifo_wp  <= 5'd0;
                fifo_rp  <= 5'd0;
                fifo_cnt <= 6'd0;
            end else begin
                // ---- FIFO 推/弹 ----
                fifo_wp  <= fifo_wp  + push0 + push1 + push2;
                fifo_rp  <= fifo_rp  + do_merge;
                fifo_cnt <= fifo_cnt + push0 + push1 + push2 - do_merge;

                if (push0) fifo_mem[waddr0] <= ev_pack0;
                if (push1) fifo_mem[waddr1] <= ev_pack1;
                if (push2) fifo_mem[waddr2] <= ev_pack2;

                if (do_merge) begin
                    // ---- 消费一个 run 事件: 合并/新建 blob ----
                    b_active <= nx_active;
                    for (ri = 0; ri < MAX_BLOBS; ri = ri + 1) begin
                        b_minx[ri]  <= nx_minx[ri];
                        b_maxx[ri]  <= nx_maxx[ri];
                        b_miny[ri]  <= nx_miny[ri];
                        b_maxy[ri]  <= nx_maxy[ri];
                        b_lasty[ri] <= nx_lasty[ri];
                        b_count[ri] <= nx_count[ri];
                        b_sumx[ri]  <= nx_sumx[ri];
                        b_sumy[ri]  <= nx_sumy[ri];
                    end
                end else begin
                    // ---- 空闲: 轮流收尾一个 blob ----
                    scan_idx <= scan_idx + 3'd1;
                    if (finalizable[scan_idx]) begin
                        b_active[scan_idx] <= 1'b0;
                        if ((b_count[scan_idx] >= RED_MIN_PIXELS) && (res_wr_ptr < MAX_BOXES)) begin
                            res_valid[res_wr_ptr] <= 1'b1;
                            res_minx[res_wr_ptr]  <= b_minx[scan_idx];
                            res_maxx[res_wr_ptr]  <= b_maxx[scan_idx];
                            res_miny[res_wr_ptr]  <= b_miny[scan_idx];
                            res_maxy[res_wr_ptr]  <= b_maxy[scan_idx];
                            res_count[res_wr_ptr] <= b_count[scan_idx];
                            res_sumx[res_wr_ptr]  <= b_sumx[scan_idx];
                            res_sumy[res_wr_ptr]  <= b_sumy[scan_idx];
                            res_wr_ptr <= res_wr_ptr + 4'd1;
                        end
                    end
                end
            end
        end
    end

    //========================================================================
    // 7. 框边界命中掩码(组合, 命中任意一个输出框)
    //========================================================================
    function F_hit_box;
        input [10:0] px;
        input [9:0]  py;
        input [10:0] minx, maxx;
        input [9:0]  miny, maxy;
        begin
            F_hit_box = (((py == miny) || (py == maxy)) && (px >= minx) && (px <= maxx)) ||
                        (((px == minx) || (px == maxx)) && (py >= miny) && (py <= maxy));
        end
    endfunction

    reg [3:0] hit_mask;
    integer hi;
    always @(*) begin
        hit_mask = 4'd0;
        for (hi = 0; hi < MAX_BOXES; hi = hi + 1) begin
            if (out_valid[hi]) begin
                hit_mask[0] = hit_mask[0] || F_hit_box(x0, v_cnt, out_minx[hi], out_maxx[hi], out_miny[hi], out_maxy[hi]);
                hit_mask[1] = hit_mask[1] || F_hit_box(x1, v_cnt, out_minx[hi], out_maxx[hi], out_miny[hi], out_maxy[hi]);
                hit_mask[2] = hit_mask[2] || F_hit_box(x2, v_cnt, out_minx[hi], out_maxx[hi], out_miny[hi], out_maxy[hi]);
                hit_mask[3] = hit_mask[3] || F_hit_box(x3, v_cnt, out_minx[hi], out_maxx[hi], out_miny[hi], out_maxy[hi]);
            end
        end
    end

    assign O_box_hit = hit_mask;

endmodule
