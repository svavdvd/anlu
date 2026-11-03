//============================================================================
// object_detect_3color.v -- 三色物体检测(时分复用) + 多连通域外接框 + 质心统计
//
// 方案B: 单套 blob 跟踪表 / run 事件 FIFO / 合并引擎, 跨帧时分复用跑红/蓝/绿。
//   pass 帧 N 判红 -> 帧 N+1 判蓝 -> 帧 N+2 判绿 -> 循环。
//   逐像素分类器(廉价组合逻辑)仍并行保留三份, 供调试掩码与当前 pass 选取;
//   结果锁进三份 out 表 + 三份质心 bank, 画框与机械臂接口与旧三模块完全一致。
//
// 三色判定(逐像素, HSV 空间, 无除法):
//   V = max(R,G,B);  S = (max-min)/max;  H 用"最大通道 + 次通道|差|"等效判据
//   红: max==R 且 |G-B| 小; 蓝: max==B 且 |R-G| 小; 绿: max==G 且 G>=R+B(次通道和)
//   红/蓝共用阈值: V_MIN=40, S=1/2, HUE=1/3; 绿用 G_* 阈值(更低 V、S 放宽 1/3, 判据 R+B<=2G, 数据用后 AWB)
//
// 多连通域检测(与旧三模块同一套算法, 只是分时复用一套硬件):
//   1) RLE 前端: 当前 pass 的 4 像素掩码 -> 闭合横 run [xs,xe]
//   2) run 事件 FIFO(每拍最多推 3 个, 逐拍消费 1 个)
//   3) blob 跟踪表(MAX_BLOBS 个): 上一行且 x 重叠的活跃 blob 合并, 否则新建
//   4) 收尾(finalize): 面积 >= MIN_PIXELS 写入结果表
//   5) 帧边界(tuser): 结果表锁存到「当前 pass 对应颜色」的 out 表 + 质心 bank,
//      再切到下一色
//
// 输出(每色一组, 与旧 red/blue/green_object_detect 的三模块叠加等价):
//   O_*_box_hit  = 命中该色「面积最大框」边界的掩码(每种颜色只画一个框); O_*_mask = 判色掩码(调试)
//   O_*_min_x..sum_y = 面积最大框的框+质心(供机械臂); O_*_frame_done = 该色更新脉冲
//
// 资源: 省去 2/3 的 blob 表 / FIFO / 合并算术(约 7000 FF + 1400 LUT)。
// 代价: 三色结果有 0/1/2 帧相位差(@60fps 最大 33ms)。
//============================================================================

module object_detect_3color #(
    parameter IMG_WIDTH         = 1280,     // 图像宽度(像素)
    parameter IMG_HEIGHT        = 720,      // 图像高度(行)
    parameter [7:0]  V_MIN       = 8'd40,   // 红/蓝 V(亮度 max) 下限(后 AWB 数据, R/B 已增益放大)
    parameter [3:0]  S_NUM       = 4'd1,    // 归一化饱和度下限 S = delta/max >= NUM/DEN
    parameter [3:0]  S_DEN       = 4'd2,    // 1/2 -> S >= 0.5, 拒白/灰/粉
    parameter [3:0]  HUE_NUM     = 4'd1,    // hue 窗口: |次通道差| <= delta * NUM/DEN
    parameter [3:0]  HUE_DEN     = 4'd3,    // 1/3 -> H 在中心角 ± 20° 内
    // 绿色专属阈值(绿 pass 吃后 AWB 数据, 与红/蓝同源):
    //   - 后 AWB 下白/反光高光被白平衡拉回中性(R≈G≈B, S≈0), 不会误判为绿;
    //     这正是绿从前 AWB 改到后 AWB 的原因(前 AWB 下白色天然偏绿, 与绿混淆)。
    //   - AWB 增益 R/B 会略降绿饱和度, 故 S 放宽到 1/3 恢复灵敏度;
    //     白/高光已被拉成 S≈0, 放宽不会重新招来高光误判。
    //   - AWB 只增益 R/B、不增益 G, 故 G 弱光下仍偏暗, 亮度下限单独放宽。
    //   - 色相: 冷/暖光下 R/B 失衡会让 |R-B| 变大从而误拒绿,
    //     故绿改用「次通道和 R+B」判据(R+B <= G*NUM/DEN), 对色温鲁棒。
    parameter [7:0]  G_V_MIN     = 8'd12,   // 绿 V 下限(G 不受 AWB 增益, 弱光需更低; MIN_PIXELS 抗噪)
    parameter [3:0]  G_S_NUM     = 4'd1,    // 绿色饱和度下限 S >= NUM/DEN
    parameter [3:0]  G_S_DEN     = 4'd3,    // 1/3 -> S >= 0.333, 后 AWB 白/高光 S≈0 故可放宽
    parameter [3:0]  G_HUE_NUM   = 4'd2,    // 绿: R+B <= G * NUM/DEN(次通道和)
    parameter [3:0]  G_HUE_DEN   = 4'd1,    // 2/1, 即 R+B <= 2G
    parameter [19:0] MIN_PIXELS  = 20'd100, // 单个连通域最小面积(像素)
    parameter MAX_BLOBS = 8,                // 最多同时跟踪的活跃连通域数
    parameter MAX_BOXES = 8                 // 最多输出的框数
)(
    input  wire        I_clk,
    input  wire        I_rst_n,          // 异步复位, 低有效
    input  wire [95:0] I_tdata,          // 4 像素 x RGB888
    input  wire [95:0] I_tdata_alt,      // pre-AWB 对齐数据(已弃用, 绿改用 I_tdata)
    input  wire        I_tvalid,
    input  wire        I_tuser,          // 帧开始(SOF)
    input  wire        I_tlast,          // 行结束(EOL)

    // ---- 红色结果 ----
    output reg         O_red_bbox_valid,
    output wire [3:0]  O_red_box_hit,
    output wire [3:0]  O_red_mask,
    output reg  [10:0] O_red_min_x,
    output reg  [9:0]  O_red_min_y,
    output reg  [10:0] O_red_max_x,
    output reg  [9:0]  O_red_max_y,
    output reg  [19:0] O_red_count,
    output reg  [31:0] O_red_sum_x,
    output reg  [31:0] O_red_sum_y,
    output reg         O_red_frame_done,

    // ---- 蓝色结果 ----
    output reg         O_blue_bbox_valid,
    output wire [3:0]  O_blue_box_hit,
    output wire [3:0]  O_blue_mask,
    output reg  [10:0] O_blue_min_x,
    output reg  [9:0]  O_blue_min_y,
    output reg  [10:0] O_blue_max_x,
    output reg  [9:0]  O_blue_max_y,
    output reg  [19:0] O_blue_count,
    output reg  [31:0] O_blue_sum_x,
    output reg  [31:0] O_blue_sum_y,
    output reg         O_blue_frame_done,

    // ---- 绿色结果 ----
    output reg         O_green_bbox_valid,
    output wire [3:0]  O_green_box_hit,
    output wire [3:0]  O_green_mask,
    output reg  [10:0] O_green_min_x,
    output reg  [9:0]  O_green_min_y,
    output reg  [10:0] O_green_max_x,
    output reg  [9:0]  O_green_max_y,
    output reg  [19:0] O_green_count,
    output reg  [31:0] O_green_sum_x,
    output reg  [31:0] O_green_sum_y,
    output reg         O_green_frame_done
);

    localparam [1:0] PASS_RED   = 2'd0;
    localparam [1:0] PASS_BLUE  = 2'd1;
    localparam [1:0] PASS_GREEN = 2'd2;

    localparam IMG_WIDTH_4X = IMG_WIDTH >> 2;   // 每行 4 像素组数 = 320
    localparam FIFO_DEPTH   = 32;               // run 事件 FIFO 深度(正常场景远用不满)

    //------------------------------------------------------------------------
    // 当前颜色 pass(帧边界切换): 0=红 1=蓝 2=绿
    //------------------------------------------------------------------------
    reg [1:0] pass;

    //------------------------------------------------------------------------
    // 逐像素 RGB->HSV 判定(组合逻辑, 无除法 —— 除法折叠进乘法比较)
    //   V = max, delta = max-min;
    //   饱和度: delta*S_DEN >= max*S_NUM   (等价于 delta/max >= NUM/DEN)
    //   色相  : |次通道差|*HUE_DEN <= delta*HUE_NUM, 且 主通道 == 该色
    //------------------------------------------------------------------------
    function F_hit;
        input [7:0] r;
        input [7:0] g;
        input [7:0] b;
        input [1:0] color;
        reg [7:0] mx;
        reg [7:0] mn;
        reg [7:0] delta;
        reg [8:0] sec_abs;
        reg        is_max;
        reg [3:0]  c_s_num, c_s_den, c_hue_num, c_hue_den;
        reg [7:0]  hue_base;                       // hue_rhs 乘数基: 红/蓝用 delta, 绿用 mx(G)
        reg [7:0]  v_min;                          // 亮度下限: 红/蓝 V_MIN, 绿 G_V_MIN
        reg [15:0] sat_lhs, sat_rhs, hue_lhs, hue_rhs;
        begin
            mx    = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
            mn    = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
            delta = mx - mn;                                  // max - min
            case (color)
                PASS_BLUE: begin
                    sec_abs = (r >= g) ? ({1'b0, r} - {1'b0, g}) : ({1'b0, g} - {1'b0, r});
                    is_max  = (b >= r) && (b >= g);            // H 落在蓝扇区(B 最大)
                    hue_base = delta;                           // 蓝: |R-G| 对比 delta
                    v_min    = V_MIN;                           // 蓝: 后 AWB 数据, 标准下限
                    c_s_num = S_NUM; c_s_den = S_DEN; c_hue_num = HUE_NUM; c_hue_den = HUE_DEN;
                end
                PASS_GREEN: begin
                    // 绿: 次通道判据用 R+B(而非 |R-B|)。暖光(R↑B↓)/冷光(R↓B↑)都会让
                    //   |R-B| 变大从而误拒绿; 「G 单独盖过 R+B」对 R/B 失衡完全鲁棒。
                    sec_abs = {1'b0, r} + {1'b0, b};           // R+B(0..510)
                    is_max  = (g >= r) && (g >= b);            // H 落在绿扇区(G 最大)
                    hue_base = mx;                              // 绿: R+B 对比 G(=mx)
                    v_min    = G_V_MIN;                         // 绿: G 不受 AWB 增益, 弱光仍偏暗
                    c_s_num = G_S_NUM; c_s_den = G_S_DEN; c_hue_num = G_HUE_NUM; c_hue_den = G_HUE_DEN;
                end
                default: begin
                    sec_abs = (g >= b) ? ({1'b0, g} - {1'b0, b}) : ({1'b0, b} - {1'b0, g});
                    is_max  = (r >= g) && (r >= b);            // H 落在红扇区(R 最大)
                    hue_base = delta;                           // 红: |G-B| 对比 delta
                    v_min    = V_MIN;                           // 红: 后 AWB 数据, 标准下限
                    c_s_num = S_NUM; c_s_den = S_DEN; c_hue_num = HUE_NUM; c_hue_den = HUE_DEN;
                end
            endcase
            sat_lhs = {8'd0, delta} * {12'd0, c_s_den};
            sat_rhs = {8'd0, mx}    * {12'd0, c_s_num};
            hue_rhs = {8'd0, hue_base} * {12'd0, c_hue_num};
            hue_lhs = {7'd0, sec_abs} * {12'd0, c_hue_den};
            F_hit = (mx >= v_min) &&                        // [V] 亮度下限(红/蓝 V_MIN, 绿 G_V_MIN)
                    (sat_lhs >= sat_rhs) &&                 // [S] 归一化饱和度下限
                    is_max &&                                // [H] 主通道扇区
                    (hue_lhs <= hue_rhs);                    // [H] 窗口(|次通道差| 小)
        end
    endfunction

    // 三色掩码(组合, 常开): 供调试涂色 + 当前 pass 选取
    wire red0 = F_hit(I_tdata[23:16], I_tdata[15:8],  I_tdata[7:0],   PASS_RED);
    wire red1 = F_hit(I_tdata[47:40], I_tdata[39:32], I_tdata[31:24], PASS_RED);
    wire red2 = F_hit(I_tdata[71:64], I_tdata[63:56], I_tdata[55:48], PASS_RED);
    wire red3 = F_hit(I_tdata[95:88], I_tdata[87:80], I_tdata[79:72], PASS_RED);

    wire blue0 = F_hit(I_tdata[23:16], I_tdata[15:8],  I_tdata[7:0],   PASS_BLUE);
    wire blue1 = F_hit(I_tdata[47:40], I_tdata[39:32], I_tdata[31:24], PASS_BLUE);
    wire blue2 = F_hit(I_tdata[71:64], I_tdata[63:56], I_tdata[55:48], PASS_BLUE);
    wire blue3 = F_hit(I_tdata[95:88], I_tdata[87:80], I_tdata[79:72], PASS_BLUE);

    // 绿改用后 AWB 数据(I_tdata, 与红/蓝同源): 前 AWB 下白/反光高光天然偏绿(传感器对 G
    // 更敏感 + 双线性去马赛克高光串色), 会与真绿混淆; 后 AWB 把白/高光拉回中性(R≈G≈B, S≈0)。
    wire green0 = F_hit(I_tdata[23:16], I_tdata[15:8],  I_tdata[7:0],   PASS_GREEN);
    wire green1 = F_hit(I_tdata[47:40], I_tdata[39:32], I_tdata[31:24], PASS_GREEN);
    wire green2 = F_hit(I_tdata[71:64], I_tdata[63:56], I_tdata[55:48], PASS_GREEN);
    wire green3 = F_hit(I_tdata[95:88], I_tdata[87:80], I_tdata[79:72], PASS_GREEN);

    assign O_red_mask   = {red3,   red2,   red1,   red0};
    assign O_blue_mask  = {blue3,  blue2,  blue1,  blue0};
    assign O_green_mask = {green3, green2, green1, green0};

    // 当前 pass 的命中掩码(喂 RLE 前端)
    wire hit0 = (pass == PASS_RED)   ? red0   : (pass == PASS_BLUE) ? blue0   : green0;
    wire hit1 = (pass == PASS_RED)   ? red1   : (pass == PASS_BLUE) ? blue1   : green1;
    wire hit2 = (pass == PASS_RED)   ? red2   : (pass == PASS_BLUE) ? blue2   : green2;
    wire hit3 = (pass == PASS_RED)   ? red3   : (pass == PASS_BLUE) ? blue3   : green3;

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
    // 1. RLE 前端: 每拍 4 像素命中掩码 -> 闭合横 run [xs, xe]
    //========================================================================
    reg        open_run;      // 上一拍末尾(x3)命中且同行, run 延续到本拍
    reg [10:0] open_start;    // 该延续 run 的起始 x

    wire c_carry = open_run && !hit0;
    wire c0 = hit0 && !hit1;
    wire c1 = hit1 && !hit2;
    wire c2 = hit2 && !hit3;
    wire c3 = hit3 && I_tlast;

    wire [10:0] start0 = open_run ? open_start : x0;
    wire [10:0] start1 = hit0 ? (open_run ? open_start : x0) : x1;
    wire [10:0] start2 = hit1 ? (hit0 ? (open_run ? open_start : x0) : x1) : x2;
    wire [10:0] start3 = hit2 ? (hit1 ? (hit0 ? (open_run ? open_start : x0) : x1) : x2) : x3;

    wire nopen       = hit3 && !I_tlast;   // 本拍 run 延续到下一拍(行尾不延续)
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

    wire [31:0] ev_pack0 = {v_cnt, ev_xe[0], ev_xs[0]};
    wire [31:0] ev_pack1 = {v_cnt, ev_xe[1], ev_xs[1]};
    wire [31:0] ev_pack2 = {v_cnt, ev_xe[2], ev_xs[2]};

    wire push0 = ev_valid0 && I_tvalid && !I_tuser && (fifo_cnt < FIFO_DEPTH);
    wire push1 = ev_valid1 && I_tvalid && !I_tuser && (fifo_cnt + push0 < FIFO_DEPTH);
    wire push2 = ev_valid2 && I_tvalid && !I_tuser && (fifo_cnt + push0 + push1 < FIFO_DEPTH);

    wire [4:0] waddr0 = fifo_wp;
    wire [4:0] waddr1 = fifo_wp + push0;
    wire [4:0] waddr2 = fifo_wp + push0 + push1;

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

    reg [MAX_BLOBS-1:0] match;
    reg        any_match;
    reg [3:0]  first_match;
    reg [3:0]  free_slot;
    reg        have_free;
    wire [10:0] m_minx, m_maxx;
    wire [9:0]  m_miny;
    wire [19:0] m_count;
    wire [31:0] m_sumx, m_sumy;
    reg         found;

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

    end

    //------------------------------------------------------------------------
    // 归约(平衡树, 深度 log2(MAX_BLOBS)=3): 把原串行 O(N) 的 min/max/count/sum
    // 归约改成平衡树, 消除 16 个加法器级联的关键路径(时序违例 -13.7ns 的根源)。
    // 功能完全等价(位宽足够, 无溢出), 不增加寄存器, 不改变每拍一事件的吞吐。
    //------------------------------------------------------------------------
    // 参与值选择: 未匹配的 blob 用归约恒等元, 使平衡树可无条件归约
    wire [10:0] sel_minx  [0:MAX_BLOBS-1];
    wire [10:0] sel_maxx  [0:MAX_BLOBS-1];
    wire [9:0]  sel_miny  [0:MAX_BLOBS-1];
    wire [19:0] sel_count [0:MAX_BLOBS-1];
    wire [31:0] sel_sumx  [0:MAX_BLOBS-1];
    wire [31:0] sel_sumy  [0:MAX_BLOBS-1];

    genvar gr;
    generate
        for (gr = 0; gr < MAX_BLOBS; gr = gr + 1) begin : g_sel
            assign sel_minx[gr]  = match[gr] ? b_minx[gr]  : 11'h7FF;
            assign sel_maxx[gr]  = match[gr] ? b_maxx[gr]  : 11'd0;
            assign sel_miny[gr]  = match[gr] ? b_miny[gr]  : 10'h3FF;
            assign sel_count[gr] = match[gr] ? b_count[gr] : 20'd0;
            assign sel_sumx[gr]  = match[gr] ? b_sumx[gr]  : 32'd0;
            assign sel_sumy[gr]  = match[gr] ? b_sumy[gr]  : 32'd0;
        end
    endgenerate

    // 第 1 级: 8 -> 4
    wire [10:0] l1_minx [0:3];
    wire [10:0] l1_maxx [0:3];
    wire [9:0]  l1_miny [0:3];
    wire [19:0] l1_count [0:3];
    wire [31:0] l1_sumx [0:3];
    wire [31:0] l1_sumy [0:3];

    assign l1_minx[0] = (sel_minx[0] < sel_minx[1]) ? sel_minx[0] : sel_minx[1];
    assign l1_minx[1] = (sel_minx[2] < sel_minx[3]) ? sel_minx[2] : sel_minx[3];
    assign l1_minx[2] = (sel_minx[4] < sel_minx[5]) ? sel_minx[4] : sel_minx[5];
    assign l1_minx[3] = (sel_minx[6] < sel_minx[7]) ? sel_minx[6] : sel_minx[7];
    assign l1_maxx[0] = (sel_maxx[0] > sel_maxx[1]) ? sel_maxx[0] : sel_maxx[1];
    assign l1_maxx[1] = (sel_maxx[2] > sel_maxx[3]) ? sel_maxx[2] : sel_maxx[3];
    assign l1_maxx[2] = (sel_maxx[4] > sel_maxx[5]) ? sel_maxx[4] : sel_maxx[5];
    assign l1_maxx[3] = (sel_maxx[6] > sel_maxx[7]) ? sel_maxx[6] : sel_maxx[7];
    assign l1_miny[0] = (sel_miny[0] < sel_miny[1]) ? sel_miny[0] : sel_miny[1];
    assign l1_miny[1] = (sel_miny[2] < sel_miny[3]) ? sel_miny[2] : sel_miny[3];
    assign l1_miny[2] = (sel_miny[4] < sel_miny[5]) ? sel_miny[4] : sel_miny[5];
    assign l1_miny[3] = (sel_miny[6] < sel_miny[7]) ? sel_miny[6] : sel_miny[7];
    assign l1_count[0] = sel_count[0] + sel_count[1];
    assign l1_count[1] = sel_count[2] + sel_count[3];
    assign l1_count[2] = sel_count[4] + sel_count[5];
    assign l1_count[3] = sel_count[6] + sel_count[7];
    assign l1_sumx[0]  = sel_sumx[0] + sel_sumx[1];
    assign l1_sumx[1]  = sel_sumx[2] + sel_sumx[3];
    assign l1_sumx[2]  = sel_sumx[4] + sel_sumx[5];
    assign l1_sumx[3]  = sel_sumx[6] + sel_sumx[7];
    assign l1_sumy[0]  = sel_sumy[0] + sel_sumy[1];
    assign l1_sumy[1]  = sel_sumy[2] + sel_sumy[3];
    assign l1_sumy[2]  = sel_sumy[4] + sel_sumy[5];
    assign l1_sumy[3]  = sel_sumy[6] + sel_sumy[7];

    // 第 2 级: 4 -> 2
    wire [10:0] l2_minx [0:1];
    wire [10:0] l2_maxx [0:1];
    wire [9:0]  l2_miny [0:1];
    wire [19:0] l2_count [0:1];
    wire [31:0] l2_sumx [0:1];
    wire [31:0] l2_sumy [0:1];

    assign l2_minx[0] = (l1_minx[0] < l1_minx[1]) ? l1_minx[0] : l1_minx[1];
    assign l2_minx[1] = (l1_minx[2] < l1_minx[3]) ? l1_minx[2] : l1_minx[3];
    assign l2_maxx[0] = (l1_maxx[0] > l1_maxx[1]) ? l1_maxx[0] : l1_maxx[1];
    assign l2_maxx[1] = (l1_maxx[2] > l1_maxx[3]) ? l1_maxx[2] : l1_maxx[3];
    assign l2_miny[0] = (l1_miny[0] < l1_miny[1]) ? l1_miny[0] : l1_miny[1];
    assign l2_miny[1] = (l1_miny[2] < l1_miny[3]) ? l1_miny[2] : l1_miny[3];
    assign l2_count[0] = l1_count[0] + l1_count[1];
    assign l2_count[1] = l1_count[2] + l1_count[3];
    assign l2_sumx[0]  = l1_sumx[0] + l1_sumx[1];
    assign l2_sumx[1]  = l1_sumx[2] + l1_sumx[3];
    assign l2_sumy[0]  = l1_sumy[0] + l1_sumy[1];
    assign l2_sumy[1]  = l1_sumy[2] + l1_sumy[3];

    // 第 3 级: 2 -> 1
    assign m_minx  = (l2_minx[0] < l2_minx[1]) ? l2_minx[0] : l2_minx[1];
    assign m_maxx  = (l2_maxx[0] > l2_maxx[1]) ? l2_maxx[0] : l2_maxx[1];
    assign m_miny  = (l2_miny[0] < l2_miny[1]) ? l2_miny[0] : l2_miny[1];
    assign m_count = l2_count[0] + l2_count[1];
    assign m_sumx  = l2_sumx[0] + l2_sumx[1];
    assign m_sumy  = l2_sumy[0] + l2_sumy[1];

    wire [10:0] mg_minx  = (run_xs < m_minx) ? run_xs : m_minx;
    wire [10:0] mg_maxx  = (run_xe > m_maxx) ? run_xe : m_maxx;
    wire [9:0]  mg_miny  = m_miny;
    wire [9:0]  mg_maxy  = run_y;
    wire [19:0] mg_count = m_count + {8'd0, ev_len};
    wire [31:0] mg_sumx  = m_sumx + ev_sumx;
    wire [31:0] mg_sumy  = m_sumy + ev_sumy;

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
            finalizable[fi] = b_active[fi] && (
                (I_tvalid && (v_cnt > 10'd0) && (b_lasty[fi] < (v_cnt - 10'd1)))
                || (!I_tvalid && (v_cnt == 10'd0))
            );
        end
    end

    //========================================================================
    // 5. 结果表(当前 pass 用) / 三份输出表 / 三份质心 bank
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

    // 三份输出表(画框用, 每色在各自 pass 的 tuser 更新一次)
    reg [MAX_BOXES-1:0] out_valid_r;
    reg [10:0] out_minx_r [0:MAX_BOXES-1];
    reg [10:0] out_maxx_r [0:MAX_BOXES-1];
    reg [9:0]  out_miny_r [0:MAX_BOXES-1];
    reg [9:0]  out_maxy_r [0:MAX_BOXES-1];

    reg [MAX_BOXES-1:0] out_valid_b;
    reg [10:0] out_minx_b [0:MAX_BOXES-1];
    reg [10:0] out_maxx_b [0:MAX_BOXES-1];
    reg [9:0]  out_miny_b [0:MAX_BOXES-1];
    reg [9:0]  out_maxy_b [0:MAX_BOXES-1];

    reg [MAX_BOXES-1:0] out_valid_g;
    reg [10:0] out_minx_g [0:MAX_BOXES-1];
    reg [10:0] out_maxx_g [0:MAX_BOXES-1];
    reg [9:0]  out_miny_g [0:MAX_BOXES-1];
    reg [9:0]  out_maxy_g [0:MAX_BOXES-1];

    // 面积最大的结果框(组合, 供 O_*_min_x..sum_y 锁存)
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
    // 6. 主时序: FIFO 推/弹 + 合并 + 收尾 + 帧边界锁存(按 pass 分流)
    //========================================================================
    integer ri;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            pass <= PASS_RED;

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
            scan_idx  <= 3'd0;

            out_valid_r <= {MAX_BOXES{1'b0}};
            out_valid_b <= {MAX_BOXES{1'b0}};
            out_valid_g <= {MAX_BOXES{1'b0}};
            for (ri = 0; ri < MAX_BOXES; ri = ri + 1) begin
                out_minx_r[ri] <= 11'd0; out_maxx_r[ri] <= 11'd0;
                out_miny_r[ri] <= 10'd0; out_maxy_r[ri] <= 10'd0;
                out_minx_b[ri] <= 11'd0; out_maxx_b[ri] <= 11'd0;
                out_miny_b[ri] <= 10'd0; out_maxy_b[ri] <= 10'd0;
                out_minx_g[ri] <= 11'd0; out_maxx_g[ri] <= 11'd0;
                out_miny_g[ri] <= 10'd0; out_maxy_g[ri] <= 10'd0;
            end

            O_red_bbox_valid   <= 1'b0;
            O_red_min_x        <= 11'd0;
            O_red_min_y        <= 10'd0;
            O_red_max_x        <= 11'd0;
            O_red_max_y        <= 10'd0;
            O_red_count        <= 20'd0;
            O_red_sum_x        <= 32'd0;
            O_red_sum_y        <= 32'd0;
            O_red_frame_done   <= 1'b0;

            O_blue_bbox_valid  <= 1'b0;
            O_blue_min_x       <= 11'd0;
            O_blue_min_y       <= 10'd0;
            O_blue_max_x       <= 11'd0;
            O_blue_max_y       <= 10'd0;
            O_blue_count       <= 20'd0;
            O_blue_sum_x       <= 32'd0;
            O_blue_sum_y       <= 32'd0;
            O_blue_frame_done  <= 1'b0;

            O_green_bbox_valid <= 1'b0;
            O_green_min_x      <= 11'd0;
            O_green_min_y      <= 10'd0;
            O_green_max_x      <= 11'd0;
            O_green_max_y      <= 10'd0;
            O_green_count      <= 20'd0;
            O_green_sum_x      <= 32'd0;
            O_green_sum_y      <= 32'd0;
            O_green_frame_done <= 1'b0;
        end else begin
            O_red_frame_done   <= 1'b0;   // 默认 0, 仅本色 tuser 拍置 1
            O_blue_frame_done  <= 1'b0;
            O_green_frame_done <= 1'b0;

            if (I_tuser) begin
                // ---- 帧边界: 当前 pass 的结果锁存到对应颜色 bank, 再切 pass ----
                case (pass)
                    PASS_BLUE: begin
                        O_blue_bbox_valid <= |res_valid;
                        O_blue_min_x      <= lg_minx;
                        O_blue_min_y      <= lg_miny;
                        O_blue_max_x      <= lg_maxx;
                        O_blue_max_y      <= lg_maxy;
                        O_blue_count      <= lg_count;
                        O_blue_sum_x      <= lg_sumx;
                        O_blue_sum_y      <= lg_sumy;
                        O_blue_frame_done <= 1'b1;

                        // 屏上每种颜色只画最大框: out 表只存 slot0(=lg, 面积最大框)
                        out_valid_b <= { MAX_BOXES{1'b0} };
                        out_valid_b[0] <= lg_found;
                        out_minx_b[0] <= lg_minx;
                        out_maxx_b[0] <= lg_maxx;
                        out_miny_b[0] <= lg_miny;
                        out_maxy_b[0] <= lg_maxy;
                    end
                    PASS_GREEN: begin
                        O_green_bbox_valid <= |res_valid;
                        O_green_min_x      <= lg_minx;
                        O_green_min_y      <= lg_miny;
                        O_green_max_x      <= lg_maxx;
                        O_green_max_y      <= lg_maxy;
                        O_green_count      <= lg_count;
                        O_green_sum_x      <= lg_sumx;
                        O_green_sum_y      <= lg_sumy;
                        O_green_frame_done <= 1'b1;

                        // 屏上每种颜色只画最大框: out 表只存 slot0(=lg, 面积最大框)
                        out_valid_g <= { MAX_BOXES{1'b0} };
                        out_valid_g[0] <= lg_found;
                        out_minx_g[0] <= lg_minx;
                        out_maxx_g[0] <= lg_maxx;
                        out_miny_g[0] <= lg_miny;
                        out_maxy_g[0] <= lg_maxy;
                    end
                    default: begin   // PASS_RED
                        O_red_bbox_valid <= |res_valid;
                        O_red_min_x      <= lg_minx;
                        O_red_min_y      <= lg_miny;
                        O_red_max_x      <= lg_maxx;
                        O_red_max_y      <= lg_maxy;
                        O_red_count      <= lg_count;
                        O_red_sum_x      <= lg_sumx;
                        O_red_sum_y      <= lg_sumy;
                        O_red_frame_done <= 1'b1;

                        // 屏上每种颜色只画最大框: out 表只存 slot0(=lg, 面积最大框)
                        out_valid_r <= { MAX_BOXES{1'b0} };
                        out_valid_r[0] <= lg_found;
                        out_minx_r[0] <= lg_minx;
                        out_maxx_r[0] <= lg_maxx;
                        out_miny_r[0] <= lg_miny;
                        out_maxy_r[0] <= lg_maxy;
                    end
                endcase

                res_valid   <= {MAX_BOXES{1'b0}};
                res_wr_ptr  <= 4'd0;
                b_active    <= {MAX_BLOBS{1'b0}};

                fifo_wp  <= 5'd0;
                fifo_rp  <= 5'd0;
                fifo_cnt <= 6'd0;

                pass <= (pass == PASS_GREEN) ? PASS_RED : pass + 2'd1;
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
                        if ((b_count[scan_idx] >= MIN_PIXELS) && (res_wr_ptr < MAX_BOXES)) begin
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
    // 7. 框边界命中掩码(组合, 每色一套, 命中该色任一输出框)
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

    reg [3:0] hit_mask_r;
    reg [3:0] hit_mask_b;
    reg [3:0] hit_mask_g;
    integer hi;
    always @(*) begin
        hit_mask_r = 4'd0;
        hit_mask_b = 4'd0;
        hit_mask_g = 4'd0;
        for (hi = 0; hi < MAX_BOXES; hi = hi + 1) begin
            if (out_valid_r[hi]) begin
                hit_mask_r[0] = hit_mask_r[0] || F_hit_box(x0, v_cnt, out_minx_r[hi], out_maxx_r[hi], out_miny_r[hi], out_maxy_r[hi]);
                hit_mask_r[1] = hit_mask_r[1] || F_hit_box(x1, v_cnt, out_minx_r[hi], out_maxx_r[hi], out_miny_r[hi], out_maxy_r[hi]);
                hit_mask_r[2] = hit_mask_r[2] || F_hit_box(x2, v_cnt, out_minx_r[hi], out_maxx_r[hi], out_miny_r[hi], out_maxy_r[hi]);
                hit_mask_r[3] = hit_mask_r[3] || F_hit_box(x3, v_cnt, out_minx_r[hi], out_maxx_r[hi], out_miny_r[hi], out_maxy_r[hi]);
            end
            if (out_valid_b[hi]) begin
                hit_mask_b[0] = hit_mask_b[0] || F_hit_box(x0, v_cnt, out_minx_b[hi], out_maxx_b[hi], out_miny_b[hi], out_maxy_b[hi]);
                hit_mask_b[1] = hit_mask_b[1] || F_hit_box(x1, v_cnt, out_minx_b[hi], out_maxx_b[hi], out_miny_b[hi], out_maxy_b[hi]);
                hit_mask_b[2] = hit_mask_b[2] || F_hit_box(x2, v_cnt, out_minx_b[hi], out_maxx_b[hi], out_miny_b[hi], out_maxy_b[hi]);
                hit_mask_b[3] = hit_mask_b[3] || F_hit_box(x3, v_cnt, out_minx_b[hi], out_maxx_b[hi], out_miny_b[hi], out_maxy_b[hi]);
            end
            if (out_valid_g[hi]) begin
                hit_mask_g[0] = hit_mask_g[0] || F_hit_box(x0, v_cnt, out_minx_g[hi], out_maxx_g[hi], out_miny_g[hi], out_maxy_g[hi]);
                hit_mask_g[1] = hit_mask_g[1] || F_hit_box(x1, v_cnt, out_minx_g[hi], out_maxx_g[hi], out_miny_g[hi], out_maxy_g[hi]);
                hit_mask_g[2] = hit_mask_g[2] || F_hit_box(x2, v_cnt, out_minx_g[hi], out_maxx_g[hi], out_miny_g[hi], out_maxy_g[hi]);
                hit_mask_g[3] = hit_mask_g[3] || F_hit_box(x3, v_cnt, out_minx_g[hi], out_maxx_g[hi], out_miny_g[hi], out_maxy_g[hi]);
            end
        end
    end

    assign O_red_box_hit   = hit_mask_r;
    assign O_blue_box_hit  = hit_mask_b;
    assign O_green_box_hit = hit_mask_g;

endmodule
