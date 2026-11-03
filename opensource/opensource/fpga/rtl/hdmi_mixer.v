module hdmi_mixer #(
    parameter H_OFFSET = 128,
    parameter V_OFFSET = 60,
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720,
    parameter integer DEBUG_MODE = 0
)(
    input wire        I_clk,
    input wire        I_rst_n,

    input wire        I_video_vsync,
    input wire        I_video_hsync,
    input wire        I_video_de,
    input wire        I_video_user,
    input wire        I_video_last,

    input wire[3:0]   I_debug_status,
    input wire[2:0]   I_img_mode,

    // 实时帧率(两位 BCD, 由 fps_meter 在显示时钟域统计)
    input wire[3:0]   I_fps_tens,
    input wire[3:0]   I_fps_ones,

    output wire       O_video_rd_en,
    input wire[23:0]  I_video_rd_data,

    output reg        O_hdmi_vsync,
    output reg        O_hdmi_hsync,
    output reg        O_hdmi_de,
    output reg[23:0]  O_hdmi_data
);

    reg        S_video_vsync_1d;
    reg        S_video_vsync_2d;
    reg        S_video_hsync_1d;
    reg        S_video_hsync_2d;
    reg        S_video_de_1d;
    reg        S_video_de_2d;
    reg [23:0] S_video_data_1d;
    reg [23:0] S_video_data_2d;
    reg        S_video_vsync_3d;
    reg        S_video_hsync_3d;
    reg        S_video_de_3d;
    reg [23:0] S_video_data_3d;
    reg        S_osd_hit_r;
    reg [23:0] S_osd_color_r;
    reg [11:0] S_x;
    reg [11:0] S_y;
    reg [11:0] S_x_1d;
    reg [11:0] S_x_2d;
    reg [11:0] S_y_1d;
    reg [11:0] S_y_2d;

    reg        S_osd_hit;
    reg [23:0] S_osd_color;
    reg [11:0] S_logo_x;
    reg [11:0] S_logo_y;
    reg [3:0]  S_char_idx;
    reg [2:0]  S_char_row;
    reg [2:0]  S_char_col;
    reg [4:0]  S_char_bits;
    reg        S_char_on;
    reg        S_logo_blue_on;
    reg        S_logo_red_on;
    reg [11:0] S_logo_tmp_y;
    reg [3:0]  S_logo_red_band;
    reg [11:0] S_logo_red_x_start;
    reg [11:0] S_logo_red_x_end;
    reg [14:0] S_logo_addr;
    wire[24:0] S_logo_pixel;
    reg [31:0] S_frame_cnt;
    reg [11:0] S_dyn_x;
    reg [11:0] S_dyn_y;
    reg [3:0]  S_dyn_char;
    reg [2:0]  S_dyn_col;
    reg [2:0]  S_dyn_row;
    reg [4:0]  S_dyn_bits;
    reg        S_dyn_on;
    reg [3:0]  S_dyn_digit_idx;
    reg [11:0] S_dyn_digit_x;
    reg [6:0]  S_dyn_seg;
    reg [3:0]  S_cnt_d0;
    reg [3:0]  S_cnt_d1;
    reg [3:0]  S_cnt_d2;
    reg [3:0]  S_cnt_d3;
    reg [3:0]  S_cnt_d4;
    reg [3:0]  S_cnt_d5;
    reg        S_txt_hit;      // 当前像素落在两行黄色状态文字内
    reg        S_txt_sel;      // 0=累计帧数行(安路CNT), 1=实时帧率行(安路FPS)
    reg        S_dyn_dupd;     // 当前数字位有效(超出数字个数则不画)
    reg [7:0]  S_mt_lx;        // 模式文字: 行内偏移(0..159)
    reg [3:0]  S_mt_col;       // 模式文字: 字内列 0..15
    reg        S_mt_on;        // 模式文字: 该像素点亮
    reg [15:0] S_mt_bits_r;    // 模式文字: 提前一拍锁存的字模行
    reg [2:0]  S_lbl_row;
    reg [2:0]  S_lbl_col;
    reg [1:0]  S_lbl_idx;
    reg [4:0]  S_lbl_bits;
    reg [6:0]  S_lbl_local_x;
    reg [3:0]  S_dyn_dec_digit;
    reg [4:0]  S_cn_local_x;
    reg [4:0]  S_cn_local_y;
    reg [3:0]  S_cn_col;
    reg [3:0]  S_cn_row;
    reg [15:0] S_cn_row_bits;
    reg        S_cn_on;
    reg [9:0]  S_cn_scale_x;
    reg [9:0]  S_cn_scale_y;
    reg [7:0]  S_pf_char;
    reg [4:0]  S_pf_local_x;
    reg [4:0]  S_pf_local_y;
    reg [9:0]  S_pf_scale_x;
    reg [9:0]  S_pf_scale_y;
    reg [3:0]  S_pf_col;
    reg [3:0]  S_pf_row;
    wire [15:0] S_pf_row_bits;
    reg        S_pf_on;

    localparam LOGO_X = 12'd0;
    localparam LOGO_Y = 12'd0;
    localparam LOGO_W = 12'd160;
    localparam LOGO_H = 12'd160;

    localparam COLOR_LOGO_BLUE   = 24'h1f315c;
    localparam COLOR_LOGO_RED    = 24'hec1c2d;
    localparam COLOR_TEXT        = 24'h00e6ff;
    localparam DYN_OSD_X         = 12'd375;
    localparam DYN_OSD_Y         = 12'd120;
    localparam DYN_OSD_W         = 12'd200;
    localparam DYN_OSD_H         = 12'd24;
    localparam CN_Y_OFFSET       = 12'd2;
    localparam COLOR_DYN_TEXT    = 24'hfff200;
    localparam CHAR_AN           = 8'h80;
    localparam CHAR_LU           = 8'h81;
    localparam CHAR_C            = 8'h43;
    localparam CHAR_N            = 8'h4e;
    localparam CHAR_T            = 8'h54;
    localparam CHAR_0            = 8'h30;
    localparam CHAR_COLON        = 8'h3a;

    //---- 左上角 LOGO 右侧的两行黄色状态文字 (上: 累计帧数, 下: 实时帧率) ----
    // LOGO 占 (0,0)-(160,160); 文字从 x=176 起, 两行各 24 高, 一上一下。
    // 行内布局(与原有"安路CNT:xxxxxx"完全一致, 只是整体平移):
    //   0..23 安 | 24..47 路 | 52..99 三字母(C N T / F P S) | 100..107 冒号 | 110.. 数字
    localparam TXT_X             = 12'd176;
    localparam TXT_Y1            = 12'd40;    // 第 1 行: 安路CNT:<6 位累计帧数>
    localparam TXT_Y2            = 12'd76;    // 第 2 行: 安路FPS:<2 位实时帧率>
    localparam TXT_W             = 12'd200;
    localparam TXT_H             = 12'd24;

    //---- 右上角: 图像处理档位名称文字 (原 32x32 色块位置) ----
    // 16x16 中文字模 2 倍放大 -> 每字 32x32; 4 字(128 宽), 2 值化档 5 字(160 宽),
    // 统一右对齐到 x=1272, 向上贴住原来色块的纵向范围。
    localparam MODE_TXT_Y        = 12'd12;
    localparam MODE_TXT_H        = 12'd32;
    localparam MODE_TXT_X4       = 12'd1144;  // 4 字(128 宽) 左侧起点
    localparam MODE_TXT_X5       = 12'd1112;  // 5 字(160 宽) 左侧起点
    localparam MODE_TXT_W4       = 12'd128;
    localparam MODE_TXT_W5       = 12'd160;

    localparam CHAR_F            = 8'h46;
    localparam CHAR_P            = 8'h50;
    localparam CHAR_S            = 8'h53;

    function [4:0] F_logo_text_bits;
        input [3:0] I_char;
        input [2:0] I_row;
        begin
            case(I_char)
                4'd0: begin
                    case(I_row) // A
                        3'd0: F_logo_text_bits = 5'b01110;
                        3'd1: F_logo_text_bits = 5'b10001;
                        3'd2: F_logo_text_bits = 5'b10001;
                        3'd3: F_logo_text_bits = 5'b11111;
                        3'd4: F_logo_text_bits = 5'b10001;
                        3'd5: F_logo_text_bits = 5'b10001;
                        3'd6: F_logo_text_bits = 5'b10001;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd1: begin
                    case(I_row) // N
                        3'd0: F_logo_text_bits = 5'b10001;
                        3'd1: F_logo_text_bits = 5'b11001;
                        3'd2: F_logo_text_bits = 5'b10101;
                        3'd3: F_logo_text_bits = 5'b10011;
                        3'd4: F_logo_text_bits = 5'b10001;
                        3'd5: F_logo_text_bits = 5'b10001;
                        3'd6: F_logo_text_bits = 5'b10001;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd2: begin
                    case(I_row) // L
                        3'd0: F_logo_text_bits = 5'b10000;
                        3'd1: F_logo_text_bits = 5'b10000;
                        3'd2: F_logo_text_bits = 5'b10000;
                        3'd3: F_logo_text_bits = 5'b10000;
                        3'd4: F_logo_text_bits = 5'b10000;
                        3'd5: F_logo_text_bits = 5'b10000;
                        3'd6: F_logo_text_bits = 5'b11111;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd3: begin
                    case(I_row) // O
                        3'd0: F_logo_text_bits = 5'b01110;
                        3'd1: F_logo_text_bits = 5'b10001;
                        3'd2: F_logo_text_bits = 5'b10001;
                        3'd3: F_logo_text_bits = 5'b10001;
                        3'd4: F_logo_text_bits = 5'b10001;
                        3'd5: F_logo_text_bits = 5'b10001;
                        3'd6: F_logo_text_bits = 5'b01110;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd4: begin
                    case(I_row) // G
                        3'd0: F_logo_text_bits = 5'b01110;
                        3'd1: F_logo_text_bits = 5'b10001;
                        3'd2: F_logo_text_bits = 5'b10000;
                        3'd3: F_logo_text_bits = 5'b10111;
                        3'd4: F_logo_text_bits = 5'b10001;
                        3'd5: F_logo_text_bits = 5'b10001;
                        3'd6: F_logo_text_bits = 5'b01110;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd5: begin
                    case(I_row) // I
                        3'd0: F_logo_text_bits = 5'b11111;
                        3'd1: F_logo_text_bits = 5'b00100;
                        3'd2: F_logo_text_bits = 5'b00100;
                        3'd3: F_logo_text_bits = 5'b00100;
                        3'd4: F_logo_text_bits = 5'b00100;
                        3'd5: F_logo_text_bits = 5'b00100;
                        3'd6: F_logo_text_bits = 5'b11111;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd6: begin
                    case(I_row) // C
                        3'd0: F_logo_text_bits = 5'b01110;
                        3'd1: F_logo_text_bits = 5'b10001;
                        3'd2: F_logo_text_bits = 5'b10000;
                        3'd3: F_logo_text_bits = 5'b10000;
                        3'd4: F_logo_text_bits = 5'b10000;
                        3'd5: F_logo_text_bits = 5'b10001;
                        3'd6: F_logo_text_bits = 5'b01110;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                default: F_logo_text_bits = 5'b00000;
            endcase
        end
    endfunction

    // 7-seg bit mapping: {a,b,c,d,e,f,g}
    function [6:0] F_hex7seg;
        input [3:0] I_hex;
        begin
            case(I_hex)
                4'h0: F_hex7seg = 7'b1111110;
                4'h1: F_hex7seg = 7'b0110000;
                4'h2: F_hex7seg = 7'b1101101;
                4'h3: F_hex7seg = 7'b1111001;
                4'h4: F_hex7seg = 7'b0110011;
                4'h5: F_hex7seg = 7'b1011011;
                4'h6: F_hex7seg = 7'b1011111;
                4'h7: F_hex7seg = 7'b1110000;
                4'h8: F_hex7seg = 7'b1111111;
                4'h9: F_hex7seg = 7'b1111011;
                4'ha: F_hex7seg = 7'b1110111;
                4'hb: F_hex7seg = 7'b0011111;
                4'hc: F_hex7seg = 7'b1001110;
                4'hd: F_hex7seg = 7'b0111101;
                4'he: F_hex7seg = 7'b1001111;
                4'hf: F_hex7seg = 7'b1000111;
                default: F_hex7seg = 7'b0000000;
            endcase
        end
    endfunction

    function [4:0] F_cnt_label_bits;
        input [1:0] I_idx;
        input [2:0] I_row;
        begin
            case(I_idx)
                2'd0: begin // C
                    case(I_row)
                        3'd0: F_cnt_label_bits = 5'b01110;
                        3'd1: F_cnt_label_bits = 5'b10001;
                        3'd2: F_cnt_label_bits = 5'b10000;
                        3'd3: F_cnt_label_bits = 5'b10000;
                        3'd4: F_cnt_label_bits = 5'b10000;
                        3'd5: F_cnt_label_bits = 5'b10001;
                        3'd6: F_cnt_label_bits = 5'b01110;
                        default: F_cnt_label_bits = 5'b00000;
                    endcase
                end
                2'd1: begin // N
                    case(I_row)
                        3'd0: F_cnt_label_bits = 5'b10001;
                        3'd1: F_cnt_label_bits = 5'b11001;
                        3'd2: F_cnt_label_bits = 5'b10101;
                        3'd3: F_cnt_label_bits = 5'b10011;
                        3'd4: F_cnt_label_bits = 5'b10001;
                        3'd5: F_cnt_label_bits = 5'b10001;
                        3'd6: F_cnt_label_bits = 5'b10001;
                        default: F_cnt_label_bits = 5'b00000;
                    endcase
                end
                2'd2: begin // T
                    case(I_row)
                        3'd0: F_cnt_label_bits = 5'b11111;
                        3'd1: F_cnt_label_bits = 5'b00100;
                        3'd2: F_cnt_label_bits = 5'b00100;
                        3'd3: F_cnt_label_bits = 5'b00100;
                        3'd4: F_cnt_label_bits = 5'b00100;
                        3'd5: F_cnt_label_bits = 5'b00100;
                        3'd6: F_cnt_label_bits = 5'b00100;
                        default: F_cnt_label_bits = 5'b00000;
                    endcase
                end
                default: begin // ':'
                    case(I_row)
                        3'd2: F_cnt_label_bits = 5'b00100;
                        3'd5: F_cnt_label_bits = 5'b00100;
                        default: F_cnt_label_bits = 5'b00000;
                    endcase
                end
            endcase
        end
    endfunction

    // Minimal 16x16 Chinese bitmap rows for "an" and "lu".
    function [15:0] F_cn16_row;
        input I_char; // 0: an, 1: lu
        input [3:0] I_row;
        begin
            if(!I_char) begin
                case(I_row)
                    4'd0:  F_cn16_row = 16'h0200;
                    4'd1:  F_cn16_row = 16'h7ff0;
                    4'd2:  F_cn16_row = 16'h4410;
                    4'd3:  F_cn16_row = 16'h0400;
                    4'd4:  F_cn16_row = 16'h7ff0;
                    4'd5:  F_cn16_row = 16'h0840;
                    4'd6:  F_cn16_row = 16'h1040;
                    4'd7:  F_cn16_row = 16'h1c80;
                    4'd8:  F_cn16_row = 16'h0300;
                    4'd9:  F_cn16_row = 16'h0cc0;
                    4'd10: F_cn16_row = 16'h7020;
                    4'd11: F_cn16_row = 16'h0000;
                    4'd12: F_cn16_row = 16'h0000;
                    4'd13: F_cn16_row = 16'h0000;
                    4'd14: F_cn16_row = 16'h0000;
                    4'd15: F_cn16_row = 16'h0000;
                    default: F_cn16_row = 16'h0000;
                endcase
            end
            else begin
                case(I_row)
                    4'd0:  F_cn16_row = 16'h79e0;
                    4'd1:  F_cn16_row = 16'h4a20;
                    4'd2:  F_cn16_row = 16'h4d40;
                    4'd3:  F_cn16_row = 16'h7880;
                    4'd4:  F_cn16_row = 16'h1140;
                    4'd5:  F_cn16_row = 16'h1630;
                    4'd6:  F_cn16_row = 16'h5be0;
                    4'd7:  F_cn16_row = 16'h5220;
                    4'd8:  F_cn16_row = 16'h5220;
                    4'd9:  F_cn16_row = 16'h5be0;
                    4'd10: F_cn16_row = 16'h6220;
                    4'd11: F_cn16_row = 16'h0000;
                    4'd12: F_cn16_row = 16'h0000;
                    4'd13: F_cn16_row = 16'h0000;
                    4'd14: F_cn16_row = 16'h0000;
                    4'd15: F_cn16_row = 16'h0000;
                    default: F_cn16_row = 16'h0000;
                endcase
            end
        end
    endfunction

    function [4:0] F_dyn_char_bits;
        input [3:0] I_char;
        input [2:0] I_row;
        begin
            case(I_char)
                4'h0: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10011; 3'd3:F_dyn_char_bits=5'b10101; 3'd4:F_dyn_char_bits=5'b11001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h1: begin case(I_row) 3'd0:F_dyn_char_bits=5'b00100; 3'd1:F_dyn_char_bits=5'b01100; 3'd2:F_dyn_char_bits=5'b00100; 3'd3:F_dyn_char_bits=5'b00100; 3'd4:F_dyn_char_bits=5'b00100; 3'd5:F_dyn_char_bits=5'b00100; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h2: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b00001; 3'd3:F_dyn_char_bits=5'b00110; 3'd4:F_dyn_char_bits=5'b01000; 3'd5:F_dyn_char_bits=5'b10000; 3'd6:F_dyn_char_bits=5'b11111; default:F_dyn_char_bits=5'b00000; endcase end
                4'h3: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11110; 3'd1:F_dyn_char_bits=5'b00001; 3'd2:F_dyn_char_bits=5'b00001; 3'd3:F_dyn_char_bits=5'b01110; 3'd4:F_dyn_char_bits=5'b00001; 3'd5:F_dyn_char_bits=5'b00001; 3'd6:F_dyn_char_bits=5'b11110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h4: begin case(I_row) 3'd0:F_dyn_char_bits=5'b00010; 3'd1:F_dyn_char_bits=5'b00110; 3'd2:F_dyn_char_bits=5'b01010; 3'd3:F_dyn_char_bits=5'b10010; 3'd4:F_dyn_char_bits=5'b11111; 3'd5:F_dyn_char_bits=5'b00010; 3'd6:F_dyn_char_bits=5'b00010; default:F_dyn_char_bits=5'b00000; endcase end
                4'h5: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11111; 3'd1:F_dyn_char_bits=5'b10000; 3'd2:F_dyn_char_bits=5'b11110; 3'd3:F_dyn_char_bits=5'b00001; 3'd4:F_dyn_char_bits=5'b00001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h6: begin case(I_row) 3'd0:F_dyn_char_bits=5'b00110; 3'd1:F_dyn_char_bits=5'b01000; 3'd2:F_dyn_char_bits=5'b10000; 3'd3:F_dyn_char_bits=5'b11110; 3'd4:F_dyn_char_bits=5'b10001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h7: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11111; 3'd1:F_dyn_char_bits=5'b00001; 3'd2:F_dyn_char_bits=5'b00010; 3'd3:F_dyn_char_bits=5'b00100; 3'd4:F_dyn_char_bits=5'b01000; 3'd5:F_dyn_char_bits=5'b01000; 3'd6:F_dyn_char_bits=5'b01000; default:F_dyn_char_bits=5'b00000; endcase end
                4'h8: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10001; 3'd3:F_dyn_char_bits=5'b01110; 3'd4:F_dyn_char_bits=5'b10001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h9: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10001; 3'd3:F_dyn_char_bits=5'b01111; 3'd4:F_dyn_char_bits=5'b00001; 3'd5:F_dyn_char_bits=5'b00010; 3'd6:F_dyn_char_bits=5'b11100; default:F_dyn_char_bits=5'b00000; endcase end
                4'ha: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10001; 3'd3:F_dyn_char_bits=5'b11111; 3'd4:F_dyn_char_bits=5'b10001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b10001; default:F_dyn_char_bits=5'b00000; endcase end
                4'hb: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10001; 3'd3:F_dyn_char_bits=5'b11110; 3'd4:F_dyn_char_bits=5'b10001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b11110; default:F_dyn_char_bits=5'b00000; endcase end
                4'hc: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10000; 3'd3:F_dyn_char_bits=5'b10000; 3'd4:F_dyn_char_bits=5'b10000; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'hd: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10001; 3'd3:F_dyn_char_bits=5'b10001; 3'd4:F_dyn_char_bits=5'b10001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b11110; default:F_dyn_char_bits=5'b00000; endcase end
                4'he: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11111; 3'd1:F_dyn_char_bits=5'b10000; 3'd2:F_dyn_char_bits=5'b10000; 3'd3:F_dyn_char_bits=5'b11110; 3'd4:F_dyn_char_bits=5'b10000; 3'd5:F_dyn_char_bits=5'b10000; 3'd6:F_dyn_char_bits=5'b11111; default:F_dyn_char_bits=5'b00000; endcase end
                4'hf: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11111; 3'd1:F_dyn_char_bits=5'b10000; 3'd2:F_dyn_char_bits=5'b10000; 3'd3:F_dyn_char_bits=5'b11110; 3'd4:F_dyn_char_bits=5'b10000; 3'd5:F_dyn_char_bits=5'b10000; 3'd6:F_dyn_char_bits=5'b10000; default:F_dyn_char_bits=5'b00000; endcase end
                default: F_dyn_char_bits = 5'b00000;
            endcase
        end
    endfunction

    // Unified prefix character library (16x16 rows): an/lu/C/N/T/:
    function [15:0] F_osd_char16_row;
        input [7:0] I_char;
        input [3:0] I_row;
        begin
            case(I_char)
                CHAR_AN: begin
                    case(I_row)
                        4'd0:  F_osd_char16_row = 16'h0200;
                        4'd1:  F_osd_char16_row = 16'h7ff0;
                        4'd2:  F_osd_char16_row = 16'h4410;
                        4'd3:  F_osd_char16_row = 16'h0400;
                        4'd4:  F_osd_char16_row = 16'h7ff0;
                        4'd5:  F_osd_char16_row = 16'h0840;
                        4'd6:  F_osd_char16_row = 16'h1040;
                        4'd7:  F_osd_char16_row = 16'h1c80;
                        4'd8:  F_osd_char16_row = 16'h0300;
                        4'd9:  F_osd_char16_row = 16'h0cc0;
                        4'd10: F_osd_char16_row = 16'h7020;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                CHAR_LU: begin
                    case(I_row)
                        4'd0:  F_osd_char16_row = 16'h79e0;
                        4'd1:  F_osd_char16_row = 16'h4a20;
                        4'd2:  F_osd_char16_row = 16'h4d40;
                        4'd3:  F_osd_char16_row = 16'h7880;
                        4'd4:  F_osd_char16_row = 16'h1140;
                        4'd5:  F_osd_char16_row = 16'h1630;
                        4'd6:  F_osd_char16_row = 16'h5be0;
                        4'd7:  F_osd_char16_row = 16'h5220;
                        4'd8:  F_osd_char16_row = 16'h5220;
                        4'd9:  F_osd_char16_row = 16'h5be0;
                        4'd10: F_osd_char16_row = 16'h6220;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                CHAR_C: begin
                    case(I_row)
                        4'd2:  F_osd_char16_row = 16'h03f0;
                        4'd3:  F_osd_char16_row = 16'h0c0c;
                        4'd4:  F_osd_char16_row = 16'h1800;
                        4'd5:  F_osd_char16_row = 16'h3000;
                        4'd6:  F_osd_char16_row = 16'h3000;
                        4'd7:  F_osd_char16_row = 16'h3000;
                        4'd8:  F_osd_char16_row = 16'h3000;
                        4'd9:  F_osd_char16_row = 16'h3000;
                        4'd10: F_osd_char16_row = 16'h1800;
                        4'd11: F_osd_char16_row = 16'h0c0c;
                        4'd12: F_osd_char16_row = 16'h03f0;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                CHAR_N: begin
                    case(I_row)
                        4'd2:  F_osd_char16_row = 16'h300c;
                        4'd3:  F_osd_char16_row = 16'h380c;
                        4'd4:  F_osd_char16_row = 16'h3c0c;
                        4'd5:  F_osd_char16_row = 16'h360c;
                        4'd6:  F_osd_char16_row = 16'h330c;
                        4'd7:  F_osd_char16_row = 16'h318c;
                        4'd8:  F_osd_char16_row = 16'h30cc;
                        4'd9:  F_osd_char16_row = 16'h306c;
                        4'd10: F_osd_char16_row = 16'h303c;
                        4'd11: F_osd_char16_row = 16'h301c;
                        4'd12: F_osd_char16_row = 16'h300c;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                CHAR_T: begin
                    case(I_row)
                        4'd2:  F_osd_char16_row = 16'h3ffc;
                        4'd3:  F_osd_char16_row = 16'h03c0;
                        4'd4:  F_osd_char16_row = 16'h03c0;
                        4'd5:  F_osd_char16_row = 16'h03c0;
                        4'd6:  F_osd_char16_row = 16'h03c0;
                        4'd7:  F_osd_char16_row = 16'h03c0;
                        4'd8:  F_osd_char16_row = 16'h03c0;
                        4'd9:  F_osd_char16_row = 16'h03c0;
                        4'd10: F_osd_char16_row = 16'h03c0;
                        4'd11: F_osd_char16_row = 16'h03c0;
                        4'd12: F_osd_char16_row = 16'h03c0;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                CHAR_COLON: begin
                    case(I_row)
                        4'd6:  F_osd_char16_row = 16'h0180;
                        4'd7:  F_osd_char16_row = 16'h0180;
                        4'd10: F_osd_char16_row = 16'h0180;
                        4'd11: F_osd_char16_row = 16'h0180;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                default: F_osd_char16_row = 16'h0000;
            endcase
        end
    endfunction

    assign O_video_rd_en = I_video_de;

    //------------------------------------------------------------------------
    // 档位 -> 模式名称字模编号 (见 osd_cn_lib.v 编码表)
    //   0 直 1 通 2 模 3 式 | 4 灰 5 度 | 6 二 7 值 8 化 | 9 反 10 色
    //   11 伽 12 马 | 13 边 14 缘 | 15 卡 | 16 识 17 别
    //   编号 = 字符串里的第 I_i 个字; 超出该档位字数时返回最后一个字
    //------------------------------------------------------------------------
    function [4:0] F_mode_cid;
        input [2:0] I_m;
        input [2:0] I_i;
        begin
            case(I_m)
                // 直通模式
                3'd0:    F_mode_cid = (I_i == 3'd0) ? 5'd0  :
                                      (I_i == 3'd1) ? 5'd1  :
                                      (I_i == 3'd2) ? 5'd2  : 5'd3;
                // 灰度模式
                3'd1:    F_mode_cid = (I_i == 3'd0) ? 5'd4  :
                                      (I_i == 3'd1) ? 5'd5  :
                                      (I_i == 3'd2) ? 5'd2  : 5'd3;
                // 二值化模式
                3'd2:    F_mode_cid = (I_i == 3'd0) ? 5'd6  :
                                      (I_i == 3'd1) ? 5'd7  :
                                      (I_i == 3'd2) ? 5'd8  :
                                      (I_i == 3'd3) ? 5'd2  : 5'd3;
                // 反色模式
                3'd3:    F_mode_cid = (I_i == 3'd0) ? 5'd9  :
                                      (I_i == 3'd1) ? 5'd10 :
                                      (I_i == 3'd2) ? 5'd2  : 5'd3;
                // 伽马模式
                3'd4:    F_mode_cid = (I_i == 3'd0) ? 5'd11 :
                                      (I_i == 3'd1) ? 5'd12 :
                                      (I_i == 3'd2) ? 5'd2  : 5'd3;
                // 边缘模式
                3'd5:    F_mode_cid = (I_i == 3'd0) ? 5'd13 :
                                      (I_i == 3'd1) ? 5'd14 :
                                      (I_i == 3'd2) ? 5'd2  : 5'd3;
                // 卡通模式
                3'd6:    F_mode_cid = (I_i == 3'd0) ? 5'd15 :
                                      (I_i == 3'd1) ? 5'd1  :
                                      (I_i == 3'd2) ? 5'd2  : 5'd3;
                // 识别模式(7)
                default: F_mode_cid = (I_i == 3'd0) ? 5'd16 :
                                      (I_i == 3'd1) ? 5'd17 :
                                      (I_i == 3'd2) ? 5'd2  : 5'd3;
            endcase
        end
    endfunction

    // 模式文字: 只有"二值化模式"是 5 个字, 其余都是 4 个字; 统一右对齐到 x=1272
    wire [11:0] MODE_TXT_X = (I_img_mode == 3'd2) ? MODE_TXT_X5 : MODE_TXT_X4;
    wire [11:0] MODE_TXT_W = (I_img_mode == 3'd2) ? MODE_TXT_W5 : MODE_TXT_W4;

    anlogic_logo_rom u_anlogic_logo_rom(
        .I_addr  ( S_logo_addr  ),
        .O_pixel ( S_logo_pixel )
    );

    osd_char_lib u_osd_char_lib(
        .I_char     ( S_pf_char     ),
        .I_row      ( S_pf_row      ),
        .O_row_bits ( S_pf_row_bits )
    );

    //------------------------------------------------------------------------
    // 模式文字的字模行"预取"
    //   S_x_1d 在第 T-1 拍的值，正好等于 S_x_2d 在第 T 拍的值；所以用 x_1d/y_1d
    //   把 (第几个字, 字内行) 算出来、查好字模行、在第 T 拍锁存，第 T 拍只做
    //   "按列取 1 位 + 选颜色"。这样中文字模 ROM(综合后约 8 级)被移出像素关键路径。
    //
    //   !! 这四个 wire 必须声明在 u_osd_cn_lib 实例化之前 !!
    //   Verilog 在模块端口表里遇到未声明的标识符会按 1-bit wire 隐式声明，
    //   之后再写 wire[4:0] 只会报 HDL-7225 CRITICAL-WARNING 而不改端口位宽，
    //   结果 I_char/I_row 只接上 bit0、高位悬空 -> case 匹配不到任何字 ->
    //   字模全 0 -> 右上角一个字都画不出来。
    //------------------------------------------------------------------------
    wire [11:0] S_mt_lx_p  = S_x_1d - MODE_TXT_X;
    wire [11:0] S_mt_ly_p  = S_y_1d - MODE_TXT_Y;
    wire [4:0]  S_mt_cid_p = F_mode_cid(I_img_mode, S_mt_lx_p[7:5]);
    wire [3:0]  S_mt_row_p = S_mt_ly_p[4:1];

    wire [15:0] S_mt_row_bits;

    osd_cn_lib u_osd_cn_lib(
        .I_char     ( S_mt_cid_p     ),
        .I_row      ( S_mt_row_p     ),
        .O_row_bits ( S_mt_row_bits  )
    );

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_mt_bits_r <= 16'd0;
        else
            S_mt_bits_r <= S_mt_row_bits;
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_user) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_de) begin
            if(I_video_last) begin
                S_x <= 12'd0;
                S_y <= S_y + 12'd1;
            end
            else begin
                S_x <= S_x + 12'd1;
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_frame_cnt <= 32'd0;
            S_cnt_d0 <= 4'd0;
            S_cnt_d1 <= 4'd0;
            S_cnt_d2 <= 4'd0;
            S_cnt_d3 <= 4'd0;
            S_cnt_d4 <= 4'd0;
            S_cnt_d5 <= 4'd0;
        end
        else if(I_video_user) begin
            S_frame_cnt <= S_frame_cnt + 32'd1;
            if(S_cnt_d0 == 4'd9) begin
                S_cnt_d0 <= 4'd0;
                if(S_cnt_d1 == 4'd9) begin
                    S_cnt_d1 <= 4'd0;
                    if(S_cnt_d2 == 4'd9) begin
                        S_cnt_d2 <= 4'd0;
                        if(S_cnt_d3 == 4'd9) begin
                            S_cnt_d3 <= 4'd0;
                            if(S_cnt_d4 == 4'd9) begin
                                S_cnt_d4 <= 4'd0;
                                if(S_cnt_d5 == 4'd9)
                                    S_cnt_d5 <= 4'd0;
                                else
                                    S_cnt_d5 <= S_cnt_d5 + 4'd1;
                            end
                            else
                                S_cnt_d4 <= S_cnt_d4 + 4'd1;
                        end
                        else
                            S_cnt_d3 <= S_cnt_d3 + 4'd1;
                    end
                    else
                        S_cnt_d2 <= S_cnt_d2 + 4'd1;
                end
                else
                    S_cnt_d1 <= S_cnt_d1 + 4'd1;
            end
            else
                S_cnt_d0 <= S_cnt_d0 + 4'd1;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x_1d <= 12'd0;
            S_x_2d <= 12'd0;
            S_y_1d <= 12'd0;
            S_y_2d <= 12'd0;
        end
        else begin
            S_x_1d <= S_x;
            S_x_2d <= S_x_1d;
            S_y_1d <= S_y;
            S_y_2d <= S_y_1d;
        end
    end

    always @(*) begin
        S_osd_hit   = 1'b0;
        S_osd_color = 24'd0;
        S_logo_addr = 15'd0;

        S_logo_x = 12'd0;
        S_logo_y = 12'd0;
        S_char_idx = 4'd0;
        S_char_row = 3'd0;
        S_char_col = 3'd0;
        S_char_bits = 5'd0;
        S_char_on = 1'b0;
        S_logo_blue_on = 1'b0;
        S_logo_red_on = 1'b0;
        S_logo_tmp_y = 12'd0;
        S_logo_red_band = 4'd0;
        S_logo_red_x_start = 12'd0;
        S_logo_red_x_end = 12'd0;
        S_dyn_x = 12'd0;
        S_dyn_y = 12'd0;
        S_dyn_char = 4'd0;
        S_dyn_col = 3'd0;
        S_dyn_row = 3'd0;
        S_dyn_bits = 5'd0;
        S_dyn_on = 1'b0;
        S_dyn_digit_idx = 4'd0;
        S_dyn_digit_x = 12'd0;
        S_dyn_seg = 7'd0;
        S_lbl_row = 3'd0;
        S_lbl_col = 3'd0;
        S_lbl_idx = 2'd0;
        S_lbl_bits = 5'd0;
        S_lbl_local_x = 7'd0;
        S_dyn_dec_digit = 4'd0;
        S_cn_local_x = 5'd0;
        S_cn_local_y = 5'd0;
        S_cn_col = 4'd0;
        S_cn_row = 4'd0;
        S_cn_row_bits = 16'd0;
        S_cn_on = 1'b0;
        S_cn_scale_x = 10'd0;
        S_cn_scale_y = 10'd0;
        S_pf_char = 8'd0;
        S_pf_local_x = 5'd0;
        S_pf_local_y = 5'd0;
        S_pf_scale_x = 10'd0;
        S_pf_scale_y = 10'd0;
        S_pf_col = 4'd0;
        S_pf_row = 4'd0;
        S_pf_on = 1'b0;
        S_txt_hit = 1'b0;
        S_txt_sel = 1'b0;
        S_dyn_dupd = 1'b0;
        S_mt_lx = 8'd0;
        S_mt_col = 4'd0;
        S_mt_on = 1'b0;

        if(S_video_de_2d) begin
            //---- 左上角 LOGO(160x160)，最高优先级 ----
            if((S_x_2d >= LOGO_X) && (S_x_2d < (LOGO_X + LOGO_W)) &&
               (S_y_2d >= LOGO_Y) && (S_y_2d < (LOGO_Y + LOGO_H))) begin
                S_logo_x = S_x_2d - LOGO_X;
                S_logo_y = S_y_2d - LOGO_Y;
                S_logo_addr = S_logo_y * LOGO_W + S_logo_x;
                if(S_logo_pixel[24]) begin
                    S_osd_hit   = 1'b1;
                    S_osd_color = S_logo_pixel[23:0];
                end
            end

            //---- 右上角：直接显示当前档位名称(原来的 32x32 颜色方块已删除) ----
            //  16x16 中文字模 2 倍放大 -> 每字 32x32，右对齐到 x=1272
            //  字模行(S_mt_bits_r)已在前一拍用 x_1d/y_1d 查好并锁存，这里只做"取位"，
            //  避免"中文字模 ROM + 取位 + 颜色选择"串成一条 20 级的超长组合路径。
            if((!S_osd_hit) &&
               (S_y_2d >= MODE_TXT_Y) && (S_y_2d < (MODE_TXT_Y + MODE_TXT_H)) &&
               (S_x_2d >= MODE_TXT_X) && (S_x_2d < (MODE_TXT_X + MODE_TXT_W))) begin
                S_mt_lx  = S_x_2d - MODE_TXT_X;
                S_mt_col = S_mt_lx[4:1];      // 字内列 0..15
                case(S_mt_col)
                    4'd0:  S_mt_on = S_mt_bits_r[15];
                    4'd1:  S_mt_on = S_mt_bits_r[14];
                    4'd2:  S_mt_on = S_mt_bits_r[13];
                    4'd3:  S_mt_on = S_mt_bits_r[12];
                    4'd4:  S_mt_on = S_mt_bits_r[11];
                    4'd5:  S_mt_on = S_mt_bits_r[10];
                    4'd6:  S_mt_on = S_mt_bits_r[9];
                    4'd7:  S_mt_on = S_mt_bits_r[8];
                    4'd8:  S_mt_on = S_mt_bits_r[7];
                    4'd9:  S_mt_on = S_mt_bits_r[6];
                    4'd10: S_mt_on = S_mt_bits_r[5];
                    4'd11: S_mt_on = S_mt_bits_r[4];
                    4'd12: S_mt_on = S_mt_bits_r[3];
                    4'd13: S_mt_on = S_mt_bits_r[2];
                    4'd14: S_mt_on = S_mt_bits_r[1];
                    4'd15: S_mt_on = S_mt_bits_r[0];
                    default: S_mt_on = 1'b0;
                endcase
                if(S_mt_on) begin
                    S_osd_hit   = 1'b1;
                    S_osd_color = COLOR_DYN_TEXT;
                end
            end

            //---- LOGO 右侧：两行黄色状态文字，一个上一个下 ----
            //  第 1 行 安路CNT: 已累计显示的图像张数(6 位)
            //  第 2 行 安路FPS: 当前实时帧率(2 位)
            if((!S_osd_hit) &&
               (S_x_2d >= TXT_X) && (S_x_2d < (TXT_X + TXT_W))) begin
                if((S_y_2d >= TXT_Y1) && (S_y_2d < (TXT_Y1 + TXT_H))) begin
                    S_dyn_x   = S_x_2d - TXT_X;
                    S_dyn_y   = S_y_2d - TXT_Y1;
                    S_txt_sel = 1'b0;
                    S_txt_hit = 1'b1;
                end
                else if((S_y_2d >= TXT_Y2) && (S_y_2d < (TXT_Y2 + TXT_H))) begin
                    S_dyn_x   = S_x_2d - TXT_X;
                    S_dyn_y   = S_y_2d - TXT_Y2;
                    S_txt_sel = 1'b1;
                    S_txt_hit = 1'b1;
                end
            end

            if(S_txt_hit) begin
                // 前缀 "安" / "路"：16x16 字模按 24x24 显示(1.5 倍)
                if(S_dyn_x < 12'd48) begin
                    if(S_dyn_y >= CN_Y_OFFSET)
                        S_pf_local_y = S_dyn_y - CN_Y_OFFSET;
                    else
                        S_pf_local_y = 5'd31;

                    if(S_dyn_x < 12'd24) begin
                        S_pf_char    = CHAR_AN;
                        S_pf_local_x = S_dyn_x;
                    end
                    else begin
                        S_pf_char    = CHAR_LU;
                        S_pf_local_x = S_dyn_x - 12'd24;
                    end

                    if((S_pf_local_x < 5'd24) && (S_pf_local_y < 5'd24)) begin
                        S_pf_scale_x = S_pf_local_x * 10'd16;
                        S_pf_scale_y = S_pf_local_y * 10'd16;
                        S_pf_col = S_pf_scale_x / 10'd24;
                        S_pf_row = S_pf_scale_y / 10'd24;
                        case(S_pf_col)
                            4'd0:  S_pf_on = S_pf_row_bits[15];
                            4'd1:  S_pf_on = S_pf_row_bits[14];
                            4'd2:  S_pf_on = S_pf_row_bits[13];
                            4'd3:  S_pf_on = S_pf_row_bits[12];
                            4'd4:  S_pf_on = S_pf_row_bits[11];
                            4'd5:  S_pf_on = S_pf_row_bits[10];
                            4'd6:  S_pf_on = S_pf_row_bits[9];
                            4'd7:  S_pf_on = S_pf_row_bits[8];
                            4'd8:  S_pf_on = S_pf_row_bits[7];
                            4'd9:  S_pf_on = S_pf_row_bits[6];
                            4'd10: S_pf_on = S_pf_row_bits[5];
                            4'd11: S_pf_on = S_pf_row_bits[4];
                            4'd12: S_pf_on = S_pf_row_bits[3];
                            4'd13: S_pf_on = S_pf_row_bits[2];
                            4'd14: S_pf_on = S_pf_row_bits[1];
                            4'd15: S_pf_on = S_pf_row_bits[0];
                            default: S_pf_on = 1'b0;
                        endcase
                    end
                end
                // 三个字母：第 1 行 C/N/T(CNT)，第 2 行 F/P/S(FPS)，均为 16x16
                else if((S_dyn_x >= 12'd52) && (S_dyn_x < 12'd100) &&
                        (S_dyn_y >= 12'd4) && (S_dyn_y < 12'd20)) begin
                    if(S_dyn_x < 12'd68) begin
                        S_pf_char    = S_txt_sel ? CHAR_F : CHAR_C;
                        S_pf_local_x = S_dyn_x - 12'd52;
                    end
                    else if(S_dyn_x < 12'd84) begin
                        S_pf_char    = S_txt_sel ? CHAR_P : CHAR_N;
                        S_pf_local_x = S_dyn_x - 12'd68;
                    end
                    else begin
                        S_pf_char    = S_txt_sel ? CHAR_S : CHAR_T;
                        S_pf_local_x = S_dyn_x - 12'd84;
                    end
                    S_pf_local_y = S_dyn_y - 12'd4;
                    S_pf_col = S_pf_local_x[3:0];
                    S_pf_row = S_pf_local_y[3:0];
                    case(S_pf_col)
                        4'd0:  S_pf_on = S_pf_row_bits[15];
                        4'd1:  S_pf_on = S_pf_row_bits[14];
                        4'd2:  S_pf_on = S_pf_row_bits[13];
                        4'd3:  S_pf_on = S_pf_row_bits[12];
                        4'd4:  S_pf_on = S_pf_row_bits[11];
                        4'd5:  S_pf_on = S_pf_row_bits[10];
                        4'd6:  S_pf_on = S_pf_row_bits[9];
                        4'd7:  S_pf_on = S_pf_row_bits[8];
                        4'd8:  S_pf_on = S_pf_row_bits[7];
                        4'd9:  S_pf_on = S_pf_row_bits[6];
                        4'd10: S_pf_on = S_pf_row_bits[5];
                        4'd11: S_pf_on = S_pf_row_bits[4];
                        4'd12: S_pf_on = S_pf_row_bits[3];
                        4'd13: S_pf_on = S_pf_row_bits[2];
                        4'd14: S_pf_on = S_pf_row_bits[1];
                        4'd15: S_pf_on = S_pf_row_bits[0];
                        default: S_pf_on = 1'b0;
                    endcase
                end
                // 冒号 ":"：8x16
                else if((S_dyn_x >= 12'd100) && (S_dyn_x < 12'd108) &&
                        (S_dyn_y >= 12'd4) && (S_dyn_y < 12'd20)) begin
                    S_pf_char    = CHAR_COLON;
                    S_pf_local_x = S_dyn_x - 12'd100;
                    S_pf_local_y = S_dyn_y - 12'd4;
                    S_pf_scale_x = S_pf_local_x * 10'd16;
                    S_pf_col = S_pf_scale_x / 10'd8;
                    S_pf_row = S_pf_local_y[3:0];
                    case(S_pf_col)
                        4'd0:  S_pf_on = S_pf_row_bits[15];
                        4'd1:  S_pf_on = S_pf_row_bits[14];
                        4'd2:  S_pf_on = S_pf_row_bits[13];
                        4'd3:  S_pf_on = S_pf_row_bits[12];
                        4'd4:  S_pf_on = S_pf_row_bits[11];
                        4'd5:  S_pf_on = S_pf_row_bits[10];
                        4'd6:  S_pf_on = S_pf_row_bits[9];
                        4'd7:  S_pf_on = S_pf_row_bits[8];
                        4'd8:  S_pf_on = S_pf_row_bits[7];
                        4'd9:  S_pf_on = S_pf_row_bits[6];
                        4'd10: S_pf_on = S_pf_row_bits[5];
                        4'd11: S_pf_on = S_pf_row_bits[4];
                        4'd12: S_pf_on = S_pf_row_bits[3];
                        4'd13: S_pf_on = S_pf_row_bits[2];
                        4'd14: S_pf_on = S_pf_row_bits[1];
                        4'd15: S_pf_on = S_pf_row_bits[0];
                        default: S_pf_on = 1'b0;
                    endcase
                end

                // 数字：每字 15x24，首字从 x=110 起
                //  第 1 行 6 位(累计帧数) / 第 2 行 2 位(帧率，其余位不画)
                if((S_dyn_x >= 12'd110) && (S_dyn_x < 12'd200)) begin
                    if(S_dyn_x < 12'd125) begin
                        S_dyn_digit_x   = S_dyn_x - 12'd110;
                        S_dyn_dec_digit = S_txt_sel ? I_fps_tens : S_cnt_d5;
                        S_dyn_dupd      = 1'b1;
                    end
                    else if(S_dyn_x < 12'd140) begin
                        S_dyn_digit_x   = S_dyn_x - 12'd125;
                        S_dyn_dec_digit = S_txt_sel ? I_fps_ones : S_cnt_d4;
                        S_dyn_dupd      = 1'b1;
                    end
                    else if(S_dyn_x < 12'd155) begin
                        S_dyn_digit_x   = S_dyn_x - 12'd140;
                        S_dyn_dec_digit = S_cnt_d3;
                        S_dyn_dupd      = ~S_txt_sel;
                    end
                    else if(S_dyn_x < 12'd170) begin
                        S_dyn_digit_x   = S_dyn_x - 12'd155;
                        S_dyn_dec_digit = S_cnt_d2;
                        S_dyn_dupd      = ~S_txt_sel;
                    end
                    else if(S_dyn_x < 12'd185) begin
                        S_dyn_digit_x   = S_dyn_x - 12'd170;
                        S_dyn_dec_digit = S_cnt_d1;
                        S_dyn_dupd      = ~S_txt_sel;
                    end
                    else begin
                        S_dyn_digit_x   = S_dyn_x - 12'd185;
                        S_dyn_dec_digit = S_cnt_d0;
                        S_dyn_dupd      = ~S_txt_sel;
                    end

                    if((S_dyn_digit_x < 12'd15) && (S_dyn_y < 12'd24)) begin
                        S_pf_char    = CHAR_0 + {4'd0, S_dyn_dec_digit};
                        S_pf_local_x = S_dyn_digit_x[4:0];
                        S_pf_local_y = S_dyn_y[4:0];
                        S_pf_scale_x = S_pf_local_x * 10'd16;
                        S_pf_scale_y = S_pf_local_y * 10'd16;
                        S_pf_col = S_pf_scale_x / 10'd15;
                        S_pf_row = S_pf_scale_y / 10'd24;
                        if(S_dyn_dupd) begin
                            case(S_pf_col)
                                4'd0:  S_pf_on = S_pf_row_bits[15];
                                4'd1:  S_pf_on = S_pf_row_bits[14];
                                4'd2:  S_pf_on = S_pf_row_bits[13];
                                4'd3:  S_pf_on = S_pf_row_bits[12];
                                4'd4:  S_pf_on = S_pf_row_bits[11];
                                4'd5:  S_pf_on = S_pf_row_bits[10];
                                4'd6:  S_pf_on = S_pf_row_bits[9];
                                4'd7:  S_pf_on = S_pf_row_bits[8];
                                4'd8:  S_pf_on = S_pf_row_bits[7];
                                4'd9:  S_pf_on = S_pf_row_bits[6];
                                4'd10: S_pf_on = S_pf_row_bits[5];
                                4'd11: S_pf_on = S_pf_row_bits[4];
                                4'd12: S_pf_on = S_pf_row_bits[3];
                                4'd13: S_pf_on = S_pf_row_bits[2];
                                4'd14: S_pf_on = S_pf_row_bits[1];
                                4'd15: S_pf_on = S_pf_row_bits[0];
                                default: S_pf_on = 1'b0;
                            endcase
                        end
                    end
                end

                if(S_pf_on)
                    S_dyn_on = 1'b1;

                if(S_dyn_on) begin
                    S_osd_hit   = 1'b1;
                    S_osd_color = COLOR_DYN_TEXT;
                end
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_video_vsync_1d <= 1'b0;
            S_video_vsync_2d <= 1'b0;
            S_video_vsync_3d <= 1'b0;
            S_video_hsync_1d <= 1'b0;
            S_video_hsync_2d <= 1'b0;
            S_video_hsync_3d <= 1'b0;
            S_video_de_1d    <= 1'b0;
            S_video_de_2d    <= 1'b0;
            S_video_de_3d    <= 1'b0;
            O_hdmi_vsync     <= 1'b0;
            O_hdmi_hsync     <= 1'b0;
            O_hdmi_de        <= 1'b0;
            O_hdmi_data      <= 24'd0;
            S_video_data_3d  <= 24'd0;
            S_osd_hit_r      <= 1'b0;
            S_osd_color_r    <= 24'd0;
        end
        else begin
            S_video_vsync_1d <= I_video_vsync;
            S_video_vsync_2d <= S_video_vsync_1d;
            S_video_vsync_3d <= S_video_vsync_2d;
            O_hdmi_vsync     <= S_video_vsync_3d;

            S_video_hsync_1d <= I_video_hsync;
            S_video_hsync_2d <= S_video_hsync_1d;
            S_video_hsync_3d <= S_video_hsync_2d;
            O_hdmi_hsync     <= S_video_hsync_3d;

            S_video_de_1d    <= I_video_de;
            S_video_de_2d    <= S_video_de_1d;
            S_video_de_3d    <= S_video_de_2d;
            O_hdmi_de        <= S_video_de_3d;
            S_video_data_1d  <= I_video_rd_data;
            S_video_data_2d  <= S_video_data_1d;
            S_video_data_3d  <= S_video_data_2d;

            // 打一拍 OSD 命中/颜色, 切断 OSD 组合逻辑 -> 输出 mux 的长路径
            S_osd_hit_r      <= S_osd_hit;
            S_osd_color_r    <= S_osd_color;

            if(DEBUG_MODE == 1) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd128)
                        O_hdmi_data <= 24'hff0000;
                    else if(S_x < 12'd256)
                        O_hdmi_data <= 24'h00ff00;
                    else if(S_x < 12'd384)
                        O_hdmi_data <= 24'h0000ff;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= 24'hffff00;
                    else if(S_x < 12'd640)
                        O_hdmi_data <= 24'h00ffff;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= 24'hff00ff;
                    else if(S_x < 12'd896)
                        O_hdmi_data <= 24'hffffff;
                    else
                        O_hdmi_data <= 24'h202020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 2) begin
                if(S_video_de_2d) begin
                    case (I_debug_status)
                        4'b0000: O_hdmi_data <= 24'h000000;
                        4'b0001: O_hdmi_data <= 24'hff0000;
                        4'b0011: O_hdmi_data <= 24'hffff00;
                        4'b0111: O_hdmi_data <= 24'h00ff00;
                        4'b1111: O_hdmi_data <= I_video_rd_data;
                        default: O_hdmi_data <= 24'h0000ff;
                    endcase
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 3) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd256)
                        O_hdmi_data <= I_debug_status[0] ? 24'hff8000 : 24'h201000;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= I_debug_status[1] ? 24'hffff00 : 24'h202000;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= I_debug_status[2] ? 24'hff0000 : 24'h200000;
                    else
                        O_hdmi_data <= I_debug_status[3] ? 24'h00ffff : 24'h002020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else begin
                if(S_osd_hit_r)
                    O_hdmi_data <= S_osd_color_r;
                else
                    O_hdmi_data <= S_video_data_3d;
            end
        end
    end

endmodule


//============================================================================
// 下面追加 fps_meter：实时帧率计(1 秒窗口)，供本模块 OSD 显示用。
// 并入本文件的原因同上：TD 工程外部新增源文件会被丢弃。
//============================================================================
module fps_meter #(
    parameter integer CLK_HZ = 75000000      // I_clk 频率(Hz)，用于 1 秒时间基准
)(
    input  wire       I_clk,                 // 显示域像素时钟
    input  wire       I_rst_n,
    input  wire       I_frame_toggle,        // 相机域帧起始 toggle（每帧翻转）
    output reg  [3:0] O_tens,                // 帧率十位 0~9
    output reg  [3:0] O_ones                 // 帧率个位 0~9
);

    // 1 秒窗口的计数值（27bit 足够容纳 1.34e8）
    localparam [26:0] TICK_END = CLK_HZ - 1;

    //------------------------------------------------------------------------
    // 1) 帧 toggle 三级同步 + 双边沿检测
    //------------------------------------------------------------------------
    reg [2:0] S_tgl_sync;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            S_tgl_sync <= 3'd0;
        else
            S_tgl_sync <= {S_tgl_sync[1:0], I_frame_toggle};
    end

    wire S_frm_evt = S_tgl_sync[2] ^ S_tgl_sync[1];   // 每帧一个单周期脉冲

    //------------------------------------------------------------------------
    // 2) 帧计数 + 1 秒时间基准
    //------------------------------------------------------------------------
    reg [7:0]  S_frm_cnt;
    reg [26:0] S_tick_cnt;

    // 十进制拆分：cnt/10 用常数乘 205 后右移 11 位实现（对 0~255 严格成立），
    // 个位 = cnt - 十位*10 = cnt - (十位<<3) - (十位<<1)。无除法器、无 DSP。
    wire [7:0]  S_disp      = (S_frm_cnt > 8'd99) ? 8'd99 : S_frm_cnt;   // 满量程饱和
    wire [15:0] S_x205      = ({8'd0, S_disp} <<< 7) + ({8'd0, S_disp} <<< 6) +
                              ({8'd0, S_disp} <<< 3) + ({8'd0, S_disp} <<< 2) +
                               {8'd0, S_disp};
    wire [7:0]  S_dec_tens  = S_x205[15:11];
    wire [7:0]  S_dec_ones  = S_disp - (S_dec_tens <<< 3) - (S_dec_tens <<< 1);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_frm_cnt  <= 8'd0;
            S_tick_cnt <= 27'd0;
            O_tens     <= 4'd0;
            O_ones     <= 4'd0;
        end
        else begin
            if (S_tick_cnt == TICK_END) begin
                // 窗口结束：锁存本次统计结果，重新开始计数（本拍若正好来帧则计入新窗口）
                S_tick_cnt <= 27'd0;
                S_frm_cnt  <= S_frm_evt ? 8'd1 : 8'd0;
                O_tens     <= S_dec_tens[3:0];
                O_ones     <= S_dec_ones[3:0];
            end
            else begin
                S_tick_cnt <= S_tick_cnt + 27'd1;
                if (S_frm_evt)
                    S_frm_cnt <= (S_frm_cnt == 8'hFF) ? 8'hFF : S_frm_cnt + 8'd1;
            end
        end
    end

endmodule
