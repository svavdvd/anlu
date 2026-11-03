module osd_char_lib (
    input wire [7:0] I_char,
    input wire [3:0] I_row,
    output reg [15:0] O_row_bits
);

    localparam CHAR_AN    = 8'h80;
    localparam CHAR_LU    = 8'h81;
    localparam CHAR_A     = 8'h41;
    localparam CHAR_B     = 8'h42;
    localparam CHAR_C     = 8'h43;
    localparam CHAR_D     = 8'h44;
    localparam CHAR_E     = 8'h45;
    localparam CHAR_F     = 8'h46;
    localparam CHAR_G     = 8'h47;
    localparam CHAR_H     = 8'h48;
    localparam CHAR_I     = 8'h49;
    localparam CHAR_J     = 8'h4a;
    localparam CHAR_K     = 8'h4b;
    localparam CHAR_L     = 8'h4c;
    localparam CHAR_M     = 8'h4d;
    localparam CHAR_N     = 8'h4e;
    localparam CHAR_O     = 8'h4f;
    localparam CHAR_P     = 8'h50;
    localparam CHAR_Q     = 8'h51;
    localparam CHAR_R     = 8'h52;
    localparam CHAR_S     = 8'h53;
    localparam CHAR_T     = 8'h54;
    localparam CHAR_U     = 8'h55;
    localparam CHAR_V     = 8'h56;
    localparam CHAR_W     = 8'h57;
    localparam CHAR_X     = 8'h58;
    localparam CHAR_Y     = 8'h59;
    localparam CHAR_Z     = 8'h5a;
    localparam CHAR_0     = 8'h30;
    localparam CHAR_1     = 8'h31;
    localparam CHAR_2     = 8'h32;
    localparam CHAR_3     = 8'h33;
    localparam CHAR_4     = 8'h34;
    localparam CHAR_5     = 8'h35;
    localparam CHAR_6     = 8'h36;
    localparam CHAR_7     = 8'h37;
    localparam CHAR_8     = 8'h38;
    localparam CHAR_9     = 8'h39;
    localparam CHAR_COLON = 8'h3a;

    always @(*) begin
        case(I_char)
            CHAR_AN: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0200;
                    4'd1: O_row_bits = 16'h7ff0;
                    4'd2: O_row_bits = 16'h4410;
                    4'd3: O_row_bits = 16'h0400;
                    4'd4: O_row_bits = 16'h7ff0;
                    4'd5: O_row_bits = 16'h0840;
                    4'd6: O_row_bits = 16'h1040;
                    4'd7: O_row_bits = 16'h1c80;
                    4'd8: O_row_bits = 16'h0300;
                    4'd9: O_row_bits = 16'h0cc0;
                    4'd10: O_row_bits = 16'h7020;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_LU: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h79e0;
                    4'd1: O_row_bits = 16'h4a20;
                    4'd2: O_row_bits = 16'h4d40;
                    4'd3: O_row_bits = 16'h7880;
                    4'd4: O_row_bits = 16'h1140;
                    4'd5: O_row_bits = 16'h1630;
                    4'd6: O_row_bits = 16'h5be0;
                    4'd7: O_row_bits = 16'h5220;
                    4'd8: O_row_bits = 16'h5220;
                    4'd9: O_row_bits = 16'h5be0;
                    4'd10: O_row_bits = 16'h6220;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_A: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h1c00;
                    4'd3: O_row_bits = 16'h1400;
                    4'd4: O_row_bits = 16'h1400;
                    4'd5: O_row_bits = 16'h1400;
                    4'd6: O_row_bits = 16'h3600;
                    4'd7: O_row_bits = 16'h3e00;
                    4'd8: O_row_bits = 16'h2200;
                    4'd9: O_row_bits = 16'h2200;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_B: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h3300;
                    4'd4: O_row_bits = 16'h3300;
                    4'd5: O_row_bits = 16'h3e00;
                    4'd6: O_row_bits = 16'h3300;
                    4'd7: O_row_bits = 16'h3300;
                    4'd8: O_row_bits = 16'h3300;
                    4'd9: O_row_bits = 16'h3e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_C: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h03f0;
                    4'd3: O_row_bits = 16'h0c0c;
                    4'd4: O_row_bits = 16'h1800;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h3000;
                    4'd7: O_row_bits = 16'h3000;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3000;
                    4'd10: O_row_bits = 16'h1800;
                    4'd11: O_row_bits = 16'h0c0c;
                    4'd12: O_row_bits = 16'h03f0;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_D: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h3300;
                    4'd4: O_row_bits = 16'h3300;
                    4'd5: O_row_bits = 16'h3300;
                    4'd6: O_row_bits = 16'h3300;
                    4'd7: O_row_bits = 16'h3300;
                    4'd8: O_row_bits = 16'h3600;
                    4'd9: O_row_bits = 16'h3c00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_E: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3e00;
                    4'd6: O_row_bits = 16'h3000;
                    4'd7: O_row_bits = 16'h3000;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_F: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h3e00;
                    4'd7: O_row_bits = 16'h3000;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3000;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_G: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h1c00;
                    4'd3: O_row_bits = 16'h3200;
                    4'd4: O_row_bits = 16'h6000;
                    4'd5: O_row_bits = 16'h6000;
                    4'd6: O_row_bits = 16'h6e00;
                    4'd7: O_row_bits = 16'h6600;
                    4'd8: O_row_bits = 16'h3600;
                    4'd9: O_row_bits = 16'h1e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_H: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3300;
                    4'd3: O_row_bits = 16'h3300;
                    4'd4: O_row_bits = 16'h3300;
                    4'd5: O_row_bits = 16'h3f00;
                    4'd6: O_row_bits = 16'h3300;
                    4'd7: O_row_bits = 16'h3300;
                    4'd8: O_row_bits = 16'h3300;
                    4'd9: O_row_bits = 16'h3300;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_I: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h1e00;
                    4'd3: O_row_bits = 16'h0c00;
                    4'd4: O_row_bits = 16'h0c00;
                    4'd5: O_row_bits = 16'h0c00;
                    4'd6: O_row_bits = 16'h0c00;
                    4'd7: O_row_bits = 16'h0c00;
                    4'd8: O_row_bits = 16'h0c00;
                    4'd9: O_row_bits = 16'h1e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_J: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h0600;
                    4'd4: O_row_bits = 16'h0600;
                    4'd5: O_row_bits = 16'h0600;
                    4'd6: O_row_bits = 16'h0600;
                    4'd7: O_row_bits = 16'h0600;
                    4'd8: O_row_bits = 16'h2600;
                    4'd9: O_row_bits = 16'h1c00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_K: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3300;
                    4'd3: O_row_bits = 16'h3600;
                    4'd4: O_row_bits = 16'h3400;
                    4'd5: O_row_bits = 16'h3c00;
                    4'd6: O_row_bits = 16'h3c00;
                    4'd7: O_row_bits = 16'h3400;
                    4'd8: O_row_bits = 16'h3600;
                    4'd9: O_row_bits = 16'h3300;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_L: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3000;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h3000;
                    4'd7: O_row_bits = 16'h3000;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_M: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3600;
                    4'd3: O_row_bits = 16'h3600;
                    4'd4: O_row_bits = 16'h3600;
                    4'd5: O_row_bits = 16'h3600;
                    4'd6: O_row_bits = 16'h2a00;
                    4'd7: O_row_bits = 16'h2a00;
                    4'd8: O_row_bits = 16'h2200;
                    4'd9: O_row_bits = 16'h2200;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_N: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h300c;
                    4'd3: O_row_bits = 16'h380c;
                    4'd4: O_row_bits = 16'h3c0c;
                    4'd5: O_row_bits = 16'h360c;
                    4'd6: O_row_bits = 16'h330c;
                    4'd7: O_row_bits = 16'h318c;
                    4'd8: O_row_bits = 16'h30cc;
                    4'd9: O_row_bits = 16'h306c;
                    4'd10: O_row_bits = 16'h303c;
                    4'd11: O_row_bits = 16'h301c;
                    4'd12: O_row_bits = 16'h300c;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_O: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h1e00;
                    4'd3: O_row_bits = 16'h3200;
                    4'd4: O_row_bits = 16'h6300;
                    4'd5: O_row_bits = 16'h6300;
                    4'd6: O_row_bits = 16'h6300;
                    4'd7: O_row_bits = 16'h6300;
                    4'd8: O_row_bits = 16'h3600;
                    4'd9: O_row_bits = 16'h3c00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_P: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h3300;
                    4'd4: O_row_bits = 16'h3300;
                    4'd5: O_row_bits = 16'h3300;
                    4'd6: O_row_bits = 16'h3e00;
                    4'd7: O_row_bits = 16'h3000;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3000;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_Q: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h1e00;
                    4'd3: O_row_bits = 16'h3200;
                    4'd4: O_row_bits = 16'h6300;
                    4'd5: O_row_bits = 16'h6300;
                    4'd6: O_row_bits = 16'h6300;
                    4'd7: O_row_bits = 16'h6300;
                    4'd8: O_row_bits = 16'h7600;
                    4'd9: O_row_bits = 16'h3c00;
                    4'd10: O_row_bits = 16'h0c00;
                    4'd11: O_row_bits = 16'h0700;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_R: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h3300;
                    4'd4: O_row_bits = 16'h3300;
                    4'd5: O_row_bits = 16'h3300;
                    4'd6: O_row_bits = 16'h3c00;
                    4'd7: O_row_bits = 16'h3600;
                    4'd8: O_row_bits = 16'h3600;
                    4'd9: O_row_bits = 16'h3200;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_S: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h1e00;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3c00;
                    4'd6: O_row_bits = 16'h0f00;
                    4'd7: O_row_bits = 16'h0300;
                    4'd8: O_row_bits = 16'h0300;
                    4'd9: O_row_bits = 16'h3e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_T: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3ffc;
                    4'd3: O_row_bits = 16'h03c0;
                    4'd4: O_row_bits = 16'h03c0;
                    4'd5: O_row_bits = 16'h03c0;
                    4'd6: O_row_bits = 16'h03c0;
                    4'd7: O_row_bits = 16'h03c0;
                    4'd8: O_row_bits = 16'h03c0;
                    4'd9: O_row_bits = 16'h03c0;
                    4'd10: O_row_bits = 16'h03c0;
                    4'd11: O_row_bits = 16'h03c0;
                    4'd12: O_row_bits = 16'h03c0;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_U: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3300;
                    4'd3: O_row_bits = 16'h3300;
                    4'd4: O_row_bits = 16'h3300;
                    4'd5: O_row_bits = 16'h3300;
                    4'd6: O_row_bits = 16'h3300;
                    4'd7: O_row_bits = 16'h3300;
                    4'd8: O_row_bits = 16'h3300;
                    4'd9: O_row_bits = 16'h1e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_V: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h2200;
                    4'd3: O_row_bits = 16'h2200;
                    4'd4: O_row_bits = 16'h3600;
                    4'd5: O_row_bits = 16'h3600;
                    4'd6: O_row_bits = 16'h1400;
                    4'd7: O_row_bits = 16'h1400;
                    4'd8: O_row_bits = 16'h1c00;
                    4'd9: O_row_bits = 16'h1c00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_W: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h2200;
                    4'd3: O_row_bits = 16'h2200;
                    4'd4: O_row_bits = 16'h2a00;
                    4'd5: O_row_bits = 16'h2a00;
                    4'd6: O_row_bits = 16'h3600;
                    4'd7: O_row_bits = 16'h3600;
                    4'd8: O_row_bits = 16'h3600;
                    4'd9: O_row_bits = 16'h3600;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_X: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h1c00;
                    4'd4: O_row_bits = 16'h1c00;
                    4'd5: O_row_bits = 16'h0800;
                    4'd6: O_row_bits = 16'h1c00;
                    4'd7: O_row_bits = 16'h1400;
                    4'd8: O_row_bits = 16'h3600;
                    4'd9: O_row_bits = 16'h3200;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_Y: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3180;
                    4'd3: O_row_bits = 16'h1300;
                    4'd4: O_row_bits = 16'h1b00;
                    4'd5: O_row_bits = 16'h1e00;
                    4'd6: O_row_bits = 16'h0c00;
                    4'd7: O_row_bits = 16'h0c00;
                    4'd8: O_row_bits = 16'h0c00;
                    4'd9: O_row_bits = 16'h0c00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_Z: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h0400;
                    4'd4: O_row_bits = 16'h0c00;
                    4'd5: O_row_bits = 16'h0800;
                    4'd6: O_row_bits = 16'h0800;
                    4'd7: O_row_bits = 16'h1800;
                    4'd8: O_row_bits = 16'h1000;
                    4'd9: O_row_bits = 16'h3e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_0: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h0000;
                    4'd7: O_row_bits = 16'h0000;
                    4'd8: O_row_bits = 16'h300c;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h300c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_1: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h000c;
                    4'd2: O_row_bits = 16'h000c;
                    4'd3: O_row_bits = 16'h000c;
                    4'd4: O_row_bits = 16'h000c;
                    4'd5: O_row_bits = 16'h000c;
                    4'd6: O_row_bits = 16'h0000;
                    4'd7: O_row_bits = 16'h0000;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h000c;
                    4'd12: O_row_bits = 16'h000c;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_2: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h000c;
                    4'd4: O_row_bits = 16'h000c;
                    4'd5: O_row_bits = 16'h000c;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3000;
                    4'd10: O_row_bits = 16'h3000;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_3: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h000c;
                    4'd4: O_row_bits = 16'h000c;
                    4'd5: O_row_bits = 16'h000c;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_4: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h300c;
                    4'd2: O_row_bits = 16'h300c;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h000c;
                    4'd12: O_row_bits = 16'h000c;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_5: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_6: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h300c;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h300c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_7: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h000c;
                    4'd4: O_row_bits = 16'h000c;
                    4'd5: O_row_bits = 16'h000c;
                    4'd6: O_row_bits = 16'h0000;
                    4'd7: O_row_bits = 16'h0000;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h000c;
                    4'd12: O_row_bits = 16'h000c;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_8: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h300c;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h300c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_9: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_COLON: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h0000;
                    4'd3: O_row_bits = 16'h0000;
                    4'd4: O_row_bits = 16'h0000;
                    4'd5: O_row_bits = 16'h0000;
                    4'd6: O_row_bits = 16'h0180;
                    4'd7: O_row_bits = 16'h0180;
                    4'd8: O_row_bits = 16'h0000;
                    4'd9: O_row_bits = 16'h0000;
                    4'd10: O_row_bits = 16'h0180;
                    4'd11: O_row_bits = 16'h0180;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            default: O_row_bits = 16'h0000;
        endcase
    end

endmodule


//============================================================================
// 下面追加 osd_cn_lib：模式名称用的 16x16 中文字模(原计划单独成文件，
// TD 工程文件清单由 GUI 维护、外部新增文件会被丢弃，故并入本文件)。
//============================================================================
module osd_cn_lib (
    input  wire [4:0]  I_char,
    input  wire [3:0]  I_row,
    output reg  [15:0] O_row_bits
);

    always @(*) begin
        case (I_char)
            5'd0: begin   // 直  [直通]
                case (I_row)
                    4'd0:  O_row_bits = 16'h0000;
                    4'd1:  O_row_bits = 16'h0180;
                    4'd2:  O_row_bits = 16'h0180;
                    4'd3:  O_row_bits = 16'h3ffc;
                    4'd4:  O_row_bits = 16'h0180;
                    4'd5:  O_row_bits = 16'h0180;
                    4'd6:  O_row_bits = 16'h0ff0;
                    4'd7:  O_row_bits = 16'h0c10;
                    4'd8:  O_row_bits = 16'h0ff0;
                    4'd9:  O_row_bits = 16'h0c10;
                    4'd10: O_row_bits = 16'h0ff0;
                    4'd11: O_row_bits = 16'h0c10;
                    4'd12: O_row_bits = 16'h0ff0;
                    4'd13: O_row_bits = 16'h0c10;
                    4'd14: O_row_bits = 16'h7ffe;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd1: begin   // 通  [直通]
                case (I_row)
                    4'd0:  O_row_bits = 16'h0000;
                    4'd1:  O_row_bits = 16'h2400;
                    4'd2:  O_row_bits = 16'h37f8;
                    4'd3:  O_row_bits = 16'h18f0;
                    4'd4:  O_row_bits = 16'h00e0;
                    4'd5:  O_row_bits = 16'h07fc;
                    4'd6:  O_row_bits = 16'h766c;
                    4'd7:  O_row_bits = 16'h17fc;
                    4'd8:  O_row_bits = 16'h166c;
                    4'd9:  O_row_bits = 16'h17fc;
                    4'd10: O_row_bits = 16'h164c;
                    4'd11: O_row_bits = 16'h167c;
                    4'd12: O_row_bits = 16'h3e58;
                    4'd13: O_row_bits = 16'h7f0e;
                    4'd14: O_row_bits = 16'h23fe;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd2: begin   // 模  [模式]
                case (I_row)
                    4'd0:  O_row_bits = 16'h0000;
                    4'd1:  O_row_bits = 16'h18d8;
                    4'd2:  O_row_bits = 16'h18d8;
                    4'd3:  O_row_bits = 16'h13fe;
                    4'd4:  O_row_bits = 16'h18d8;
                    4'd5:  O_row_bits = 16'h7dfc;
                    4'd6:  O_row_bits = 16'h390c;
                    4'd7:  O_row_bits = 16'h39fc;
                    4'd8:  O_row_bits = 16'h3f0c;
                    4'd9:  O_row_bits = 16'h79fc;
                    4'd10: O_row_bits = 16'hd060;
                    4'd11: O_row_bits = 16'h57fe;
                    4'd12: O_row_bits = 16'h1070;
                    4'd13: O_row_bits = 16'h11d8;
                    4'd14: O_row_bits = 16'h1f8e;
                    4'd15: O_row_bits = 16'h1206;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd3: begin   // 式  [模式]
                case (I_row)
                    4'd0:  O_row_bits = 16'h0000;
                    4'd1:  O_row_bits = 16'h00d0;
                    4'd2:  O_row_bits = 16'h0058;
                    4'd3:  O_row_bits = 16'h004c;
                    4'd4:  O_row_bits = 16'h0048;
                    4'd5:  O_row_bits = 16'h7ffe;
                    4'd6:  O_row_bits = 16'h0060;
                    4'd7:  O_row_bits = 16'h3fe0;
                    4'd8:  O_row_bits = 16'h0460;
                    4'd9:  O_row_bits = 16'h0460;
                    4'd10: O_row_bits = 16'h0430;
                    4'd11: O_row_bits = 16'h0470;
                    4'd12: O_row_bits = 16'h07da;
                    4'd13: O_row_bits = 16'h7f1e;
                    4'd14: O_row_bits = 16'h200e;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd4: begin   // 灰  [灰度]
                case (I_row)
                    4'd0:  O_row_bits = 16'h0000;
                    4'd1:  O_row_bits = 16'h0400;
                    4'd2:  O_row_bits = 16'h0400;
                    4'd3:  O_row_bits = 16'h7ffe;
                    4'd4:  O_row_bits = 16'h0460;
                    4'd5:  O_row_bits = 16'h0c60;
                    4'd6:  O_row_bits = 16'h0d6c;
                    4'd7:  O_row_bits = 16'h0f6c;
                    4'd8:  O_row_bits = 16'h0b78;
                    4'd9:  O_row_bits = 16'h1a78;
                    4'd10: O_row_bits = 16'h18e0;
                    4'd11: O_row_bits = 16'h31b0;
                    4'd12: O_row_bits = 16'h639c;
                    4'd13: O_row_bits = 16'h6e0e;
                    4'd14: O_row_bits = 16'h0406;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd5: begin   // 度  [灰度]
                case (I_row)
                    4'd0:  O_row_bits = 16'h0080;
                    4'd1:  O_row_bits = 16'h00c0;
                    4'd2:  O_row_bits = 16'h3ffe;
                    4'd3:  O_row_bits = 16'h3110;
                    4'd4:  O_row_bits = 16'h3318;
                    4'd5:  O_row_bits = 16'h3ffe;
                    4'd6:  O_row_bits = 16'h3110;
                    4'd7:  O_row_bits = 16'h31f0;
                    4'd8:  O_row_bits = 16'h3000;
                    4'd9:  O_row_bits = 16'h37f8;
                    4'd10: O_row_bits = 16'h3130;
                    4'd11: O_row_bits = 16'h21e0;
                    4'd12: O_row_bits = 16'h61f0;
                    4'd13: O_row_bits = 16'hcf3e;
                    4'd14: O_row_bits = 16'h0c06;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd6: begin   // 二  [二值化]
                case (I_row)
                    4'd0:  O_row_bits = 16'h0000;
                    4'd1:  O_row_bits = 16'h0000;
                    4'd2:  O_row_bits = 16'h0000;
                    4'd3:  O_row_bits = 16'h3ffc;
                    4'd4:  O_row_bits = 16'h0000;
                    4'd5:  O_row_bits = 16'h0000;
                    4'd6:  O_row_bits = 16'h0000;
                    4'd7:  O_row_bits = 16'h0000;
                    4'd8:  O_row_bits = 16'h0000;
                    4'd9:  O_row_bits = 16'h0000;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h7ffe;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd7: begin   // 值  [二值化]
                case (I_row)
                    4'd0:  O_row_bits = 16'h1060;
                    4'd1:  O_row_bits = 16'h1860;
                    4'd2:  O_row_bits = 16'h1ffe;
                    4'd3:  O_row_bits = 16'h3040;
                    4'd4:  O_row_bits = 16'h33f8;
                    4'd5:  O_row_bits = 16'h7208;
                    4'd6:  O_row_bits = 16'hf308;
                    4'd7:  O_row_bits = 16'h73f8;
                    4'd8:  O_row_bits = 16'h3308;
                    4'd9:  O_row_bits = 16'h33f8;
                    4'd10: O_row_bits = 16'h3308;
                    4'd11: O_row_bits = 16'h33f8;
                    4'd12: O_row_bits = 16'h330c;
                    4'd13: O_row_bits = 16'h3ffe;
                    4'd14: O_row_bits = 16'h3000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd8: begin   // 化  [二值化]
                case (I_row)
                    4'd0:  O_row_bits = 16'h04c0;
                    4'd1:  O_row_bits = 16'h0cc0;
                    4'd2:  O_row_bits = 16'h0cc0;
                    4'd3:  O_row_bits = 16'h18c4;
                    4'd4:  O_row_bits = 16'h18cc;
                    4'd5:  O_row_bits = 16'h38d8;
                    4'd6:  O_row_bits = 16'h78f0;
                    4'd7:  O_row_bits = 16'h78f0;
                    4'd8:  O_row_bits = 16'h18e0;
                    4'd9:  O_row_bits = 16'h1bc0;
                    4'd10: O_row_bits = 16'h1fc0;
                    4'd11: O_row_bits = 16'h1ac0;
                    4'd12: O_row_bits = 16'h18c6;
                    4'd13: O_row_bits = 16'h18c6;
                    4'd14: O_row_bits = 16'h187c;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd9: begin   // 反  [反色]
                case (I_row)
                    4'd0:  O_row_bits = 16'h0000;
                    4'd1:  O_row_bits = 16'h0078;
                    4'd2:  O_row_bits = 16'h1ffc;
                    4'd3:  O_row_bits = 16'h1800;
                    4'd4:  O_row_bits = 16'h1800;
                    4'd5:  O_row_bits = 16'h1800;
                    4'd6:  O_row_bits = 16'h1ff8;
                    4'd7:  O_row_bits = 16'h1e18;
                    4'd8:  O_row_bits = 16'h1b10;
                    4'd9:  O_row_bits = 16'h11b0;
                    4'd10: O_row_bits = 16'h31e0;
                    4'd11: O_row_bits = 16'h30e0;
                    4'd12: O_row_bits = 16'h33b8;
                    4'd13: O_row_bits = 16'h7e1e;
                    4'd14: O_row_bits = 16'h4804;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd10: begin   // 色  [反色]
                case (I_row)
                    4'd0:  O_row_bits = 16'h0000;
                    4'd1:  O_row_bits = 16'h0600;
                    4'd2:  O_row_bits = 16'h0600;
                    4'd3:  O_row_bits = 16'h0ff0;
                    4'd4:  O_row_bits = 16'h1860;
                    4'd5:  O_row_bits = 16'h70e0;
                    4'd6:  O_row_bits = 16'h7ff8;
                    4'd7:  O_row_bits = 16'h3118;
                    4'd8:  O_row_bits = 16'h3198;
                    4'd9:  O_row_bits = 16'h3ff8;
                    4'd10: O_row_bits = 16'h3000;
                    4'd11: O_row_bits = 16'h3004;
                    4'd12: O_row_bits = 16'h3006;
                    4'd13: O_row_bits = 16'h380c;
                    4'd14: O_row_bits = 16'h1ff8;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd11: begin   // 伽  [伽马]
                case (I_row)
                    4'd0:  O_row_bits = 16'h0000;
                    4'd1:  O_row_bits = 16'h0b00;
                    4'd2:  O_row_bits = 16'h1b00;
                    4'd3:  O_row_bits = 16'h1b7e;
                    4'd4:  O_row_bits = 16'h3ff6;
                    4'd5:  O_row_bits = 16'h33f6;
                    4'd6:  O_row_bits = 16'h73f6;
                    4'd7:  O_row_bits = 16'h73f6;
                    4'd8:  O_row_bits = 16'h13f6;
                    4'd9:  O_row_bits = 16'h12f6;
                    4'd10: O_row_bits = 16'h12f6;
                    4'd11: O_row_bits = 16'h16f6;
                    4'd12: O_row_bits = 16'h16fe;
                    4'd13: O_row_bits = 16'h1db6;
                    4'd14: O_row_bits = 16'h1810;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd12: begin   // 马  [伽马]
                case (I_row)
                    4'd0:  O_row_bits = 16'h0000;
                    4'd1:  O_row_bits = 16'h0000;
                    4'd2:  O_row_bits = 16'h3ff0;
                    4'd3:  O_row_bits = 16'h0c30;
                    4'd4:  O_row_bits = 16'h0c30;
                    4'd5:  O_row_bits = 16'h0c30;
                    4'd6:  O_row_bits = 16'h0830;
                    4'd7:  O_row_bits = 16'h0830;
                    4'd8:  O_row_bits = 16'h0ffe;
                    4'd9:  O_row_bits = 16'h0004;
                    4'd10: O_row_bits = 16'h0004;
                    4'd11: O_row_bits = 16'h7ffc;
                    4'd12: O_row_bits = 16'h000c;
                    4'd13: O_row_bits = 16'h003c;
                    4'd14: O_row_bits = 16'h0038;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd13: begin   // 边  [边缘]
                case (I_row)
                    4'd0:  O_row_bits = 16'h0000;
                    4'd1:  O_row_bits = 16'h00c0;
                    4'd2:  O_row_bits = 16'h20c0;
                    4'd3:  O_row_bits = 16'h30c0;
                    4'd4:  O_row_bits = 16'h18c0;
                    4'd5:  O_row_bits = 16'h03fc;
                    4'd6:  O_row_bits = 16'h00cc;
                    4'd7:  O_row_bits = 16'h70cc;
                    4'd8:  O_row_bits = 16'h10cc;
                    4'd9:  O_row_bits = 16'h108c;
                    4'd10: O_row_bits = 16'h118c;
                    4'd11: O_row_bits = 16'h130c;
                    4'd12: O_row_bits = 16'h1e38;
                    4'd13: O_row_bits = 16'h6f3e;
                    4'd14: O_row_bits = 16'h43fe;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd14: begin   // 缘  [边缘]
                case (I_row)
                    4'd0:  O_row_bits = 16'h00c0;
                    4'd1:  O_row_bits = 16'h18c0;
                    4'd2:  O_row_bits = 16'h30fc;
                    4'd3:  O_row_bits = 16'h2588;
                    4'd4:  O_row_bits = 16'h6ff8;
                    4'd5:  O_row_bits = 16'hfbfe;
                    4'd6:  O_row_bits = 16'h18c4;
                    4'd7:  O_row_bits = 16'h33ec;
                    4'd8:  O_row_bits = 16'h7bf8;
                    4'd9:  O_row_bits = 16'h79f8;
                    4'd10: O_row_bits = 16'h477c;
                    4'd11: O_row_bits = 16'h1df6;
                    4'd12: O_row_bits = 16'h7fb7;
                    4'd13: O_row_bits = 16'h62e0;
                    4'd14: O_row_bits = 16'h00e0;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd15: begin   // 卡  [卡通]
                case (I_row)
                    4'd0:  O_row_bits = 16'h0000;
                    4'd1:  O_row_bits = 16'h0300;
                    4'd2:  O_row_bits = 16'h0300;
                    4'd3:  O_row_bits = 16'h03f8;
                    4'd4:  O_row_bits = 16'h0300;
                    4'd5:  O_row_bits = 16'h0300;
                    4'd6:  O_row_bits = 16'h0300;
                    4'd7:  O_row_bits = 16'h7ffe;
                    4'd8:  O_row_bits = 16'h0300;
                    4'd9:  O_row_bits = 16'h0300;
                    4'd10: O_row_bits = 16'h03c0;
                    4'd11: O_row_bits = 16'h0370;
                    4'd12: O_row_bits = 16'h0338;
                    4'd13: O_row_bits = 16'h0300;
                    4'd14: O_row_bits = 16'h0300;
                    4'd15: O_row_bits = 16'h0300;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd16: begin   // 识  [识别]
                case (I_row)
                    4'd0:  O_row_bits = 16'h0000;
                    4'd1:  O_row_bits = 16'h2000;
                    4'd2:  O_row_bits = 16'h39fc;
                    4'd3:  O_row_bits = 16'h1d0c;
                    4'd4:  O_row_bits = 16'h010c;
                    4'd5:  O_row_bits = 16'h010c;
                    4'd6:  O_row_bits = 16'hf98c;
                    4'd7:  O_row_bits = 16'h19fc;
                    4'd8:  O_row_bits = 16'h1000;
                    4'd9:  O_row_bits = 16'h1090;
                    4'd10: O_row_bits = 16'h14d8;
                    4'd11: O_row_bits = 16'h1d8c;
                    4'd12: O_row_bits = 16'h1f06;
                    4'd13: O_row_bits = 16'h1f02;
                    4'd14: O_row_bits = 16'h1600;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            5'd17: begin   // 别  [识别]
                case (I_row)
                    4'd0:  O_row_bits = 16'h0000;
                    4'd1:  O_row_bits = 16'h0006;
                    4'd2:  O_row_bits = 16'h3f86;
                    4'd3:  O_row_bits = 16'h31b6;
                    4'd4:  O_row_bits = 16'h31b6;
                    4'd5:  O_row_bits = 16'h3fb6;
                    4'd6:  O_row_bits = 16'h0c36;
                    4'd7:  O_row_bits = 16'h0c36;
                    4'd8:  O_row_bits = 16'h7fb6;
                    4'd9:  O_row_bits = 16'h0cb6;
                    4'd10: O_row_bits = 16'h0cb6;
                    4'd11: O_row_bits = 16'h0986;
                    4'd12: O_row_bits = 16'h1986;
                    4'd13: O_row_bits = 16'h3786;
                    4'd14: O_row_bits = 16'h771c;
                    4'd15: O_row_bits = 16'h2018;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            default: O_row_bits = 16'h0000;
        endcase
    end

endmodule
