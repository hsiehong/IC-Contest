module geofence (clk, reset, X, Y, R, valid, is_inside);
input clk;
input reset;
input [9:0] X;
input [9:0] Y;
input [10:0] R;
output reg valid;
output is_inside;

parameter ST_INIT   = 3'b000,
          ST_READ   = 3'b001,
          ST_SORT   = 3'b010,
          ST_FAREA  = 3'b011,
          ST_CALSIDE= 3'b100,
          ST_OAREA  = 3'b101,
          ST_OUTPUT = 3'b110;

reg [2:0] curr_state, next_state;
reg signed [10:0] x_in [0:6];
reg signed [10:0] y_in [0:6];
reg [10:0] r_in [0:6];
reg [3:0] counter;              // 0-15
reg [2:0] s_idx;
reg [2:0] o_idx;
reg [9:0] sidelen [0:6];
reg signed [10:0] x_vec [0:4];   // 1 bit for signed
reg signed [10:0] y_vec [0:4];
wire signed [20:0] mul_out1;      // outer product
wire signed [20:0] mul_out2;      // outer product
reg signed [11:0] mul1, mul2, mul3, mul4;
reg signed [24:0] fence_area;
reg [24:0]  obj_area;
reg [19:0] sqrt_in;
wire [9:0] sqrt_out;
reg [10:0] temp1, temp2;
reg sort, read;
reg [11:0] s;           // (a+b+c)/2
wire sort_done;

assign mul_out1 = mul1 * mul2;
assign mul_out2 = mul3 * mul4;
assign sort_done = (s_idx == 3'h4);
assign is_inside = ((fence_area >> 1) >= obj_area);

DW_sqrt #(20,0) Sqrt(.a(sqrt_in), .root(sqrt_out));

// NEXT STATE LOGIC
always @(*) begin
    case (curr_state)
        ST_INIT: next_state = (!reset) ? ST_READ : ST_INIT;
        ST_READ: next_state = (counter == 4'h5) ? ST_SORT : ST_READ;
        ST_SORT: next_state = (sort_done) ? ST_FAREA : ST_SORT;
        ST_FAREA: next_state = (counter == 4'h5) ? ST_CALSIDE : ST_FAREA;
        ST_CALSIDE: next_state = (counter == 4'h5) ? ST_OAREA : ST_CALSIDE;
        ST_OAREA: next_state = (counter == 4'h6) ? ST_OUTPUT : ST_OAREA;
        ST_OUTPUT: next_state = ST_READ;
        default: next_state = ST_INIT;
    endcase
end

// STATE REGISTER
always @(posedge clk or posedge reset) begin
    if (reset)
        curr_state  <= ST_INIT;
    else
        curr_state  <= next_state;
end

//OUTPUT LOGIC
always @(*) begin
    case(curr_state)
        ST_INIT: begin
            valid = 1'b0; sort = 1'b0;
            read = (!reset) ? 1'b1 : 1'b0;
        end
        ST_READ: begin
            read = 1'b1; valid = 1'b0; sort = 1'b0;
        end
        ST_SORT: begin
            read = 1'b0; valid = 1'b0; sort = 1'b1;
        end
        ST_FAREA: begin
            read = 1'b0; valid = 1'b0; sort = 1'b0;
        end
        ST_CALSIDE: begin
            read = 1'b0; valid = 1'b0; sort = 1'b0;
        end
        ST_OAREA: begin
            read = 1'b0; valid = 1'b0; sort = 1'b0;
        end
        ST_OUTPUT: begin
            read = 1'b0; valid = 1'b1; sort = 1'b0;
        end
        default: begin
            read = 1'b0; valid = 1'b0; sort = 1'b0;
        end
    endcase
end

// COUNTER
always @(posedge clk or posedge reset) begin
    if (reset) begin
        counter     <= 4'h0;
        o_idx       <= 0;
    end
    else begin
        case (curr_state)
            ST_INIT: begin
                counter     <= (reset) ? 4'h0 : counter + 4'b1;
            end
            ST_READ: begin
                counter     <= counter + 4'b1;
            end
            ST_SORT: begin
                counter     <= 4'h0;
            end
            ST_FAREA: begin
                counter     <= (counter == 4'h5) ? 4'h0 : (counter + 4'b1);
            end
            ST_CALSIDE: begin
                counter     <= (counter == 4'h5) ? 4'h0 : (counter + 4'b1);
            end
            ST_OAREA: begin
                counter     <= (o_idx == 3'h3) ? (counter + 4'b1) : counter;
                o_idx       <= (o_idx == 3'h3) ? 3'b0 : (o_idx + 3'b1);
            end
            ST_OUTPUT: begin
                counter     <= 4'h0;
                o_idx       <= 3'h0;
            end
        endcase
    end
end

// READ & SORT
always @(posedge clk) begin
    if (read)begin
        x_in[counter]   <= X;
        y_in[counter]   <= Y;
        r_in[counter]   <= R;
    end
    else if (sort) begin
        x_in[6]     <= x_in[0];
        y_in[6]     <= y_in[0];
        r_in[6]     <= r_in[0];
        if (mul_out1 < mul_out2 ) begin
            // SWAP point
            s_idx                  <= 3'b0;
            x_in[s_idx + 3'h1]     <= x_in[s_idx + 3'h2];
            y_in[s_idx + 3'h1]     <= y_in[s_idx + 3'h2];
            r_in[s_idx + 3'h1]     <= r_in[s_idx + 3'h2];
            x_in[s_idx + 3'h2]     <= x_in[s_idx + 3'h1];
            y_in[s_idx + 3'h2]     <= y_in[s_idx + 3'h1];
            r_in[s_idx + 3'h2]     <= r_in[s_idx + 3'h1];
        end
        else begin
            s_idx   <= s_idx + 3'b1;
        end
    end
    else begin
        s_idx   <= 3'b0;
    end
end

// AREA, SIDE
always @(posedge clk)begin
    case(curr_state)
        ST_READ: begin
            fence_area  <= 0;
            obj_area    <= 0;
        end
        ST_FAREA: begin
            fence_area  <= fence_area + (mul_out1 - mul_out2);    
        end
        ST_CALSIDE: begin
            sidelen[counter]    <= sqrt_out;
            sidelen[6]          <= sidelen[0];
        end
        ST_OAREA: begin
            case (o_idx) 
                3'h0: begin
                    temp1   <= sqrt_out;
                end
                3'h1: begin
                    temp2   <= sqrt_out;
                end
                3'h2: begin
                    obj_area    <= obj_area + mul_out1;
                end
            endcase
        end
    endcase
end

// MULTIPLE 
always @(*) begin
    case (curr_state)
        ST_SORT: begin
           case(s_idx)
               3'h0: begin
                   mul1 = x_vec[0]; mul2 = y_vec[1];
                   mul3 = x_vec[1]; mul4 = y_vec[0];
               end
               3'h1: begin
                   mul1 = x_vec[1]; mul2 = y_vec[2];
                   mul3 = x_vec[2]; mul4 = y_vec[1];
               end
               3'h2: begin
                   mul1 = x_vec[2]; mul2 = y_vec[3];
                   mul3 = x_vec[3]; mul4 = y_vec[2];
               end
               3'h3: begin
                   mul1 = x_vec[3]; mul2 = y_vec[4];
                   mul3 = x_vec[4]; mul4 = y_vec[3];
               end
               default: begin
                   mul1 = 0; mul2 = 0; mul3 = 0; mul4 = 0;
               end
           endcase
        end
        ST_FAREA: begin
           case(counter)
               4'h0: begin
                   mul1 = x_in[0]; mul2 = y_in[1];
                   mul3 = x_in[1]; mul4 = y_in[0];
               end
               4'h1: begin
                   mul1 = x_in[1]; mul2 = y_in[2];
                   mul3 = x_in[2]; mul4 = y_in[1];
               end
               4'h2: begin
                   mul1 = x_in[2]; mul2 = y_in[3];
                   mul3 = x_in[3]; mul4 = y_in[2];
               end
               4'h3: begin
                   mul1 = x_in[3]; mul2 = y_in[4];
                   mul3 = x_in[4]; mul4 = y_in[3];
               end
               4'h4: begin
                   mul1 = x_in[4]; mul2 = y_in[5];
                   mul3 = x_in[5]; mul4 = y_in[4];
               end
               4'h5: begin
                   mul1 = x_in[5]; mul2 = y_in[0];
                   mul3 = x_in[0]; mul4 = y_in[5];
               end
               default: begin
                   mul1 = 0; mul2 = 0; mul3 = 0; mul4 = 0;
               end
           endcase
        end
        ST_CALSIDE: begin
            case (counter)
                4'h0: begin
                    mul1 = (x_in[0] - x_in[1]); mul2 = (x_in[0] - x_in[1]);
                    mul3 = (y_in[0] - y_in[1]); mul4 = (y_in[0] - y_in[1]);
                end
                4'h1: begin
                    mul1 = (x_in[1] - x_in[2]); mul2 = (x_in[1] - x_in[2]);
                    mul3 = (y_in[1] - y_in[2]); mul4 = (y_in[1] - y_in[2]);
                end
                4'h2: begin
                    mul1 = (x_in[2] - x_in[3]); mul2 = (x_in[2] - x_in[3]);
                    mul3 = (y_in[2] - y_in[3]); mul4 = (y_in[2] - y_in[3]);
                end
                4'h3: begin
                    mul1 = (x_in[3] - x_in[4]); mul2 = (x_in[3] - x_in[4]);
                    mul3 = (y_in[3] - y_in[4]); mul4 = (y_in[3] - y_in[4]);
                end
                4'h4: begin
                    mul1 = (x_in[4] - x_in[5]); mul2 = (x_in[4] - x_in[5]);
                    mul3 = (y_in[4] - y_in[5]); mul4 = (y_in[4] - y_in[5]);
                end
                4'h5: begin
                    mul1 = (x_in[5] - x_in[0]); mul2 = (x_in[5] - x_in[0]);
                    mul3 = (y_in[5] - y_in[0]); mul4 = (y_in[5] - y_in[0]);
                end
                default: begin
                    mul1 = 0; mul2 = 0; mul3 = 0; mul4 = 0;
                end
            endcase
        end
        ST_OAREA: begin
            case (o_idx)
                3'h0: begin
                    mul1 = s;
                    mul2 = (s > sidelen[counter]) ? (s - sidelen[counter]) : (sidelen[counter] - s);
                    mul3 = 0; mul4 = 0;
                end
                3'h1: begin
                    mul1 = 0; mul2 = 0;
                    mul3 = (s > r_in[counter]) ? (s - r_in[counter]) : (r_in[counter] - s);
                    mul4 = (s > r_in[counter + 1]) ? (s - r_in[counter + 1]) : (r_in[counter + 1] - s);
                end
                3'h2: begin
                    mul1 = temp1;
                    mul2 = temp2;
                    mul3 = 0; mul4 = 0;
                end
                default: begin
                    mul1 = 0; mul2 = 0; mul3 = 0; mul4 = 0;
                end
            endcase
        end
        default: begin
            mul1 = 0; mul2 = 0; mul3 = 0; mul4 = 0;
        end
    endcase
end

// S
always@(*) begin
    case (counter)
        4'h0: s = (sidelen[0] + r_in[0] + r_in[1]) >> 1;
        4'h1: s = (sidelen[1] + r_in[1] + r_in[2]) >> 1;
        4'h2: s = (sidelen[2] + r_in[2] + r_in[3]) >> 1;
        4'h3: s = (sidelen[3] + r_in[3] + r_in[4]) >> 1;
        4'h4: s = (sidelen[4] + r_in[4] + r_in[5]) >> 1;
        4'h5: s = (sidelen[5] + r_in[5] + r_in[0]) >> 1;
        default: s = 0;
    endcase
end

// SQRT
always @(*) begin
    case (curr_state)
        ST_CALSIDE: begin
            sqrt_in = mul_out1[19:0] + mul_out2[19:0];
        end
        ST_OAREA: begin
            case (o_idx)
                3'h0: sqrt_in = mul_out1[19:0];
                3'h1: sqrt_in = mul_out2[19:0];
                default: sqrt_in = 20'b0;
            endcase
        end
        default: 
            sqrt_in = 20'b0;
    endcase
end

// CAL VECTOR
always @(*) begin
    x_vec[0] = x_in[1] - x_in[0]; y_vec[0] = y_in[1] - y_in[0];
    x_vec[1] = x_in[2] - x_in[0]; y_vec[1] = y_in[2] - y_in[0];
    x_vec[2] = x_in[3] - x_in[0]; y_vec[2] = y_in[3] - y_in[0];
    x_vec[3] = x_in[4] - x_in[0]; y_vec[3] = y_in[4] - y_in[0];
    x_vec[4] = x_in[5] - x_in[0]; y_vec[4] = y_in[5] - y_in[0];
end


endmodule   // end of GEOFENCE
