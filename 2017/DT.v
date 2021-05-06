module DT(
	input 			    clk, 
	input			    reset,
	output  		    done ,
	output  		    sti_rd ,
	output	    [9:0]	sti_addr ,
	input		[15:0]	sti_di,
	output  		    res_wr ,
	output  		    res_rd ,
	output	    [13:0]	res_addr ,
	output	    [7:0]	res_do,
	input		[7:0]	res_di
);
wire [13:0] count16384;
wire rdres_done, isobj;
wire [3:0] state;

CTRL ctrl(
    .clk(clk),
    .rst(reset),
    .sti_rd(sti_rd),
    .res_rd(res_rd),
    .res_wr(res_wr),
    .done(done),
    .curr_state(state),
    .count16384(count16384),
    .rdres_done(rdres_done),
    .isobj(isobj)
);
DATAPATH dpath(
    .clk(clk),
    .rst(reset),
    .state(state),
    .sti_di(sti_di),
    .count16384(count16384),
    .rdres_done(rdres_done),
    .isobj(isobj),
    .res_di(res_di),
    .res_do(res_do),
    .res_addr(res_addr),
    .sti_addr(sti_addr)
);

endmodule   // END OF DT

//*******************************************************
//********************* CONTROL *************************
//*******************************************************
module CTRL (
    clk, rst, isobj,
    sti_rd, res_wr, res_rd, done, rdres_done,
    curr_state, count16384
);

input clk, rst, isobj;
output reg sti_rd, res_wr, res_rd, done;
output reg [3:0] curr_state;
input rdres_done;

parameter ST_INIT       = 4'd0,
          ST_FWRDPX     = 4'd1,
          ST_FWRDRES    = 4'd2,
          ST_FWWRT      = 4'd3,
          ST_FW2BW      = 4'd4,
          ST_BWRDPX     = 4'd5,
          ST_BWRDRES    = 4'd6,
          ST_BWWRT      = 4'd7,
          ST_DONE       = 4'd8;

reg [3:0] next_state;
input [13:0]  count16384;         // count 0 - 16383
// reg sti_rd_reg, res_rd_reg, done_reg, res_wr_reg;
wire fw_done,  bw_done, isobj;

assign fw_done = (count16384 == 14'd16383);
assign bw_done = (count16384 == 14'd0);

// NEXT STATE LOGIC
// {{{
always @(*) begin
    case (curr_state)
        ST_INIT:
            next_state = (sti_rd) ? ST_FWRDPX : ST_INIT;
        ST_FWRDPX:
            next_state = ST_FWRDRES;
        ST_FWRDRES: begin
            if (isobj) begin
                if (rdres_done)
                    next_state = ST_FWWRT;
                else
                    next_state = ST_FWRDRES;
            end
            else
                next_state = ST_FWWRT;
        end
        ST_FWWRT:
            next_state = (fw_done) ? ST_FW2BW : ST_FWRDPX;
        ST_FW2BW:
            next_state = ST_BWRDPX;
        ST_BWRDPX: begin
            next_state = ST_BWRDRES;
        end
        ST_BWRDRES: begin
            if (isobj) begin
                if (rdres_done)
                    next_state = ST_BWWRT;
                else
                    next_state = ST_BWRDRES;
            end
            else
                next_state = ST_BWWRT;
        end
        ST_BWWRT:
            next_state = (bw_done) ? ST_DONE : ST_BWRDPX;
            // next_state = ST_DONE;
        ST_DONE:
            next_state = ST_DONE;
        default:
            next_state = ST_INIT;
    endcase
end
// }}}

// STATE REGISTER
always @(posedge clk or negedge rst) begin
    if (!rst)
        curr_state  <= ST_INIT;
    else
        curr_state  <= next_state;
end

// OUTPUT LOGIC
// {{{
always @(*) begin
    case (curr_state)
        ST_INIT: begin
            sti_rd  = (rst) ? 1'b1 : 1'b0;
            res_rd  = 1'b0;
            res_wr  = 1'b0;
            done    = 1'b0;
        end
        ST_FWRDPX: begin
            sti_rd  = 1'b1;
            res_rd  = 1'b0;
            res_wr  = 1'b0;
            done    = 1'b0;
        end
        ST_FWRDRES: begin
            sti_rd  = 1'b0;
            res_rd  = 1'b1;
            res_wr  = 1'b0;
            done    = 1'b0;
        end
        ST_FWWRT: begin
            sti_rd  = 1'b0;
            res_rd  = 1'b0;
            res_wr  = 1'b1;
            done    = 1'b0;
        end
        ST_FW2BW: begin
            sti_rd  = 1'b0;
            res_rd  = 1'b0;
            res_wr  = 1'b0;
            done    = 1'b0;
        end
        ST_BWRDPX: begin
            sti_rd  = 1'b0;
            res_rd  = 1'b1;
            res_wr  = 1'b0;
            done    = 1'b0;
        end
        ST_BWRDRES: begin
            sti_rd  = 1'b0;
            res_rd  = 1'b1;
            res_wr  = 1'b0;
            done    = 1'b0;
        end
        ST_BWWRT: begin
            sti_rd  = 1'b0;
            res_rd  = 1'b0;
            res_wr  = 1'b1;
            done    = 1'b0;
        end
        ST_DONE: begin
            sti_rd  = 1'b0;
            res_rd  = 1'b0;
            res_wr  = 1'b0;
            done    = 1'b1;
        end
        default: begin
            sti_rd  = 1'b0;
            res_rd  = 1'b0;
            res_wr  = 1'b0;
            done    = 1'b0;
        end
    endcase
end
endmodule

//*******************************************************
//********************* DATAPATH ************************
//*******************************************************
module DATAPATH (
    clk, rst, 
    state,
    sti_di,
    count16384, rdres_done,
    isobj,
    res_di, res_do, res_addr, sti_addr
);

parameter ST_INIT       = 4'd0,
          ST_FWRDPX     = 4'd1,
          ST_FWRDRES    = 4'd2,
          ST_FWWRT      = 4'd3,
          ST_FW2BW      = 4'd4,
          ST_BWRDPX     = 4'd5,
          ST_BWRDRES    = 4'd6,
          ST_BWWRT      = 4'd7,
          ST_DONE       = 4'd8;

input clk, rst;
input [3:0] state;
input [15:0] sti_di;
input [7:0] res_di;

output reg [13:0] count16384;   
output reg isobj;
output reg [7:0] res_do;
output reg [13:0] res_addr;
output rdres_done;

output [9:0] sti_addr;


wire [3:0] px_pos;

wire [13:0] rdres_addr [0:4];
wire [13:0] rdres_addr_b [0:4];

reg [7:0] data4 [0:3];
reg [7:0] fwres, temp1, temp2, temp3, temp4, res_din;
reg [3:0] count4;

reg top_out, left_out, right_out, bot_out;
wire outrange, outrange2;

assign rdres_done = (count4 == 4'd3);

assign rdres_addr[0] = count16384 - 14'd129,
       rdres_addr[1] = count16384 - 14'd128,
       rdres_addr[2] = count16384 - 14'd127,
       rdres_addr[3] = count16384 - 14'd1;
assign rdres_addr_b[0] = count16384 + 14'd1,
       rdres_addr_b[1] = count16384 + 14'd127,
       rdres_addr_b[2] = count16384 + 14'd128,
       rdres_addr_b[3] = count16384 + 14'd129;

assign outrange = (top_out | left_out),
       outrange2 = (right_out | bot_out);

assign sti_addr = count16384[13:4],
       px_pos  = (4'b1111 - count16384[3:0]);
   
// border detect
always @(*) begin
    if (state == 4'd2) begin
        top_out = (rdres_addr[count4] > count16384);
        left_out = (count16384[7:0] == 8'b0 && rdres_addr[count4][6:0] == 7'h7f);
        right_out = 4'd0;
        bot_out = 4'd0;
    end
    else if (state == 4'd6) begin
        right_out = (count16384[6:0] == 7'h7f && rdres_addr_b[count4][7:0] == 8'b0);
        bot_out = (rdres_addr_b[count4] < count16384);
        left_out = 4'd0;
        top_out = 4'd0;
    end
    else begin
        left_out = 4'd0; right_out = 4'd0;
        top_out = 4'd0; bot_out = 4'd0;
    end
end

// cal forward res
always @(*) begin
    case (state)
        ST_FWRDRES: begin
            temp1 = (data4[0] < data4[1]) ? data4[0] : data4[1];
            temp2 = (data4[2] < data4[3]) ? data4[2] : data4[3];
            temp3 = (temp1 < temp2) ? temp1 : temp2;
            temp4 = 8'b0;
            fwres = (isobj) ? (temp3 + 8'b1) : 8'b0;
        end
        ST_FWWRT: begin
            temp1 = (data4[0] < data4[1]) ? data4[0] : data4[1];
            temp2 = (data4[2] < data4[3]) ? data4[2] : data4[3];
            temp3 = (temp1 < temp2) ? temp1 : temp2;
            temp4 = 8'b0;
            fwres = (isobj) ? (temp3 + 8'b1) : 8'b0;
        end
        ST_BWRDRES: begin
            temp1 = (data4[0] < data4[1]) ? data4[0] : data4[1];
            temp2 = (data4[2] < data4[3]) ? data4[2] : data4[3];
            temp3 = (temp1 < temp2) ? temp1 : temp2;
            temp4 = (temp3 + 8'b1 < res_din) ? (temp3 + 8'b1) : res_din;
            fwres = (isobj) ? temp4 : 8'b0;
        end 
        ST_BWWRT: begin
            temp1 = (data4[0] < data4[1]) ? data4[0] : data4[1];
            temp2 = (data4[2] < data4[3]) ? data4[2] : data4[3];
            temp3 = (temp1 < temp2) ? temp1 : temp2;
            temp4 = ((temp3 + 8'b1) < res_din) ? (temp3 + 8'b1) : res_din;
            fwres = (isobj) ? temp4 : 8'b0;
        end 
        default: begin
            temp1 = 8'b0; temp2 = 8'b0;
            temp3 = 8'b0; temp4 = 8'b0;
            fwres = 8'b0;
        end
    endcase
end

// COUNTER
always @(posedge clk or negedge rst) begin
    if (!rst) begin
        count16384  <= 14'b0;
        count4      <= 4'b0;
    end
    else begin
        case (state) 
            ST_INIT: begin
                count4          <= 4'b0;
                count16384      <= 14'b0;
            end
            ST_FWRDPX: begin
                count4          <= 4'b0;
            end
            ST_FWRDRES: begin
                count4          <= count4 + 4'b1;
            end
            ST_FWWRT: begin
               count16384       <= count16384 + 14'b1;
               count4           <= 4'b0;
            end
            ST_FW2BW: begin
                count16384      <= 14'd16383;
            end
            ST_BWRDPX : begin
                count4          <= 4'b0;
            end
            ST_BWRDRES: begin
                count4          <= count4 + 4'b1;
            end
            ST_BWWRT: begin
                count16384      <= count16384 - 14'b1;
                count4          <= 4'b0;
            end
            
        endcase
    end
end

// DATA, ADDR
always @(posedge clk or negedge rst) begin 
    if (!rst) begin
        isobj       <= 1'b0;
    end
    else begin
        case (state) 
            ST_INIT: begin
                isobj           <= 1'b0;            
            end
            ST_FWRDPX: begin
                isobj           <= sti_di[px_pos];        
                res_addr        <= (sti_di[px_pos]) ? rdres_addr[0] : res_addr;
            end
            ST_FWRDRES: begin
                res_addr        <= (count4 == 4'd3 || !(isobj)) ? count16384 : rdres_addr[count4 + 1];  // setting addr wrt back to mem
                data4[count4]   <= (outrange) ? 8'hff : res_di;
            end
            ST_FWWRT: begin
                isobj           <= 1'b0;
            end
            ST_FW2BW: begin
                res_addr        <= 14'd16383;
            end
            ST_BWRDPX: begin
                isobj           <= (res_di) ? 1'b1 : 1'b0;
                res_din         <= res_di;
                res_addr        <= (res_di) ? rdres_addr_b[0] : res_addr;
            end
            ST_BWRDRES: begin
                res_addr        <= (count4 == 4'd3 || !(isobj)) ? count16384 : rdres_addr_b[count4 + 1];    // setting addr wrt back to mem
                data4[count4]   <= (outrange2) ? 8'hff : res_di;
            end
            ST_BWWRT: begin
                isobj           <= 1'b0;
                res_addr        <= count16384 - 14'b1;
            end
        endcase
    end
end

always @(*) begin
    if (state == ST_FWWRT || state == ST_BWWRT)
        res_do = (isobj) ? fwres : 8'b0;
    else
        res_do = 8'b0;
end
endmodule
