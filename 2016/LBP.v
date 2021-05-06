
`timescale 1ns/10ps
module LBP ( clk, reset, gray_addr, gray_req, gray_ready, gray_data, lbp_addr, lbp_valid, lbp_data, finish);
input   	clk;
input   	reset;
output  [13:0] 	gray_addr;
output         	gray_req;
input   	gray_ready;
input   [7:0] 	gray_data;
output  [13:0] 	lbp_addr;
output  	lbp_valid;
output  [7:0] 	lbp_data;
output  	finish;
//====================================================================

wire [1:0] state;
wire [13:0] index;
wire read_done;

assign lbp_addr = index;

Control CTRL(
    .clk(clk),
    .reset(reset),
    .curr_state(state),
    .global_index(index),
    .gray_req(gray_req),
    .gray_ready(gray_ready),
    .lbp_valid(lbp_valid),
    .finish(finish),
    .read_done(read_done)
);
Datapath DPATH(
    .clk(clk),
    .reset(reset),
    .state(state),
    .result(lbp_data),
    .global_idx(index),
    .gray_addr(gray_addr),
    .gray_data(gray_data),
    .read_done(read_done)
);
//module Datapath(clk, reset, state, result, global_idx, gray_addr, gray_data);

//====================================================================
endmodule

// **************************** CONTROL MODULE *********************
module Control(clk, reset, curr_state, global_index, gray_req, gray_ready, lbp_valid, finish, read_done);

input clk, reset, gray_ready;
input read_done;

output reg [1:0] curr_state;
output reg gray_req, lbp_valid;
output finish;

output reg [13:0] global_index; // reading index
reg [6:0] row_count;// 126 value per row, 0-125
reg [13:0] wrt_count;
reg [1:0] next_state;


assign finish = (wrt_count == 14'd15876) ? 1 : 0; // total write 15876 result

parameter ST_INIT = 2'b00,
          ST_RDOP = 2'b01,
          ST_WRT  = 2'b10,
          ST_DONE = 2'b11;

always @(*) begin
    case (curr_state)
        ST_INIT: 
            next_state = (gray_ready) ? ST_RDOP : ST_INIT;
        ST_RDOP:
            next_state = (read_done) ? ST_WRT : ST_RDOP;
        ST_WRT:
            next_state = (finish) ? ST_DONE : ST_RDOP;
        ST_DONE:
            next_state = ST_DONE;
    endcase
end

always @(posedge clk or posedge reset) begin
    if (reset)
        curr_state <= ST_INIT;
    else
        curr_state <= next_state;
end

always @(*) begin
    case (curr_state) 
        ST_INIT: begin
            {gray_req, lbp_valid} = 2'b00;
        end
        ST_RDOP: begin
            {gray_req, lbp_valid} = 2'b10;
        end
        ST_WRT: begin
            {gray_req, lbp_valid} = 2'b11;
        end
        ST_DONE: begin
            {gray_req, lbp_valid} = 2'b10;
        end
        default: begin
            {gray_req, lbp_valid} = 2'b10;
        end
    endcase
end

// counter setting
always @(posedge clk or posedge reset) begin
    if (reset) begin
        global_index <= 14'd129;
        row_count <= 7'd0;
        wrt_count <= 14'd0;
    end
    else begin
        if (curr_state == ST_WRT) begin
            global_index <= (row_count == 14'd125) ? global_index + 14'd3 : global_index + 14'd1;
            row_count <= (row_count == 7'd125) ? 7'd0: row_count + 7'd1;
            wrt_count <= wrt_count + 14'd1;
        end
    end
end

endmodule
// **************************** CONTROL END *********************


// **************************** DATAPATH *********************
module Datapath(clk, reset, state, result, global_idx, gray_addr, gray_data, read_done);
input clk, reset;
input [1:0] state;
input [13:0] global_idx;
output [13:0] gray_addr;
input [7:0] gray_data;
output reg [7:0] result;
output read_done;

reg [3:0] counter; // 0-8, only use in reading state
reg [7:0] mid_data;

wire [13:0] gv [0:8];   // read address each round
assign gv[0] = global_idx;
assign gv[1] = global_idx - 14'd129;
assign gv[2] = global_idx - 14'd128;
assign gv[3] = global_idx - 14'd127;
assign gv[4] = global_idx - 14'd1;
assign gv[5] = global_idx + 14'd1;
assign gv[6] = global_idx + 14'd127;
assign gv[7] = global_idx + 14'd128;
assign gv[8] = global_idx + 14'd129;
assign gray_addr = gv[counter];
/*
always @(posedge clk, posedge reset) begin 
    if(reset)
        gray_addr <= ;
    else
        gray_addr <= gv[counter];
end
*/
assign read_done = (counter == 4'd8) ? 1'b1 : 1'b0;

// counter set
always @(posedge clk or posedge reset) begin
    if (reset) begin
        counter <= 4'd0; 
    end
    else begin
        if (state == 2'b01) // read state
            counter <= (counter == 4'd8) ? 4'd0 : counter + 4'd1;
    end
end

// result
always @(posedge clk or posedge reset) begin
    if (reset)
        result <= 8'b0;
    else begin
        if (counter == 0)
            mid_data <= gray_data;
        else
            result[counter - 1] <= (gray_data >= mid_data) ? 1'b1 : 1'b0;
    end
end

endmodule
// **************************** DATAPATH END *********************

