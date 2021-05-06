
`timescale 1ns/10ps

module  CONV(
	input		        clk,
	input		        reset,
	output reg 	        busy,	
	input		        ready,	
			
	output     [11:0]   iaddr,
	input      [19:0]   idata,	
	
	output reg 	        cwr,
	output reg [11:0]   caddr_wr,
	output reg [19:0]   cdata_wr,
	
	output reg 	        crd,
	output reg [11:0]   caddr_rd,
	input      [19:0]   cdata_rd,
	
	output reg [2:0]    csel
);

parameter ST_INIT = 3'b000,
          ST_RDOP = 3'b001,
          ST_OUT1 = 3'b010,
          ST_POOL = 3'b011,
          ST_OUT2 = 3'b100,
          ST_FLAT = 3'b101,
          ST_OUT3 = 3'b110,
          ST_DONE = 3'b111;

reg [2:0] curr_state, next_state;
wire readone, pooldone, flatdone;
reg outkernel;       // 0 for output result of kernel 0, 1 for output kernel 1

wire [59:0] px012, px345, px678;
wire [3:0] count9;   // count 0-8, send to CAL module
wire [12:0] count4096;  // control write index in RDOP state
wire [11:0] count1024;  // control write index in POOL state
wire [11:0] count2048;
wire [19:0] poolres;
wire [19:0] flatres;
wire [11:0] rdaddr_pool;    // control read index in POOL state, count 0, 2, ... 4030
wire [11:0] rdaddr_flat;
wire [19:0] conv0_res, conv1_res;


// ***************************//
// *****NEXT STATE LOGIC******//
// ***************************//
// {{{
always @(*) begin
    case (curr_state) 
        ST_INIT: 
            next_state = (ready) ? ST_RDOP : ST_INIT;
        ST_RDOP: 
            next_state = (readone) ? ST_OUT1 : ST_RDOP;
        ST_OUT1: begin
            if (count4096 == 13'd4096 & outkernel == 1'b1)
                next_state = ST_POOL;
            else if (outkernel == 1'b1)
                next_state = ST_RDOP;
            else 
                next_state = ST_OUT1;
        end
        ST_POOL: begin
            if (pooldone)
                next_state = ST_OUT2;
            else
                next_state = ST_POOL;
        end
        ST_OUT2: begin
            if (count1024 == 12'd1023 & outkernel == 1'b1)
                next_state = ST_FLAT; 
            else
                next_state = ST_POOL;
        end
        ST_FLAT: 
            next_state = ST_OUT3;
        ST_OUT3:
            next_state = (count2048 == 12'd2047) ? ST_DONE : ST_FLAT;
        ST_DONE:
            next_state = ST_DONE;
        default:
            next_state = ST_INIT;
    endcase
end
// }}}

// ******STATE REGISTER*******//
always @(posedge clk or posedge reset) begin
    if (reset)
        curr_state <= ST_INIT;
    else
        curr_state <= next_state;
end

reg busy_asyn;
//assign busy = busy_;
// assign busy = (curr_state == ST_INIT || curr_state == ST_DONE) ? 1'b1: 1'b0;

// ***************************//
// *******OUTPUT LOGIC********//
// ***************************//
// {{{
always @(*) begin
    case (curr_state) 
        ST_INIT: begin
            crd  = 1'b0;
            cwr  = 1'b0;
            busy_asyn = 1'b0;
            csel = 3'b000;
        end
        ST_RDOP: begin
            crd  = 1'b0;
            cwr  = 1'b0;
            busy_asyn = 1'b1;
            csel = 3'b000;
        end
        ST_OUT1: begin
            crd  = 1'b0;
            cwr  = 1'b1;
            busy_asyn = 1'b1;
            csel = (outkernel == 1'b0) ? 3'b001 : 3'b010;
        end
        ST_POOL: begin
            crd  = 1'b1;
            cwr  = 1'b0;
            busy_asyn = 1'b1;
            csel = (outkernel == 1'b0) ? 3'b001 : 3'b010;
        end
        ST_OUT2: begin
            crd  = 1'b0;
            cwr  = 1'b1;
            busy_asyn = 1'b1;
            csel = (outkernel == 1'b0) ? 3'b011 : 3'b100;
        end
        ST_FLAT: begin
            crd  = 1'b1;
            cwr  = 1'b0;
            busy_asyn = 1'b1;
            csel = (outkernel == 1'b0) ? 3'b011 : 3'b100;
        end
        ST_OUT3: begin
            crd  = 1'b0;
            cwr  = 1'b1;
            busy_asyn = 1'b1;
            //busy_asyn = (count2048 == 12'd2048) ? 1'b0 : 1'b1;
            csel = 3'b101;
        end
        ST_DONE: begin
            crd  = 1'b0;
            cwr  = 1'b0;
            busy_asyn = 1'b0;
            csel = 3'b000;
        end
        default: begin
            crd  = 1'b0;
            cwr  = 1'b0;
            busy_asyn = 1'b0;
            csel = 3'b000;
        end
    endcase
end

// busy signal setting
always @(posedge clk, posedge reset) begin
    if(reset)
        busy <= 1'b0;
    else
        busy <= busy_asyn;
end


// }}}

// ***************************//
// ******* outkernel *********//
// ***************************//
// {{{
always @(posedge clk or posedge reset) begin
    if (reset)
        outkernel <= 1'b0;
    else begin
        case (curr_state)
            ST_RDOP:   // READ
                outkernel <= 1'b0;
            ST_OUT1:   // OUT1
                outkernel <= outkernel + 1'b1;
            ST_OUT2:   // OUT2
                outkernel <= outkernel + 1'b1;
            ST_OUT3: 
                outkernel <= outkernel + 1'b1;
        endcase
    end
end
// }}}

// ***************************//
// ******* caddr_wr **********//
// ***************************//
// {{{
always @(*) begin
    case(curr_state)
        ST_RDOP:     // RDOP
            caddr_wr = count4096 - 1;
        ST_OUT1:     // OUT1
            caddr_wr = count4096 - 1;
        ST_POOL:     // POOL
            caddr_wr = count1024;
        ST_OUT2:
            caddr_wr = count1024;
        ST_OUT3:
            caddr_wr = count2048;
        default:
            caddr_wr = 0;
    endcase
end
// }}}

// ***************************//
// ******* cdata_wr **********//
// ***************************//
// {{{
always @(*) begin
    case (curr_state)
        ST_OUT1: begin
            if (outkernel == 1'b0)
                cdata_wr = (conv0_res[19] == 1'b1) ? 20'b0 : conv0_res;
            else
                cdata_wr = (conv1_res[19] == 1'b1) ? 20'b0 : conv1_res; 
        end
        ST_OUT2:
            cdata_wr = poolres;
        ST_OUT3:
            cdata_wr = flatres;
        default: 
            cdata_wr = 20'b0;
    endcase
end
// }}}

// ***************************//
// ******* caddr_rd **********//
// ***************************//
// {{{
always @(*) begin
    case (curr_state)
        ST_POOL:
            caddr_rd = rdaddr_pool;
        ST_OUT2: begin
            caddr_rd = rdaddr_pool;
        end
        ST_FLAT:
            caddr_rd = rdaddr_flat;
        ST_OUT3:
            caddr_rd = 12'b0;
        default:
            caddr_rd = 12'b0;
    endcase
end
// }}}

READ read(
    .clk(clk),
    .rst(reset),
    .ready(ready),
    .readone(readone),
    .rdindex(count4096),
    .read_counter(count9),
    .iaddr(iaddr),
    .idata(idata),
    .state(curr_state),
    .row0(px012),
    .row1(px345),
    .row2(px678)
);

CAL cal(
    .clk(clk),
    .rst(reset),
    .row1(px012),
    .row2(px345),
    .row3(px678),
    .k0_res(conv0_res),
    .k1_res(conv1_res),
    .state(curr_state),
    .counter(count9)
);

POOL pool(
    .clk(clk),
    .rst(reset),
    .cdata_rd(cdata_rd),
    .rdaddr_pool(rdaddr_pool),
    .pooldone(pooldone),
    .state(curr_state),
    .poolres(poolres),
    .kernelsel(outkernel),
    .wrtidx_pool(count1024)
);

FLAT flat(
    .clk(clk),
    .rst(reset),
    .state(curr_state),
    .wrtidx_flat(count2048),
    .rdidx_flat(rdaddr_flat),
    .cdata_rd(cdata_rd),
    .wrt_out(flatres),
    .kernelout(outkernel)
);

endmodule

//*******************************************************************
//********************** READ MODULE start **************************
//*******************************************************************
// {{{
/* reading 9 pixel each round, ans pass 9 pixel to CAL module */
module READ(clk, rst, ready, readone,  rdindex, read_counter, iaddr, idata, state, row0, row1, row2);
    input clk, rst, ready;
    input [2:0] state;
    input [19:0] idata;

    output reg [12:0] rdindex;   // 0-4095, total read px count
    
    output [59:0] row0, row1, row2;
    output [11:0] iaddr;
    output readone;     // count9
    
    output reg [3:0] read_counter; // count 0 - 8
    
    wire [12:0] rdindex_signed;

    reg [5:0] row_counter;  // count 0 - 63
    reg signed [19:0] pixel [0:8];
    wire [12:0] read_index [0:9];    // local reading index(filter)
    reg signed [12:0] addr; // should judge < 0

    wire pad;       // 1: pad, 0: not pad
    wire lborder_detect, rborder_detect, outrange_detect;

    //assign rdindex_signed = {1'b0, rdindex};
    assign rdindex_signed = rdindex;

    assign readone = (read_counter == 4'b1001) ? 1'b1 : 1'b0;
        // reading data index
    assign iaddr = addr[11:0];

        // detect left and right border
    assign lborder_detect = (row_counter == 6'b0 && ((read_index[read_counter] & 13'h3f) == 13'h3f));
    assign rborder_detect = (row_counter == 6'd63 && (((read_index[read_counter] - 13'b1) & 13'h3f) == 13'h3f));
        // detect top and bottom border
    assign outrange_detect = (read_index[read_counter] > 4095);
    assign pad = outrange_detect | lborder_detect | rborder_detect;
    
        // READ IN, output to CAL module
    assign row0 = {pixel[0], pixel[1], pixel[2]},
           row1 = {pixel[3], pixel[4], pixel[5]},
           row2 = {pixel[6], pixel[7], pixel[8]};

        // INDEX of 9 pixel
    assign read_index[0] = rdindex_signed - 12'd65 ,
           read_index[1] = rdindex_signed - 12'd64 ,
           read_index[2] = rdindex_signed - 12'd63 ,
           read_index[3] = rdindex_signed - 12'd1  ,
           read_index[4] = rdindex_signed          ,
           read_index[5] = rdindex_signed + 12'd1  ,
           read_index[6] = rdindex_signed + 12'd63 ,
           read_index[7] = rdindex_signed + 12'd64 ,
           read_index[8] = rdindex_signed + 12'd65 ,
           read_index[9] = -1;

        // READ PIXEL, update addr and pixel
    integer i;
    always @(*) begin
        if (rst)
            addr = 13'b0;
        else begin
            case (state)
                3'b001: 
                    addr = read_index[read_counter];
                default:
                    addr = 13'b0;
            endcase
            
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 9; i = i + 1)
                pixel[i] <= 20'b0;
            //addr    <= 13'b0;
        end
        else begin
            case (state) 
                3'b001: begin
                    //addr                <= read_index[read_counter];
                    pixel[read_counter] <= (pad) ? 20'b0 : idata;
                end
            endcase
        end
    end

        // COUNTER SETTING
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rdindex         <= 12'b0;
            row_counter     <= 6'b0;
            read_counter    <= 4'b0;
        end
        else begin
            if (state == 3'b001) begin  // RDOP state
                read_counter <= read_counter + 1;
                if (read_counter == 4'b1000) begin
                    row_counter <= row_counter + 6'b1;
                    rdindex         <= rdindex + 12'b1; // increment each round
                end
            end
            else if (state == 3'b010) begin     // OUT1 state
                read_counter    <= 4'b0;
            end
        end
    end
endmodule
//********************** READ MODULE end **************************
// }}}

//******************************************************************
//********************** CAL module start **************************
//******************************************************************
// {{{
/* multiple pixel and kernel, adding bias and return conv result */
module CAL(
    clk, rst, 
    row1, row2, row3,
    k0_res, k1_res,
    state,
    counter            /* receive from READ mod(read_counter), count 0 - 8 each iteration */
);
    input clk, rst;
    input [59:0] row1, row2, row3;
    input [2:0] state;
    output [19:0] k0_res, k1_res;
    input [3:0] counter;
    reg signed [39:0] mult1 [0:8];  // save temp multiple result
    reg signed [39:0] mult2 [0:8];  // save temp multiple result

    wire signed [39:0] bias1, bias2;
    wire signed [19:0] px [0:8];
    wire signed [39:0] tempsum1, tempsum2; 
    wire signed [19:0] ker1 [0:8];
    wire signed [19:0] ker2 [0:8];

    assign bias1 = {4'b0, 20'h01310, 16'b0},
           bias2 = {4'b0, 20'hF7295, 16'b0};

    assign px[0] = row1[59:40], px[1] = row1[39:20], px[2] = row1[19:0],
           px[3] = row2[59:40], px[4] = row2[39:20], px[5] = row2[19:0],
           px[6] = row3[59:40], px[7] = row3[39:20], px[8] = row3[19:0];

    assign ker1[0] = 20'h0A89E, ker1[1] = 20'h092D5, ker1[2] = 20'h06D43,
           ker1[3] = 20'h01004, ker1[4] = 20'hF8F71, ker1[5] = 20'hF6E54,
           ker1[6] = 20'hFA6D7, ker1[7] = 20'hFC834, ker1[8] = 20'hFAC19;
    assign ker2[0] = 20'hFDB55, ker2[1] = 20'h02992, ker2[2] = 20'hFC994,
           ker2[3] = 20'h050FD, ker2[4] = 20'h02F20, ker2[5] = 20'h0202D,
           ker2[6] = 20'h03BD7, ker2[7] = 20'hFD369, ker2[8] = 20'h05E68;
        
        // MULTIPLE pixel with kernel and ADD bias
    always @ (*) begin
            // KERNEL 1
        mult1[0] = px[0] * ker1[0];
        mult1[1] = px[1] * ker1[1];
        mult1[2] = px[2] * ker1[2];
        mult1[3] = px[3] * ker1[3];
        mult1[4] = px[4] * ker1[4];
        mult1[5] = px[5] * ker1[5];
        mult1[6] = px[6] * ker1[6];
        mult1[7] = px[7] * ker1[7];
        mult1[8] = px[8] * ker1[8];
            // KERNEL 2
        mult2[0] = px[0] * ker2[0];
        mult2[1] = px[1] * ker2[1];
        mult2[2] = px[2] * ker2[2];
        mult2[3] = px[3] * ker2[3];
        mult2[4] = px[4] * ker2[4];
        mult2[5] = px[5] * ker2[5];
        mult2[6] = px[6] * ker2[6];
        mult2[7] = px[7] * ker2[7];
        mult2[8] = px[8] * ker2[8];
    end
    /*
    integer k;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (k = 0; k < 9; k = k + 1) begin
                mult1[k] <= 40'b0;
                mult2[k] <= 40'b0;
            end
        end
        else begin
            mult1[counter] <= px[counter] * ker1[counter];
            mult2[counter] <= px[counter] * ker2[counter];
        end
    end 
    */
    assign tempsum1 = (mult1[0] + mult1[1]) + (mult1[2] + mult1[3]) + (mult1[4] + mult1[5]) + (mult1[6] + mult1[7]) + (mult1[8]) + bias1,
           tempsum2 = (mult2[0] + mult2[1]) + (mult2[2] + mult2[3]) + (mult2[4] + mult2[5]) + (mult2[6] + mult2[7]) + (mult2[8]) + bias2;

    assign k0_res = (tempsum1[35:16] + tempsum1[15]),
           k1_res = (tempsum2[35:16] + tempsum2[15]);

endmodule
//********************** CAL module end **************************
// }}}

//******************************************************************
//********************** POOL module start *************************
//******************************************************************
// {{{
module POOL(clk, rst, cdata_rd, rdaddr_pool, pooldone, state, poolres, kernelsel, wrtidx_pool);
    input clk, rst;
    input [19:0] cdata_rd;
    input [2:0] state;
    output pooldone;
    output [19:0] poolres;
    input kernelsel;
    output [11:0] rdaddr_pool;          // Control read index
    output reg [11:0] wrtidx_pool;          // Control write index

    reg [11:0] read_index;
    reg [3:0] count4;          // count 0 - 3, control each iteration
    wire [11:0] read_addr4 [0:4];
    reg [19:0] tempmax;         // store max
    reg [5:0] row_count;        // count 0 - 63, countrol read_index

    assign pooldone = (count4 == 3'd4) ? 1'b1 : 1'b0; 
    assign poolres  = tempmax;
        
    assign read_addr4[0] = read_index         ,
           read_addr4[1] = read_index + 12'b1 ,
           read_addr4[2] = read_index + 12'd64,
           read_addr4[3] = read_index + 12'd65,
           read_addr4[4] = read_index + 12'd65;
    assign rdaddr_pool = read_addr4[count4];
    reg [19:0] max1, max2;
    reg [19:0] in4 [0:4];


    always @(posedge clk or posedge rst) begin
        if (rst) begin
            read_index  <= 12'b0;
            count4      <= 3'b0;
            // reset in4
            in4[0] <= 20'b0; in4[1] <= 20'b0; in4[2] <= 20'b0; in4[3] <= 20'b0; in4[4] <= 20'b0;
            row_count   <= 6'b0;
            wrtidx_pool <= 12'b0;
        end
        else begin
            case (state)
                3'b010: begin   // OUT1 state, prepare signal
                    read_index  <= 12'b0;
                    row_count   <= 6'b0;
                    wrtidx_pool <= 12'b0;
                end
                3'b011: begin   // POOL state
                    count4      <= count4 + 3'b1;
                    in4[count4] <= cdata_rd;
                end
                3'b100: begin   // OUT2 state
                    if (kernelsel == 1'b1) begin
                        count4      <= 0;
                        read_index  <= (row_count == 6'd62) ? (read_index + 12'd66) : (read_index + 12'd2);
                        row_count   <= (row_count == 6'd62) ? 0 : (row_count + 6'd2);
                        wrtidx_pool <= wrtidx_pool + 12'b1;
                    end                 
                    else begin
                        count4  <= 3'b0;
                    end
                end
            endcase
        end
    end

    // GET MAX
    always @(*) begin
        if (rst) begin
            max1 = 20'b0; max2 = 20'b0; tempmax = 20'b0;
        end
        else begin
            max1 = (in4[0] > in4[1]) ? in4[0] : in4[1];
            max2 = (in4[2] > in4[3]) ? in4[2] : in4[3];
            tempmax = (max1 > max2) ? max1 : max2;
        end
    end
endmodule
// }}}

//******************************************************************
//********************** FLAT module start *************************
//******************************************************************
//{{{
module FLAT(clk, rst, state, wrtidx_flat, rdidx_flat, cdata_rd, wrt_out, kernelout);
    input clk, rst;
    input [2:0] state;
    input [19:0] cdata_rd;
    input kernelout;
    output reg [11:0] wrtidx_flat;      // write 0 - 2048
    output reg [11:0] rdidx_flat;         // read 0 - 1023
    output [19:0] wrt_out;

    assign wrt_out = cdata_rd;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wrtidx_flat <= 12'hfff;
            rdidx_flat  <= 12'b0;
        end
        else begin
            case (state) 
                3'b101: begin       // FLAT
                    wrtidx_flat <= wrtidx_flat + 12'b1;
                end
                3'b110: begin       // OUT3
                    if (kernelout == 1'b1) rdidx_flat <= rdidx_flat + 12'b1;
                end 
            endcase
        end
    end

endmodule 
// }}}
