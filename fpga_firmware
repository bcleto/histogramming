`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/21/2021 09:51:07 AM
// Design Name: 
// Module Name: top_v10
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
//1 channel partial histogram trial

module top_v10(
	input wire clk_in1_p,
    input wire clk_in1_n,
	input  wire [4:0]   okUH,
	output wire [2:0]   okHU,
	inout wire  [31:0]  okUHU,
	inout  wire         okAA,
	
	output wire [3:0]   led
    );
    
        
    wire okClk;
    wire [112:0] okHE;
    wire [64:0]  okEH;
    wire [31:0]  ep00wire, ep01wire,ep02wire,ep03wire,ep04wire,ep22wire,ep23wire,ep24wire,ep25wire,ep26wire;
    wire [31:0]  ep20wire, ep21wire,ep3ewire,ep3fwire,din,dout,pipe_out_data,pipe_in_data,pipe_out_data_a1;
    wire pipe_in_write,pipe_in_ready;
    wire pipe_out_read,pipe_out_ready,pipe_out_read_a1,pipe_out_ready_a1;
    wire reset,empty,locked,reset_clk;
    wire [11:0] data_count;
    wire mhz200,trigger_user_left,trigger_user_right;
    wire [4109:0] data_in_left,data_in_right;
    
function [3:0] xem7360_led;
input [3:0] a;
integer i;
begin
	for(i=0; i<4; i=i+1) begin: u
		xem7360_led[i] = (a[i]==1'b1) ? (1'b0) : (1'bz);
	end
end
endfunction

/////////////////////////CLOCK GENERATOR///////////
//wire clk_shifted,okClk_shifted_buffered;   
//DONT FORGET TO ADD BUFFERS AT INPUT AND OUTPUT OF CLOCK GENERATOR
//BUFG buffer_out (.O (okClk_shifted_buffered),
//       .I (clk_shifted));

wire pll_ref_clk,pll_ref_clk_bef,ui_clk;
IBUFDS #(
          .DIFF_TERM("FALSE"),       // Differential Termination
          .IBUF_LOW_PWR("TRUE"),     // Low power="TRUE", Highest performance="FALSE" 
          .IOSTANDARD("DEFAULT")     // Specify the input I/O standard
       ) 
IBUFDS_inst (
          .O(pll_ref_clk_bef),  // Buffer output
          .I(clk_in1_p),  // Diff_p buffer input (connect directly to top-level port)
          .IB(clk_in1_n) // Diff_n buffer input (connect directly to top-level port)
       );
	
BUFG buff_pll_ref_clk (.O (ui_clk),
       .I (pll_ref_clk_bef));
       
////////////////////////////////////////////////////////
wire pll_ref_clk_bef;

//trigger
wire [31:0] ep40trigin,ep41trigin;
//assign user_trigger=ep40trigin[0];
//assign reset=ep40trigin[1];
assign reset_clk=ep40trigin[2];





wire start=ep00wire[0];
assign reset=ep00wire[1];
wire reset2=ep00wire[2];

wire [31:0]  ep05wire,ep06wire;
               
wire [4:0] current_state,next_state;
assign ep21wire=current_state;



assign trigger=ep40trigin[0];
//assign reset=ep40trigin[1];
wire read_trg=ep40trigin[3];

assign trigger2=ep40trigin[4];
assign trigger3=ep40trigin[5];

wire  aresetn=!reset;
wire reset2_n=!reset2;


wire [31:0] state;


reg trigger_reg,frame_register;
reg [3:0] trig_counter;

wire [31:0] data_out_fifo_V_V_din,pixel_addr_fifo_V_din ;



wire [20:0] din_fifo,dout_fifo,dout_ram,data_out_V_V_din;
reg [16:0] addra;
assign din_fifo=dout_ram;
reg wr_en;


// MAIN PART STARTS HERE



wire ap_rst = ep05wire[0];
wire ap_start = ep05wire[1];
wire [31:0] data_in = ep06wire [31:0];
wire input_valid=ep40trigin[1];
wire [31: 0] accumulation_V = ep04wire;
wire [15:0] frequency_out_V_V_din,mode_out_V_V_din;

part_hist_v1_0 hist1 (
  .ap_clk(ui_clk),                                      // input wire ap_clk
  .ap_rst(ap_rst),                                      // input wire ap_rst
  .ap_start(ap_start),                                  // input wire ap_start
  .ap_done(ap_done),                                    // output wire ap_done
  .ap_idle(ap_idle),                                    // output wire ap_idle
  .ap_ready(ap_ready),                                  // output wire ap_ready
  .data_in_V(data_in),                                // input wire [114 : 0] data_in_V
  .input_valid_V(input_valid),                        // input wire [0 : 0] input_valid_V
  .accumulation_V(accumulation_V),                      // input wire [31 : 0] accumulation_V
  
  .frequency_out_V_V_din(frequency_out_V_V_din),        // output wire [15 : 0] frequency_out_V_V_din
  .frequency_out_V_V_full_n(!freq_full),  // input wire frequency_out_V_V_full_n
  .frequency_out_V_V_write(frequency_out_V_V_write),    // output wire frequency_out_V_V_write
  .mode_out_V_V_din(mode_out_V_V_din),                  // output wire [15 : 0] mode_out_V_V_din
  .mode_out_V_V_full_n(!mode_full),            // input wire mode_out_V_V_full_n
  .mode_out_V_V_write(mode_out_V_V_write)              // output wire mode_out_V_V_write
);



fifo_generator_7 freq_fifo (
  .rst(ap_rst),                  // input wire rst
  .wr_clk(ui_clk),            // input wire wr_clk
  .rd_clk(okClk),            // input wire rd_clk
  .din(frequency_out_V_V_din),                  // input wire [15 : 0] din
  .wr_en(frequency_out_V_V_write),              // input wire wr_en
  .rd_en(pipe_out_read),              // input wire rd_en
  .dout(pipe_out_data),                // output wire [15 : 0] dout
  .full(freq_full),                // output wire full
  .empty(freq_empty),              // output wire empty
  .wr_data_count(ep20wire),  // output wire [6 : 0] wr_data_count
  .wr_rst_busy(),  // output wire wr_rst_busy
  .rd_rst_busy()  // output wire rd_rst_busy
);


fifo_generator_7 mode_fifo (
  .rst(ap_rst),                  // input wire rst
  .wr_clk(ui_clk),            // input wire wr_clk
  .rd_clk(okClk),            // input wire rd_clk
  .din(mode_out_V_V_din),                  // input wire [15 : 0] din
  .wr_en(mode_out_V_V_write),              // input wire wr_en
  .rd_en(pipe_out_read_a1),              // input wire rd_en            
  .dout(pipe_out_data_a1),                // output wire [15 : 0] dout  
  .full(mode_full),                // output wire full
  .empty(pipein_empty),              // output wire empty
  .wr_data_count(),  // output wire [6 : 0] wr_data_count
  .wr_rst_busy(),  // output wire wr_rst_busy
  .rd_rst_busy()  // output wire rd_rst_busy
);

/*
fifo_generator_7 pipein (
  .rst(ap_rst),                  // input wire rst
  .wr_clk(okClk),            // input wire wr_clk
  .rd_clk(okClk),            // input wire rd_clk
  .din(pipe_in_data),                  // input wire [15 : 0] din
  .wr_en(pipe_in_write),              // input wire wr_en
  .rd_en(pipe_out_read_a1),              // input wire rd_en
  .dout(pipe_out_data_a1),                // output wire [15 : 0] dout
  .full(),                // output wire full
  .empty(pipein_empty),              // output wire empty
  .wr_data_count(),  // output wire [6 : 0] wr_data_count
  .wr_rst_busy(),  // output wire wr_rst_busy
  .rd_rst_busy()  // output wire rd_rst_busy
);
*/

//LEDs
assign led = xem7360_led({pipein_empty,ap_idle,ap_ready,freq_empty}); //D3,D2,D1,D0

// Instantiate the okHost and connect endpoints.
wire [65*12-1:0]  okEHx;

okHost okHI(
	.okUH(okUH),
	.okHU(okHU),
	.okUHU(okUHU),
	.okAA(okAA),
	.okClk(okClk),
	.okHE(okHE), 
	.okEH(okEH)
);

okWireOR # (.N(12)) wireOR (okEH, okEHx);

okWireIn     wi00(.okHE(okHE),                             .ep_addr(8'h00), .ep_dataout(ep00wire));
okWireIn     wi01(.okHE(okHE),                             .ep_addr(8'h01), .ep_dataout(ep01wire));
okWireIn     wi02(.okHE(okHE),                             .ep_addr(8'h02), .ep_dataout(ep02wire));
okWireIn     wi03(.okHE(okHE),                             .ep_addr(8'h03), .ep_dataout(ep03wire));
okWireIn     wi04(.okHE(okHE),                             .ep_addr(8'h04), .ep_dataout(ep04wire));
okWireIn     wi05(.okHE(okHE),                             .ep_addr(8'h05), .ep_dataout(ep05wire));
okWireIn     wi06(.okHE(okHE),                             .ep_addr(8'h06), .ep_dataout(ep06wire));

okWireOut    wo20(.okHE(okHE), .okEH(okEHx[ 0*65 +: 65 ]), .ep_addr(8'h20), .ep_datain(ep20wire));
okWireOut    wo21(.okHE(okHE), .okEH(okEHx[ 1*65 +: 65 ]), .ep_addr(8'h21), .ep_datain(ep21wire));
okWireOut    wo22(.okHE(okHE), .okEH(okEHx[ 7*65 +: 65 ]), .ep_addr(8'h22), .ep_datain(ep22wire));
okWireOut    wo23(.okHE(okHE), .okEH(okEHx[ 8*65 +: 65 ]), .ep_addr(8'h23), .ep_datain(ep23wire));
okWireOut    wo24(.okHE(okHE), .okEH(okEHx[ 10*65 +: 65 ]), .ep_addr(8'h24), .ep_datain(ep24wire));
okWireOut    wo25(.okHE(okHE), .okEH(okEHx[ 11*65 +: 65 ]), .ep_addr(8'h25), .ep_datain(ep25wire));
okWireOut    wo26(.okHE(okHE), .okEH(okEHx[ 9*65 +: 65 ]), .ep_addr(8'h26), .ep_datain(ep26wire));

okWireOut    wo3e(.okHE(okHE), .okEH(okEHx[ 2*65 +: 65 ]), .ep_addr(8'h3e), .ep_datain(ep3ewire));
okWireOut    wo3f(.okHE(okHE), .okEH(okEHx[ 3*65 +: 65 ]), .ep_addr(8'h3f), .ep_datain(ep3fwire));

okBTPipeIn   ep80(.okHE(okHE), .okEH(okEHx[ 4*65 +: 65 ]), .ep_addr(8'h80), .ep_write(pipe_in_write), .ep_blockstrobe(), .ep_dataout(pipe_in_data), .ep_ready(pipe_in_ready));
okBTPipeOut  epA0(.okHE(okHE), .okEH(okEHx[ 5*65 +: 65 ]), .ep_addr(8'ha0), .ep_read(pipe_out_read),  .ep_blockstrobe(), .ep_datain(pipe_out_data), .ep_ready(pipe_out_ready));
okBTPipeOut  epA1(.okHE(okHE), .okEH(okEHx[ 6*65 +: 65 ]), .ep_addr(8'ha1), .ep_read(pipe_out_read_a1),  .ep_blockstrobe(), .ep_datain(pipe_out_data_a1), .ep_ready(pipe_out_ready_a1));


okTriggerIn trigin40(   .okHE(okHE),    .ep_addr(8'h40),  .ep_clk(ui_clk),   .ep_trigger(ep40trigin)); 
//okTriggerIn trigin40(   .okHE(okHE),    .ep_addr(8'h40),  .ep_clk(mhz_50),   .ep_trigger(ep40trigin)); 
okTriggerIn trigin41(   .okHE(okHE),    .ep_addr(8'h41),  .ep_clk(ui_clk),   .ep_trigger(ep41trigin));   


assign pipe_out_ready_a1=1'b1;
assign pipe_in_ready=1'b1;
assign pipe_out_ready=1'b1;
    
endmodule
