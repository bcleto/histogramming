`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/05/2021 11:30:48 AM
// Design Name: 
// Module Name: tb_2
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


module tb_2(

    );
//inputs - reg
reg ui_clk,input_trigger,ap_rst,ap_start;
reg [114:0] data_in;
//outputs - wire
wire ap_done,ap_idle,ap_ready;
wire [15:0] frequency_out_V_V_din,mode_out_V_V_din;
    
    
    
part_hist_v1_0 parthist1 (
  .ap_clk(ui_clk),                                          // input wire ap_clk
  .ap_rst(ap_rst),                                          // input wire ap_rst
  .ap_start(ap_start),                                      // input wire ap_start
  .ap_done(ap_done),                                        // output wire ap_done
  .ap_idle(ap_idle),                                        // output wire ap_idle
  .ap_ready(ap_ready),                                      // output wire ap_ready
  .data_in_V(data_in),                                    // input wire [114 : 0] data_in_V
  .input_valid_V(input_trigger),                            // input wire [0 : 0] input_valid_V
  
  .accumulation_V(12),                          // input wire [31 : 0] accumulation_V
  .frequency_out_V_V_din(frequency_out_V_V_din),            // output wire [15 : 0] data_out_fifo_V_V_din
  .frequency_out_V_V_full_n(1'b1),      // input wire data_out_fifo_V_V_full_n
  .frequency_out_V_V_write(frequency_out_V_V_write),        // output wire data_out_fifo_V_V_write
  .mode_out_V_V_din(mode_out_V_V_din),        // output wire [15 : 0] pixel_addr_fifo_V_V_din
  .mode_out_V_V_full_n(1'b1),  // input wire pixel_addr_fifo_V_V_full_n
  .mode_out_V_V_write(mode_out_V_V_write)    // output wire pixel_addr_fifo_V_V_write
);


always #5 ui_clk=~ui_clk;

initial
begin
ap_rst=1;
ui_clk=0;
ap_start = 0;
input_trigger= 0 ;
data_in = $urandom(2**115 - 1);


#1000 ap_rst=0;
ap_start=1;

//wait for memory reset
#80300 input_trigger=1;
#20 input_trigger=0;

#3500 input_trigger=1;
#20 input_trigger=0;

#3500 input_trigger=1;
#20 input_trigger=0;


#3500 input_trigger=1;
#20 input_trigger=0;

#3500 input_trigger=1;
#20 input_trigger=0;

#3500 input_trigger=1;
#20 input_trigger=0;

#3500 input_trigger=1;
#20 input_trigger=0;

#3500 input_trigger=1;
#20 input_trigger=0;

#3500 input_trigger=1;
#20 input_trigger=0;

#3500 input_trigger=1;
#20 input_trigger=0;


#13500 input_trigger=1;
#20 input_trigger=0;

#3500 input_trigger=1;
#20 input_trigger=0;

#3500 input_trigger=1;
#20 input_trigger=0;

#3500 input_trigger=1;
#20 input_trigger=0;

#3500 input_trigger=1;
#20 input_trigger=0;
end  


endmodule
