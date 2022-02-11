`default_nettype none
// Empty top module

module top (
  // I/O ports
  input  logic hz100, reset,
  input  logic [20:0] pb,
  output logic [7:0] left, right,
         ss7, ss6, ss5, ss4, ss3, ss2, ss1, ss0,
  output logic red, green, blue,

  // UART ports
  output logic [7:0] txdata,
  input  logic [7:0] rxdata,
  output logic txclk, rxclk,
  input  logic txready, rxready
);

  // Your code goes here...
  logic [3:0] encout;
  logic strobe;
  ssdec v1(.in(encout), .enable(strobe), .out(ss0[6:0]));
  prienc16to4 u1(.in(pb[15:0]), .out(encout), .strobe(strobe));
  
endmodule

// Add more modules down here...
module ssdec (
  input logic [3:0]in,
  input logic enable,
  output logic [6:0] out
);

logic [6:0] seg[15:0];

assign seg[0] = (enable == 1) ? 7'b0111111 : 0;
assign seg[1] = (enable == 1) ? 7'b0000110 : 0;
assign seg[2] = (enable == 1) ? 7'b1011011 : 0;
assign seg[3] = (enable == 1) ? 7'b1001111 : 0;
assign seg[4] = (enable == 1) ? 7'b1100110 : 0;
assign seg[5] = (enable == 1) ? 7'b1101101 : 0;
assign seg[6] = (enable == 1) ? 7'b1111101 : 0;
assign seg[7] = (enable == 1) ? 7'b0000111 : 0;
assign seg[8] = (enable == 1) ? 7'b1111111 : 0;
assign seg[9] = (enable == 1) ? 7'b1100111 : 0;
assign seg[10] = (enable == 1) ? 7'b1110111 : 0;
assign seg[11] = (enable == 1) ? 7'b1111100 : 0;
assign seg[12] = (enable == 1) ? 7'b0111001 : 0;
assign seg[13] = (enable == 1) ? 7'b1011110 : 0;
assign seg[14] = (enable == 1) ? 7'b1111001 : 0;
assign seg[15] = (enable == 1) ? 7'b1110001 : 0;

assign out[6:0] = seg[in];

endmodule

module prienc16to4 (
  input logic [15:0]in,
  output logic [3:0]out,
  output logic strobe
);

assign {out, strobe} =  in[15] == 1 ? 5'b11111 :
                        in[14] == 1 ? 5'b11101 :
                        in[13] == 1 ? 5'b11011 :
                        in[12] == 1 ? 5'b11001 :
                        in[11] == 1 ? 5'b10111 :
                        in[10] == 1 ? 5'b10101 :
                        in[9] == 1 ? 5'b10011 :
                        in[8] == 1 ? 5'b10001 :
                        in[7] == 1 ? 5'b01111 :
                        in[6] == 1 ? 5'b01101 :
                        in[5] == 1 ? 5'b01011 :
                        in[4] == 1 ? 5'b01001 :
                        in[3] == 1 ? 5'b00111 :
                        in[2] == 1 ? 5'b00101 :
                        in[1] == 1 ? 5'b00011 :
                        in[0] == 1 ? 5'b00001 :
                        5'b00000;

endmodule
