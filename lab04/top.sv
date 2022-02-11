`default_nettype none

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
  
  //Step 1
  
  //assign ss0[6:0] = pb[6:0];  
  
  //Step 2
  
  bargraph num_1(.out({left[7:0],right[7:0]}), .in(pb[15:0]));
  
  //Step 3
  
  decode3to8 ins_and_outs(.out({ss7[7], ss6[7], ss5[7], ss4[7], ss3[7], ss2[7], ss1[7], ss0[7]}), .in(pb[2:0]));

endmodule

// Add your submodules below.

module bargraph ( 
    input logic [15:0]in,
    output logic [15:0]out
);
    assign out[0] = |in[15:0];
    assign out[1] = |in[15:1];
    assign out[2] = |in[15:2];
    assign out[3] = |in[15:3];
    assign out[4] = |in[15:4];
    assign out[5] = |in[15:5];
    assign out[6] = |in[15:6];
    assign out[7] = |in[15:7];
    assign out[8] = |in[15:8];
    assign out[9] = |in[15:9];
    assign out[10] = |in[15:10];
    assign out[11] = |in[15:11];
    assign out[12] = |in[15:12];
    assign out[13] = |in[15:13];
    assign out[14] = |in[15:14];
    assign out[15] = |in[15:15];
    
endmodule

module decode3to8 (
    output logic [7:0]out,
    input logic [2:0]in
);
                
assign out[0] = in[2:0] == 3'b000;
assign out[1] = in[2:0] == 3'b001;
assign out[2] = in[2:0] == 3'b010;
assign out[3] = in[2:0] == 3'b011;
assign out[4] = in[2:0] == 3'b100;
assign out[5] = in[2:0] == 3'b101;
assign out[6] = in[2:0] == 3'b110;
assign out[7] = in[2:0] == 3'b111;

endmodule
