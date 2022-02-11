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
  logic flash;
  logic hz1;
  logic [7:0] ctr;
  
  //count8du c8_1(.CLK(hz100), .RST(reset), .DIR(pb[17]), .MAX(8'd128), .E(pb[18]), .q(right));
  //count8du c8_1(.CLK(hz100), .RST(reset), .DIR(1'b0), .MAX(8'd49), .E(1'b1), .Q(ctr));
      
  always_ff @(posedge hz100, posedge reset) begin
    if (reset == 1'b1) begin
      hz1 <= 0;
    end
    else begin
      hz1 <= ctr == 8'd49;
    end
  end
  
  always_ff @(posedge hz1, posedge reset) begin
    if (reset == 1'b1) begin
      flash <= 0;
    end
    else begin
      flash <= ~flash;
    end
  end

  assign blue = flash;

  //HANGMAN
  hangman hg (.hz100(hz100), .reset(pb[19]), .hex(pb[15:10]), .ctrdisp(ss7[6:0]), .letterdisp({ss3[6:0], ss2[6:0], ss1[6:0], ss0[6:0]}), .win(green), .lose(red), .flash(flash));

endmodule

// Add more modules down here...
module count8du (input logic CLK, input logic RST, input logic DIR, input logic [7:0] MAX, input logic E, output logic [7:0] Q);
  logic [7:0] next_Q;
  
  always_ff @(posedge CLK, posedge RST) begin
    if (RST == 1'b1)
      Q <= 8'b00000000;
    else
      Q <= next_Q;
  end
  
  always_comb begin
    next_Q = Q;
    
    if (E == 1'b1) begin
    
        if (DIR == 1'b0) begin
        
            if (Q == 8'b0) begin
            next_Q = MAX;
            end
            
            else begin
            next_Q[0] = ~Q[0];
            next_Q[1] = Q[1] ^ ~Q[0];
            next_Q[2] = Q[2] ^ &(~Q[1:0]);
            next_Q[3] = Q[3] ^ &(~Q[2:0]);
            next_Q[4] = Q[4] ^ &(~Q[3:0]);
            next_Q[5] = Q[5] ^ &(~Q[4:0]);
            next_Q[6] = Q[6] ^ &(~Q[5:0]);
            next_Q[7] = Q[7] ^ &(~Q[6:0]);
            end
        end    
        
        else if (DIR == 1'b1) begin
        
            if (Q == MAX) begin
            next_Q = 8'b0;
            end
            
            else begin
            next_Q[0] = ~Q[0];
            next_Q[1] = Q[1] ^ Q[0];
            next_Q[2] = Q[2] ^ &Q[1:0];
            next_Q[3] = Q[3] ^ &Q[2:0];
            next_Q[4] = Q[4] ^ &Q[3:0];
            next_Q[5] = Q[5] ^ &Q[4:0];
            next_Q[6] = Q[6] ^ &Q[5:0];
            next_Q[7] = Q[7] ^ &Q[6:0];
            end
            
        end
    end
  end
endmodule

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
