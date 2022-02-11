`default_nettype none
typedef enum logic { RDY, ENT } simonstate_t;

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

//Step 2
  /*logic hz1;
  logic [7:0] ctr;
  clock_1hz setter(.hz100(hz100), .reset(reset), .hz1(hz1));
  count8du_init v1(.CLK(hz1), .RST(reset), .DIR(1'b0), .INIT(8'd5), .E(ctr != 0), .Q(ctr));
  ssdec display(.in(ctr[3:0]), .enable(ctr != 0), .out(ss0[6:0]));*/

  //Step 3
  /*logic [7:0] ctr1, ctr2, ctr3, ctr4;
  count8du_init flashnum1 (.CLK(hz100), .RST(reset), .DIR(1'b0), .E(ctr1 != 0), .INIT(8'h99), .Q(ctr1));
  count8du_init flashnum2 (.CLK(hz100), .RST(reset), .DIR(1'b0), .E(ctr2 != 0), .INIT(8'hAB), .Q(ctr2));
  count8du_init flashnum3 (.CLK(hz100), .RST(reset), .DIR(1'b0), .E(ctr3 != 0), .INIT(8'hCD), .Q(ctr3));
  count8du_init flashnum4 (.CLK(hz100), .RST(reset), .DIR(1'b0), .E(ctr4 != 0), .INIT(8'hEF), .Q(ctr4));
  display_32_bit show_entry (.in({ctr1, ctr2, ctr3, ctr4}), .out({ss7, ss6, ss5, ss4, ss3, ss2, ss1, ss0}));*/
  
  //Step 4
    simon game (.clk(hz100), .reset(reset), .in(pb[19:0]), .left(left), .right(right), 
      .ss({ss7, ss6, ss5, ss4, ss3, ss2, ss1, ss0}), .win(green), .lose(red));
      
  endmodule

module clock_1hz(input logic hz100, input logic reset, output logic hz1);
  logic [7:0] next_Q;
  logic [7:0] Q;
  logic temp;
  
  always_ff @(posedge hz100, posedge reset) begin
      if (reset == 1'b1) begin
        Q <= 0;
      end
      else begin
        Q <= next_Q;
      end
    end
    
  always_ff @(posedge hz100) begin
      temp <= (Q < 8'd49) ? 1 : 0;
  end
  
  always_ff @(posedge temp, posedge reset) begin
    if (reset == 1'b1) begin
      hz1 <= 0;
    end
    else begin
      hz1 <= ~hz1;
    end
  end
  
  always_comb begin
    if(Q == 8'd49) begin
      next_Q = 0;
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
endmodule

module scankey(input logic clk, input logic rst, input logic [19:0] in, output logic [4:0] out, output logic strobe);
  logic delay0;
  logic delay1;

  always_ff @(posedge clk, posedge rst) begin
    if(rst) begin
        delay1 <= 0;
        delay0 <= 0;
    end
    else begin
        delay1 <= delay0;
        delay0 <= |in[19:0];
    end
  end
  
  assign strobe = delay1;
  assign out[4] = |in[19:16];
  assign out[3] = (|in[15:8]);
  assign out[2] = (|in[15:12]) | (|in[7:4]);
  assign out[1] = (|in[19:18]) | (|in[15:14]) | (|in[11:10]) | (|in[7:6]) | (|in[3:2]);
  assign out[0] = in[19] | in[17] | in[15] | in[13] | in[11] | in[9] | in[7] | in[5] | in[3] | in[1];
  
endmodule

module numentry(input logic clk, input logic [4:0] in, input logic rst, input logic clr, input logic en, output logic [31:0] out);
  logic [31:0] next_out;
  
  always_ff @(posedge clk, posedge rst) begin
    if(rst)
      out <= 32'b0;
    else if(clr)
      out <= 32'b0;
    else if(en)
      out <= next_out;
  end
  
  always_comb begin
    if(in <= 9) begin
      if(out == 0)
        next_out = {28'b0,in[3:0]};
      else
        next_out = ((out << 4) | {28'b0,in[3:0]});
    end
    else
      next_out = out;
    end
endmodule

module simonctl(input logic clk, input logic rst, input logic lvlmax, input logic win, input logic lose, output logic state);
  logic next_state;
  //typedef enum logic { RDY, ENT } simonstate_t;
  
  always_ff @(posedge rst, posedge clk) begin
    if(rst)
      state <= RDY;
    else
      state <= next_state;
  end
  
  always_comb begin
    if(lose | (~lvlmax & win))
      next_state = RDY;
    else if(~lvlmax)
      next_state = ENT;
    else  
      next_state = state;
  end  
endmodule

// Add more modules down here...
module count8du_init (input logic CLK, input logic RST, input logic DIR, input logic [7:0] INIT, input logic E, output logic [7:0] Q);
  logic [7:0] next_Q;
  
  always_ff @(posedge CLK, posedge RST) begin
    if (RST == 1'b1)
      Q <= INIT;
    else
      Q <= next_Q;
  end
  
  always_comb begin
    next_Q = Q;
    
    if (E == 1'b1) begin
    
        if (DIR == 1'b0) begin
        
            if (Q == 8'b0) begin
            next_Q = INIT;
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
        
            if (Q == INIT) begin
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

module display_32_bit(input logic [31:0] in, output logic [63:0] out);
    assign {out[7],out[15],out[23],out[31],out[39],out[47],out[55],out[63]} = 0;
  ssdec display1(.in(in[3:0]), .enable(1'b1), .out(out[6:0]));
  ssdec display2(.in(in[7:4]), .enable(|in[31:4]), .out(out[14:8]));
  ssdec display3(.in(in[11:8]), .enable(|in[31:8]), .out(out[22:16]));
  ssdec display4(.in(in[15:12]), .enable(|in[31:12]), .out(out[30:24]));
  ssdec display5(.in(in[19:16]), .enable(|in[31:16]), .out(out[38:32]));
  ssdec display6(.in(in[23:20]), .enable(|in[31:20]), .out(out[46:40]));
  ssdec display7(.in(in[27:24]), .enable(|in[31:24]), .out(out[54:48]));
  ssdec display8(.in(in[31:28]), .enable(|in[31:28]), .out(out[62:56]));
endmodule

