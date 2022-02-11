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

  // Step 1.5
  // Instantiate the Lunar Lander and set up a slower clock
  logic hz4;
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
      temp <= (Q < 8'd2) ? 1 : 0;
  end
  
  always_ff @(posedge temp, posedge reset) begin
    if (reset == 1'b1) begin
      hz4 <= 0;
    end
    else begin
      hz4 <= ~hz4;
    end
  end
  
  always_comb begin
    if(Q == 8'd4) begin
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

  lunarlander #(16'h800, 16'h4500, 16'h0, 16'h5) ll (
        .hz100(hz100), .clk(hz4), .rst(reset), .in(pb[19:0]), .crash(red), .land(green),
        .ss({ss7, ss6, ss5, ss4, ss3, ss2, ss1, ss0})
      );
  
endmodule

module lunarlander #(
  parameter FUEL=16'h800,
  parameter ALTITUDE=16'h4500,
  parameter VELOCITY=16'h0,
  parameter THRUST=16'h5,
  parameter GRAVITY=16'h5
)(
  input logic hz100, clk, rst,
  input logic [19:0] in,
  output logic [63:0] ss,
  output logic crash, land
);
  // Created logic
  logic [15:0] alt;
  logic [15:0] vel;
  logic [15:0] fuel;
  logic [15:0] thrust;
  logic [15:0] newalt;
  logic [15:0] newvel;
  logic [15:0] intval;
  logic [15:0] newfuel;
  logic [15:0] manualthrust;
  logic clock;
  logic [4:0] scankeyout;

  // Step 1.1
  // Use your bcdaddsub4 module to calculate landing parameters
  bcdaddsub4 a1(.a(alt), .b(vel), .op(1'b0), .s(newalt));
  bcdaddsub4 a2(.a(vel), .b(GRAVITY), .op(1'b1), .s(intval));
  bcdaddsub4 a3(.a(intval), .b(thrust), .op(1'b0), .s(newvel));
  bcdaddsub4 a4(.a(fuel), .b(thrust), .op(1'b1), .s(newfuel));
  
  // Step 1.2
  // Set up a modifiable thrust register
  scankey b1(.clk(hz100), .rst(rst), .in(in), .out(scankeyout), .strobe(clock));
  
  always_ff @(posedge clock, posedge rst) begin
    if(rst)
      manualthrust <= THRUST;
    else
      if(scankeyout <= 5'b01001)
        manualthrust <= {11'b0,scankeyout};
  end
  
  // Step 1.3
  // Set up the state machine logic for the lander
  typedef enum logic [2:0] {INIT=0, CALC=1, SET=2, CHK=3, HLT=4} flight_t;
      logic [2:0] flight;
      logic nland, ncrash;
      
  always_ff @(posedge clk, posedge rst) begin
    if(rst) begin
      flight <= INIT;
      crash <= 1'b0;
      land <= 1'b0;
      ncrash <= 1'b0;
      nland <= 1'b0;
      fuel <= FUEL;
      alt <= ALTITUDE;
      vel <= VELOCITY;
      thrust <= THRUST;
    end
    
    else begin
      case (flight)
      
        INIT: flight <= CALC;
        
        CALC: flight <= SET;
        
        SET: 
        begin
          if(newfuel[15] == 1'b1)
            fuel <= 0;
          else
            fuel <= newfuel;
          
          alt <= newalt;
          vel <= newvel;
          if((newfuel[15] == 1'b1) | (fuel == 16'b0))
            thrust <= 0;
          else
            thrust <= manualthrust;
          flight <= CHK;
        end
        
        CHK:
        begin
          if((newalt[15] == 1) & (thrust <= 5) & (newvel > 16'h9970)) begin
            nland <= 1;
            flight <= HLT;
          end
          else if(newalt[15] == 1) begin
            ncrash <= 1;
            flight <= HLT;
          end
          else
            flight <= CALC;
        end
        
        HLT:
        begin
          land <= nland;
          crash <= ncrash;
          alt <= 0;
          vel <= 0;
        end
      endcase
    end
  end
  // Step 1.4
  // Set up the display mechanics
      logic [23:0] lookupmsg [3:0];
      logic [1:0] sel;
      logic [15:0] val;
      logic [15:0] negval;
      logic [63:0] valdisp, negvaldisp;
      
      bcdaddsub4 c1(.a(0), .b(val), .op(1'b1), .s(negval));
      
      display_32_bit c2(.in({16'b0,val}), .out(valdisp));
      display_32_bit c3(.in({16'b0,negval}), .out(negvaldisp));
      
      always_comb begin
        if(val[15] == 1)
          ss = {lookupmsg[sel], 8'b0, 8'b01000000, negvaldisp[23:0]};
        else
          ss = {lookupmsg[sel], 8'b0, valdisp[31:0]};
      end
      
      always_comb begin
        lookupmsg[0] = 24'b011101110011100001111000;  // alt
        lookupmsg[1] = 24'b001111100111100100111000;  // vel
        lookupmsg[2] = 24'b011011110111011101101101;  // fuel (says gas)
        lookupmsg[3] = 24'b011110000111011001010000;  // thrust
        
        case(sel)
          0: val = alt;
          1: val = vel;
          2: val = fuel;
          3: val = thrust;
        endcase
      end
      
      always_ff @(posedge clock, posedge rst) begin
        if(rst)
          sel <= 0;
        else begin
          if(scankeyout == 5'b10000)
            sel <= 3;
          else if(scankeyout == 5'b10001)
            sel <= 2;
          else if(scankeyout == 5'b10010)
            sel <= 1;
          else if(scankeyout == 5'b10011)
            sel <= 0;
        end
      end
      
  

endmodule

module ha(input logic a, input logic b, output logic s, output logic co);
  assign s = a ^ b;
  assign co = a & b;
endmodule

module faha(input logic a, input logic b, input logic ci, output logic s, output logic co);
  logic cout1;
  logic cout2;
  logic s1;
  ha v1(.a(a), .b(b), .s(s1), .co(cout1));
  ha v2(.a(s1), .b(ci), .s(s), .co(cout2));
  assign co = cout1 | cout2;

endmodule

module fa(input logic a, input logic b, input logic ci, output logic s, output logic co);
  assign s = a ^ b ^ ci;
  assign co = (a & b) | (a & ci) | (b & ci);
endmodule

module fa4(input logic [3:0] a, input logic [3:0] b, input logic ci, output logic [3:0] s, output logic co);
  logic cout0;
  logic cout1;
  logic cout2;
  logic cout3;
  fa s0(.a(a[0]), .b(b[0]), .ci(ci), .co(cout0), .s(s[0]));
  fa s1(.a(a[1]), .b(b[1]), .ci(cout0), .co(cout1), .s(s[1]));
  fa s2(.a(a[2]), .b(b[2]), .ci(cout1), .co(cout2), .s(s[2]));
  fa s3(.a(a[3]), .b(b[3]), .ci(cout2), .co(cout3), .s(s[3]));
  assign co = cout3;
  
endmodule

module cla8(input logic [7:0] a, input logic [7:0] b, input logic ci, output logic [7:0] s, output logic co);
  logic [7:0] p;
  logic [7:0] g;
  logic [7:0] cout;
  
  ha a1(.a(a[0]), .b(b[0]), .s(p[0]), .co(g[0]));
  ha a2(.a(a[1]), .b(b[1]), .s(p[1]), .co(g[1]));
  ha a3(.a(a[2]), .b(b[2]), .s(p[2]), .co(g[2]));
  ha a4(.a(a[3]), .b(b[3]), .s(p[3]), .co(g[3]));
  ha a5(.a(a[4]), .b(b[4]), .s(p[4]), .co(g[4]));
  ha a6(.a(a[5]), .b(b[5]), .s(p[5]), .co(g[5]));
  ha a7(.a(a[6]), .b(b[6]), .s(p[6]), .co(g[6]));
  ha a8(.a(a[7]), .b(b[7]), .s(p[7]), .co(g[7]));
  
  assign cout[0] = g[0] | (ci & p[0]);
  assign cout[1] = g[1] | (g[0] & p[1]) | (ci & p[0] & p[1]);
  assign cout[2] = g[2] | (g[1] & p[2]) | (g[0] & p[1] & p[2]) | (ci & p[0] & p[1] & p[2]);
  assign cout[3] = g[3] | (g[2] & p[3]) | (g[1] & p[2] & p[3]) | (g[0] & p[1] & p[2] & p[3]) | (ci & p[0] & p[1] & p[2] & p[3]);
  assign cout[4] = g[4] | (g[3] & p[4]) | (g[2] & p[3] & p[4]) | (g[1] & p[2] & p[3] & p[4]) | (g[0] & p[1] & p[2] & p[3] & p[4]) | (ci & p[0] & p[1] & p[2] & p[3] & p[4]);
  assign cout[5] = g[5] | (g[4] & p[5]) | (g[3] & p[4] & p[5]) | (g[2] & p[3] & p[4] & p[5]) | (g[1] & p[2] & p[3] & p[4] & p[5]) | (g[0] & p[1] & p[2] & p[3] & p[4] & p[5]) | (ci & p[0] & p[1] & p[2] & p[3] & p[4] & p[5]);
  assign cout[6] = g[6] | (g[5] & p[6]) | (g[4] & p[5] & p[6]) | (g[3] & p[4] & p[5] & p[6]) | (g[2] & p[3] & p[4] & p[5] & p[6]) | (g[1] & p[2] & p[3] & p[4] & p[5] & p[6]) | (g[0] & p[1] & p[2] & p[3] & p[4] & p[5] & p[6]) | (ci & p[0] & p[1] & p[2] & p[3] & p[4] & p[5] & p[6]);
  assign cout[7] = g[7] | (g[6] & p[7]) | (g[5] & p[6] & p[7]) | (g[4] & p[5] & p[6] & p[7]) | (g[3] & p[4] & p[5] & p[6] & p[7]) | (g[2] & p[3] & p[4] & p[5] & p[6] & p[7]) | (g[1] & p[2] & p[3] & p[4] & p[5] & p[6] & p[7]) | (g[0] & p[1] & p[2] & p[3] & p[4] & p[5] & p[6] & p[7]) | (ci & p[0] & p[1] & p[2] & p[3] & p[4] & p[5] & p[6] & p[7]);


  assign s[0] = ci ^ p[0];
  assign s[7:1] = cout[6:0] ^ p[7:1];
  assign co = cout[7];

endmodule

module addsub8(input logic [7:0] a, input logic [7:0] b, input logic op, output logic [7:0] s, output logic co);
  logic [7:0] xordb;
  assign xordb = b ^ {op, op, op, op, op, op, op, op};
  cla8 s1(.a(a), .b(xordb), .ci(op), .s(s), .co(co));
endmodule

module bcdadd1(input logic [3:0] a, input logic [3:0] b, input logic ci, output logic co, output logic [3:0] s);
  logic first0;
  logic first1;
  logic first2;
  logic first3;
  
  logic [3:0] second;
  logic correction;
  logic cout;
  fa4 fa(.a(a), .b(b), .ci(ci), .s({first3,first2,first1,first0}), .co(cout));
  assign correction = cout | (first3 & first2) | (first3 & first1);
  assign second = {1'b0,correction,correction,1'b0};
  fa4 fa2(.a(second), .b({first3,first2,first1,first0}), .ci(1'b0), .s(s));
  assign co = correction;
  
endmodule

module bcdadd4(input logic [15:0] a, input logic [15:0] b, input logic ci, output logic co, output logic [15:0] s);
  logic cout0;
  logic cout1;
  logic cout2;
  logic cout3;
  bcdadd1 aa1(.a(a[3:0]), .b(b[3:0]), .ci(ci), .co(cout0), .s(s[3:0]));
  bcdadd1 aa2(.a(a[7:4]), .b(b[7:4]), .ci(cout0), .co(cout1), .s(s[7:4]));
  bcdadd1 aa3(.a(a[11:8]), .b(b[11:8]), .ci(cout1), .co(cout2), .s(s[11:8]));
  bcdadd1 aa4(.a(a[15:12]), .b(b[15:12]), .ci(cout2), .co(cout3), .s(s[15:12]));

  assign co = cout3;

endmodule

module bcd9comp1(input logic [3:0] in, output logic [3:0] out);
  always_comb begin
    case (in)
      4'b0000: out = 4'b1001;
      4'b0001: out = 4'b1000;
      4'b0010: out = 4'b0111;
      4'b0011: out = 4'b0110;
      4'b0100: out = 4'b0101;
      4'b0101: out = 4'b0100;
      4'b0110: out = 4'b0011;
      4'b0111: out = 4'b0010;
      4'b1000: out = 4'b0001;
      4'b1001: out = 4'b0000;
      default: out = 4'b0000;
    endcase
  end
endmodule

module bcdaddsub4(input logic [15:0] a, input logic [15:0] b, input logic op, output logic [15:0] s);
  
  logic [3:0] cout;
  logic [15:0] actualb;
  logic [15:0] complementb;
  bcd9comp1 cmpa(.in(b[3:0]), .out(complementb[3:0]));
  bcd9comp1 cmpb(.in(b[7:4]), .out(complementb[7:4]));
  bcd9comp1 cmpc(.in(b[11:8]), .out(complementb[11:8]));
  bcd9comp1 cmpd(.in(b[15:12]), .out(complementb[15:12]));
  
  always_comb begin
    if(op == 1'b1)
      actualb = complementb;
    else
      actualb = b;
  end
  
  bcdadd4 usaa1(.a(a[15:0]), .b(actualb[15:0]), .ci(op), .co(cout[0]), .s(s[15:0]));

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
