////////////////////////////////////////////////////////////////////// 
// YOU SHOULD NOT MODIFY THIS FILE.
// WE MIGHT HAVE TO CONSIDER IT ACADEMIC DISHONESTY.
//////////////////////////////////////////////////////////////////////

module lab_testbench ();
typedef enum logic { RDY, ENT } simonstate_t;
logic clk, rst, mode, enable;
logic [7:0] ctr;
logic hz1, hz1clk, scanclk;
logic hz100clk;
logic [19:0] in;
logic [4:0]keycode;
logic btnclk, pst, state;

logic [4:0]lookup[0:19];
logic lvlmax, win, lose;
logic en, clr;
logic [31:0] out, tempreg;
logic [4:0]innum;

clock_1hz c2h (.hz100(hz100clk), .reset(rst), .hz1(hz1));
scankey sk (.clk(hz1clk), .rst(rst), .in(in), .out(keycode), .strobe(btnclk));
simonctl ctrl (.clk(scanclk), .rst(rst), .lvlmax(lvlmax), .win(win), .lose(lose), .state(state));
numentry ne (.clk(scanclk), .rst(rst), .en(en), .in(innum), .out(out), .clr(clr));

integer STDERR = 32'h8000_0002;
logic errored;
integer count_cycle;

logic [7:0] ans;
logic [1023:0] testname;

task automatic clock(integer n);
    while (n != 0) begin
        clk = 1'b1;
        #1;
        clk = 1'b0;
        #1;
        n--;
    end
endtask //automatic

integer i;
initial 
begin
hz100clk = 0;
count_cycle = 0;
hz1clk = 0;
scanclk = 0;
lookup[0] = 5'b00000;
lookup[1] = 5'b00001;
lookup[2] = 5'b00010;
lookup[3] = 5'b00011;
lookup[4] = 5'b00100;
lookup[5] = 5'b00101;
lookup[6] = 5'b00110;
lookup[7] = 5'b00111;
lookup[8] = 5'b01000;
lookup[9] = 5'b01001;
lookup[10] = 5'b01010;
lookup[11] = 5'b01011;
lookup[12] = 5'b01100;
lookup[13] = 5'b01101;
lookup[14] = 5'b01110;
lookup[15] = 5'b01111;
lookup[16] = 5'b10000;
lookup[17] = 5'b10001;
lookup[18] = 5'b10010;
lookup[19] = 5'b10011;
end

always #5000000 hz100clk = ~hz100clk;
always #500000000 hz1clk = ~hz1clk;
always #125000000 scanclk = ~scanclk;


initial begin
    $dumpfile ("lab10.vcd");
    $dumpvars (0, lab_testbench);
    errored = 0;
    // put tests here
    testname = "count1hz Reset Test";
    rst = 1'b1;
    repeat (10) begin
    @(posedge hz100clk) 
     if (hz1 != 0) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR: Expected hz1 = 0 when reset = 1"); 
     end
    end

fork 
begin
    testname = "count1hz Freq Test";
    rst = 1'b0;
    fork
    begin
    @(posedge hz1); 
     forever begin
      @(posedge hz100clk); 
      count_cycle++;
     end
    end
    begin
    @(negedge hz1); 
    end
    begin
     #5000000000; //timeout
    end
    join_any
     if (count_cycle != 50) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR: Expected frequency 1Hz Wrong Counter value",count_cycle); 
     end

    testname = "scan key Reset Test";
    rst = 1'b1;
    in = 20'b1;
    repeat (10) begin
    @(posedge hz1clk); 
     if (btnclk != 0) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR: Expected ff = 0 when reset = 1"); 
     end
    end

    testname = "scan key Func Test";
    rst = 1'b0;
    for (i = 0;i<20;i++) begin
    rst = 1'b1;
    #50;
    rst = 1'b0;
    in[i] = 1;
    #10;
    fork 
     begin
      if(keycode!=lookup[i]) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR: Key press Expected ",lookup[i]," got ",keycode); 
      end
     end
     begin
      @(posedge hz1clk) 
      #100;
      if (btnclk != 0) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR: Expected ff[1] = 0 for a key press"); 
      end
      @(posedge hz1clk) 
      #100;  
      if (btnclk != 1) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR: Expected ff[1] = 1 for a key press"); 
      end
     end
     join
    in = 20'b0;
    end


    testname = "simonctl Reset Test";
    rst = 1'b1;
    repeat (10) begin
    @(posedge scanclk) 
     if (state != RDY) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR: Expected State = RDY when reset = 1"); 
     end
    end
    
    testname = "simonctl func Test";
    rst = 1'b0;
     lose = 0;
     lvlmax = 0;
     win = 0;
     @(posedge scanclk);
     #100;
     if (state != ENT) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR 0: Expected State = ENT ", state); 
     end
     pst = state; 
     #100;
     lose = 0;
     lvlmax = 0;
     win = 1;
     @(posedge scanclk) 
     #100;
     if (state != RDY) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR 1: Expected State = RDY"); 
     end
     pst = state; 
     #100;
     lose = 0;
     lvlmax = 1;
     win = 0;
     @(posedge scanclk) 
     #100;
     if (state != pst) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR 2: Expected Previous State to be retained"); 
     end
     pst = state; 
     #100;
     lose = 0;
     lvlmax = 1;
     win = 1;
     @(posedge scanclk) 
     #100;
     if (state != pst) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR 3: Expected Previous State to be retained"); 
     end
     pst = state; 
     #100;
     lose = 1;
     lvlmax = 0;
     win = 0;
     @(posedge scanclk) 
     #100;
     if (state != RDY) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR 4: Expected State = RDY"); 
     end
     pst = state; 
     #100;
     lose = 1;
     lvlmax = 0;
     win = 1;
     @(posedge scanclk) 
     #100;
     if (state != RDY) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR 5: Expected State = RDY"); 
     end
     pst = state; 
     #100;
     lose = 1;
     lvlmax = 1;
     win = 0;
     @(posedge scanclk) 
     #100;
     if (state != RDY) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR 6: Expected State = RDY"); 
     end
     pst = state; 
     #100;
     lose = 1;
     lvlmax = 1;
     win = 1;
     @(posedge scanclk) 
     #100;
     if (state != RDY) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR 7: Expected State = RDY"); 
     end
     pst = state; 
     #100;

     testname = "numentry Reset Test";
     rst = 1'b1;
     repeat (10) begin
     @(posedge scanclk) 
      if (out != 32'h0) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR: Expected out = 32'h0 when reset = 1"); 
      end
     end

     testname = "numentry clr Test";
     rst = 1'b0;
     clr = 1'b1;
     repeat (10) begin
     @(posedge scanclk) 
      if (out != 32'h0) begin
        errored = 1;
        $fdisplay(STDERR, "ERROR: Expected out = 32'h0 when clr = 1"); 
      end
     end

     testname = "numentry func Test";
     rst = 1'b0;
     clr = 1'b0;
     en = 1'b1;
     tempreg = 32'h0;
     for (i = 9;i >= 0; i--) begin
     innum = lookup[i];
     tempreg = (innum | (tempreg << 4));
     	@(posedge scanclk);
     #100;
        if (out != tempreg) begin
        	errored = 1;
        	$fdisplay(STDERR, "ERROR ", 9-i," Expected out ", tempreg, " got ", out); 
     	end
     end
     innum = lookup[10];
     	@(posedge scanclk);
     #100;
        if (out != tempreg) begin
        	errored = 1;
        	$fdisplay(STDERR, "ERROR Expected out ", tempreg, " got ", out); 
     	end
    
     testname = "numentry clr func Test";
     rst = 1'b0;
     clr = 1'b1;
     en = 1'b1;
     tempreg = 32'h0;
     for (i = 9;i >= 0; i--) begin
     innum = lookup[i];
     tempreg = (innum | (tempreg << 4));
     	@(posedge scanclk);
     #100;
        if (out != 32'h0) begin
        	errored = 1;
        	$fdisplay(STDERR, "ERROR Expected out 0 since clr = 1 "); 
     	end
     end
    
     testname = "numentry en func Test";
     rst = 1'b0;
     clr = 1'b0;
     en = 1'b0;
     tempreg = 32'h0;
     for (i = 9;i >= 0; i--) begin
     innum = lookup[i];
     tempreg = (innum | (tempreg << 4));
     	@(posedge scanclk);
     #100;
        if (out != 32'h0) begin
        	errored = 1;
        	$fdisplay(STDERR, "ERROR Expected out 0 since en = 0 "); 
     	end
     end
    
    if (errored == 0)
        $finish;
    else
        $stop;
end
begin
#3000000000;
end
join_any

end

endmodule
