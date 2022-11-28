`timescale 1 ps / 1 ps

module tb_task3(output err);
  // your implementation here
  
  reg outErr = 0;
  
  integer num_passes = 0;
  integer num_fails = 0;
  
  reg clk, rst_n;
  reg [7:0] start_pc;
  reg signed [15:0] out;
  
  task3 DUT(.clk(clk), .rst_n(rst_n), .start_pc(start_pc), .out(out));
  
  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end
  
  task restartf();
    rst_n = 1'b0;#5;
	rst_n = 1'b1;#5;
  endtask
  
  task test(input signed [15:0] exp);
    #160; 
	    
	assert(out === exp) begin
	  $display("[PASS]: val is %-d", exp);
	  num_passes = num_passes + 1;
	end else begin
	  $error("[FAIL] val is %-d (expected %-d)", out, exp);
	  outErr = 1'b1;
	  num_fails = num_fails+1;
	end
  endtask

  task test2(input signed [15:0] exp, int delay);
    #30; // 3 clock cycle for fetch
	#delay; // variable delay for execution
	assert(out === exp) begin
	  $display("[PASS]: val is %-d", exp);
	  num_passes = num_passes + 1;
	end else begin
	  $error("[FAIL] val is %-d (expected %-d)", out, exp);
	  outErr = 1'b1;
	  num_fails = num_fails+1;
	end
  endtask

  //===== TEST =====
  initial begin
    #1; // offset timing
	
	$display("\n=== TEST 1 ===");
    begin
      // test 1
	  /*
	    INSTRUCTION				CYCLE
	    @00 MOV R0, #78			3+2 = 5
	    @01 MOV R1, R0, LSR		3+3 = 6
	    @02 AND R2, R0, R1		3+4 = 7
	    @03 HALT				inf
	  */
	  
	  restartf();
	  start_pc = 8'h2a;
    #1040; 
	  
	test(16'b0000000000000000);
    test(16'b0000000000000001);
    test(16'b0000000000000010);
    test(16'b0000000000000011);
    test(16'b0000000000000100);
    test(16'b0000000000000101);
    test(16'b0000000000000110);
    test(16'b0000000000000111);

	#50;
    end  
	
	  $display("\n=== TEST 2 ===");
    begin
      // test 2
	  /*
	    INSTRUCTION				CYCLE
	    @b0 MOV R0, #d0			3+2 = 5
	    @b1 MOV R1, #d0			3+2 = 5
		@b2 MOV R2, #d0			3+2 = 5
	    @b3 MOV R3, #d0			3+2 = 5
		@b4 MOV R4, #78			3+2 = 5
	    @b5 MOV R5, #39			3+2 = 5
		@b6 MOV R6, #98			3+2 = 5
	    @b7 MOV R7, #-126		3+2 = 5
		@b8 STR R4, [R0, #0]	3+5 = 8
		@b9 STR R5, [R1, #1]	3+5 = 8
		@ba STR R6, [R2, #2]	3+5 = 8
		@bb STR R7, [R3, #3]	3+5 = 8
		@bc LDR R0, [R0, #0]	3+6 = 9
		@bd LDR R1, [R1, #1]	3+6 = 9
		@be LDR R2, [R2, #2]	3+6 = 9
		@bf LDR R3, [R3, #3]	3+6 = 9
		@c0 MOV R0, R0			3+4 = 7
		@c1 MOV R1, R1			3+4 = 7
		@c2 MOV R2, R2			3+4 = 7
		@c3 MOV R3, R3			3+4 = 7
	    @c4 HALT				inf
		d0 - d3 : store data
	  */  
	  
	  start_pc = 8'hb0;
	  restartf();
	  #1080;
	  test2(16'd78, 40);
	  test2(16'd39, 40);
	  test2(16'd98, 40);
	  test2(-16'd126, 40);
	  test2(-16'd126, 110);
	  test2(-16'd126, 110);
    end  
	
    $display("\n\n==== TEST SUMMARY ====");
    $display("  TEST COUNT: %-5d", num_passes + num_fails);
    $display("    - PASSED: %-5d", num_passes);
    $display("    - FAILED: %-5d", num_fails);
    $display("======================\n\n");
    $stop;
  end
  
  assign err = outErr;
  
endmodule: tb_task3
