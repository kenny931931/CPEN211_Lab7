module tb_task3(output err);
  // your implementation here
  
  reg outErr = 0;
  
  integer num_passes = 0;
  integer num_fails = 0;
  
  reg clk, rst_n;
  reg [7:0] start_pc;
  reg signed [15:0] out;
  
  task1 DUT(.clk(clk), .rst_n(rst_n), .start_pc(start_pc), .out(out));
  
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
	  start_pc = 8'h29;
    #1040; 
	  
	  test(16'b0000000000000000);
    test(16'b0000000000000001);
    test(16'b0000000000000010);
    test(16'b0000000000000011);
    test(16'b0000000000000100);
    test(16'b0000000000000101);
    test(16'b0000000000000110);
    test(16'b0000000000000111);

    end  
	
	// $display("\n=== TEST 2 ===");
  //   begin
  //     // test 2
	//   /*
	//     INSTRUCTION				CYCLE
	//     @04 MOV R0, #56			3+2 = 5
	// 	@05 MOV R1, #-17		3+2 = 5
	// 	@06 ADD R0, R0, R1		3+4 = 7
	// 	@07 MVN R2, R0			3+3 = 5
	// 	@08 AND R3, R2, R1		3+4 = 7
	// 	@09 MOV R4, R3			3+3 = 6
	// 	@0a HALT				inf
	//   */
	  
	//   restartf();
	//   start_pc = 8'h04;
	//   test(16'd6, 20);
	//   test(16'd6, 20);
	//   test(16'd39, 50);
	//   test(-16'd40, 40);
	//   test(-16'd56, 50);
	//   test(-16'd56, 40);
	//   test(-16'd56, 110);
	//   test(-16'd56, 110);
  //   end
	
    $display("\n\n==== TEST SUMMARY ====");
    $display("  TEST COUNT: %-5d", num_passes + num_fails);
    $display("    - PASSED: %-5d", num_passes);
    $display("    - FAILED: %-5d", num_fails);
    $display("======================\n\n");
    $stop;
  end
  
  assign err = outErr;
  
endmodule: tb_task3
