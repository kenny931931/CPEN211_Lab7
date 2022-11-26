module ALU(input [15:0] val_A, input [15:0] val_B, input [1:0] ALU_op, output [15:0] ALU_out, output [2:0] flag);
  reg [15:0] result;
  reg [2:0] state;

  assign ALU_out = result;
  assign flag = state;
  always_comb begin
    // get result
    case(ALU_op)
	  2'b00 : result = val_A + val_B; // Addition
	  2'b01 : result = val_A - val_B; // Subtraction
	  2'b10 : result = val_A & val_B; // Bitwise AND
	  2'b11 : result = ~val_B;		  // bitwise negation
	endcase
	// get Z
	// if result = 0
	if (result == 16'b0)
	  state = 3'b100;
	// if result is -ve
	else if (result[15] == 1'b1)
	  state = 3'b010;
	else
	// default case
	  state = 3'b0;
	
	// if result is overflow (+ve + +ve = -ve OR -ve + -ve = +ve) or (-ve - +ve = +ve OR +ve - -ve = -ve)
	if (ALU_op == 2'b00 && (({val_A[15], val_B[15], result[15]} == 3'b001) || ({val_A[15], val_B[15], result[15]} == 3'b110)))
	  state[0] = 3'b1;
	if (ALU_op == 2'b01 && (({val_A[15], val_B[15], result[15]} == 3'b100) || ({val_A[15], val_B[15], result[15]} == 3'b011)))
	  state[0] = 3'b1;
	  
  end
endmodule: ALU
