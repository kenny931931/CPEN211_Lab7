module datapath(input clk, input [15:0] mdata, input [7:0] pc, input [1:0] wb_sel,
                input [2:0] w_addr, input w_en, input [2:0] r_addr, input en_A,
                input en_B, input [1:0] shift_op, input sel_A, input sel_B,
                input [1:0] ALU_op, input en_C, input en_status,
				input [15:0] sximm8, input [15:0] sximm5,
                output [15:0] datapath_out, output Z_out, output N_out, output V_out);
  
  reg [15:0] reg_A, reg_B, out, val_A, val_B, w_data;
  wire [15:0] r_data, shift_out, ALU_out;
  reg [2:0] flag_out;
  wire [2:0] flag;
  
  assign datapath_out = out;
  assign Z_out = flag[2];
  assign N_out = flag[1];
  assign V_out = flag[0];
  
  // register file
  regfile r(w_data, w_addr, w_en, r_addr, clk, r_data);
  // shifter
  shifter s(reg_B, shift_op, shift_out);
  // ALU
  ALU a(val_A, val_B, ALU_op, ALU_out, flag);
  
  always_ff @(posedge clk) begin
    // register with enable
    if (en_A) reg_A <= r_data;
	if (en_B) reg_B <= r_data;
	if (en_C) out <= ALU_out;
	// status
	if (en_status) flag_out <= flag;
  end
  
  always_comb begin
	// mux A
	if (sel_A) begin
	  val_A = 16'b0;
	end else begin
	  val_A = reg_A;
	end
	// mux B
	if (sel_B) begin
	  val_B = sximm5;
	end else begin
	  val_B = shift_out;
	end
	// mux C
	case (wb_sel)
	  2'b00 : w_data = out;
	  2'b01 : w_data = {8'b0, pc};
	  2'b10 : w_data = sximm8;
	  2'b11 : w_data = mdata;
	endcase
  end
endmodule: datapath
