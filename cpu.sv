module cpu(input clk, input rst_n, input start, input [15:0] instr, input [7:0] start_pc,, input [15:0] ram_r_data,
           output waiting, output [15:0] out, output RAM_w_en, output [7:0] RAM_addr, output N, output V, output Z);
  // your implementation here
  reg [15:0] f_instr;
  reg [7:0] pc, next_pc, d_addr, ram_addr;
  wire [15:0] sximm5, sximm8, d_out;
  wire [2:0] opcode, r_addr, w_addr;
  wire [1:0] reg_sel, wb_sel, op, shift_op; //op = ALU_op OR op
  wire z, n, v, w, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, load_ir, load_pc, clear_pc, load_addr, ram_w_en;
  
  assign out = d_out;
  assign Z = z;
  assign N = n;
  assign V = v;
  assign waiting = w;
  assign RAM_w_en = ram_w_en;
  assign RAM_addr = ram_addr;
  
  // Instruction decoder
  idecoder i(f_instr, reg_sel,
        opcode, op, shift_op,
		sximm5, sximm8,
		r_addr, w_addr);
		
  // Controller FSM
  controller c(clk, rst_n, start,
        opcode, op, shift_op,
		z, n, v,
		w,
		reg_sel, wb_sel, w_en, load_ir, load_pc, clear_pc, load_addr,
		en_A, en_B, en_C, en_status, ram_w_en,
		sel_A, sel_B);
		
  // Modified datapath
  datapath d(clk, ram_r_data, pc, wb_sel,
        w_addr, w_en, r_addr, en_A,
		en_B, shift_op, sel_A, sel_B,
		op, en_C, en_status,
		sximm8, sximm5,
		d_out, z, n, v);
  
  always_ff @(posedge clk) begin
    // Load instruction (2)
	if (load_ir)
	  f_instr <= instr;
	  
	// Load PC(3)
	if (load_pc)
	  pc <= next_pc;
	  
	// Load d_addr(5)
	if (load_addr)
	  d_addr <= d_out[7:0];
  end
  
  always_comb begin
    // sel PC (4)
	next_pc = clear_pc ? start_pc : pc + 1'b1;
	
	// sel ram_r_addr
	ram_addr = sel_addr ? pc : d_addr;
  end
endmodule: cpu
