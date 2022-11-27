module task3(input clk, input rst_n, input [7:0] start_pc, output[15:0] out);
  // your implementation here
  wire ram_w_en, N, V, Z;
  wire [7:0] ram_addr;
  wire [15:0] ram_r_data, ram_w_data;
  
  cpu c(clk, rst_n, start_pc, ram_r_data,
	  waiting, out, ram_w_en, ram_addr, N, V, Z);
  
  ram r(clk, ram_w_en, ram_addr, ram_addr, ram_w_data, ram_r_data);
  
endmodule: task3

module cpu(input clk, input rst_n, input [7:0] start_pc, input [15:0] ram_r_data,
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
  controller c(clk, rst_n,
        opcode, op, shift_op,
		z, n, v,
		
		reg_sel, wb_sel, w_en, load_ir, load_pc, clear_pc, load_addr,
		en_A, en_B, en_C, en_status, ram_w_en,
		sel_A, sel_B, sel_addr);
		
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
	  f_instr <= ram_r_data;
	  
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

module idecoder(input [15:0] ir, input [1:0] reg_sel,
                output reg [2:0] opcode, output reg[1:0] ALU_op, output reg[1:0] shift_op,
				output reg [15:0] sximm5, output reg [15:0] sximm8,
                output reg[2:0] r_addr, output reg[2:0] w_addr);
  
   always_comb begin
        
        opcode = ir[15:13];
        ALU_op = ir[12:11];

        //sign extended values
		sximm5 = ir[4] == 1 ? {-11'b1,ir[4:0]} : {11'b0,ir[4:0]};
		sximm8 = ir[7] == 1 ? {-8'b1,ir[7:0]} : {8'b0,ir[7:0]};

        shift_op = ir[4:3];

        case(reg_sel)
                2'b10: {r_addr,w_addr} = {ir[10:8],ir[10:8]};
                2'b01: {r_addr,w_addr} = {ir[7:5],ir[7:5]};
                2'b00: {r_addr,w_addr} = {ir[2:0],ir[2:0]};
                default: {r_addr,w_addr} = {6'b000000};
        endcase
            

  end
endmodule: idecoder

module controller(input clk, input rst_n,
                  input [2:0] opcode, input [1:0] ALU_op, input [1:0] shift_op,
                  input Z, input N, input V,
            
                  output reg [1:0] reg_sel, output reg[1:0] wb_sel, output reg w_en, output reg load_ir,
                  output reg load_pc, output reg clear_pc, output reg load_addr,
                  output reg en_A, output reg en_B, output reg en_C, output reg en_status, output reg ram_w_en,
                  output reg sel_A, output reg sel_B, output reg sel_addr);


`define enable 4'd0
`define loadB 4'd1
`define loadA 4'd2
`define movI_one 4'd3
`define finish 4'd4

`define mov1 4'd5
`define mov2 4'd6

`define cal 4'd7

`define wait 4'd8

//after reset states
`define rst1 4'd9
`define rst2 4'd10
`define rst3 4'd11
`define rst4 4'd12

`define loadAddr 4'd13
`define loadRAM 4'd14

reg [3:0] next;
reg signal =0;
reg [3:0] state;
wire [4:0] instruction;
assign state = next;
reg waiting; 
reg halt; 
assign instruction = {opcode,ALU_op};


always_ff @( posedge clk ) begin 

    //if reset is hit, begin the reset fetch sequence 
		if (~rst_n) begin 
        signal <= 1'b0;
        en_status <= 0; //maybe change
        next <= `rst1;
        w_en <= 1'b0;
         wb_sel <= 2'b00;
          en_A <= 1'b0;
         en_B <= 1'b0;
         sel_A <= 1'b0;
         sel_B <= 1'b0;
         en_status <= 1'b0;
         en_C <= 1'b0;
         waiting <= 1'b0;
         load_addr <= 1'b0;
         halt <= 1;
         
        end



        //reset fetch sequence 
        if (~signal) begin

        case (state)
        `rst1 : {next, load_pc, clear_pc, sel_addr, ram_w_en} <= {`rst2, 1'b1,1'b1,1'b1,1'b0};
        `rst2 : {next, load_pc, clear_pc} <= {`rst3, 1'b0, 1'b0};
        `rst3 : {next, load_ir} <= {`rst4, 1'b1};
        `rst4 : {next, load_ir, signal} <= {`wait, 1'b0,1'b1};
        endcase
        end

		else if (halt) begin
        
     
        casex (instruction)

//move immediate
				5'b11010 : begin

          case (state)
              `wait : {next,reg_sel,w_en,wb_sel,waiting} <= {`movI_one, 2'b10,1'b1,2'b10, 1'b0};
              `movI_one : {waiting, signal,w_en, load_pc, signal, next} <= {1'b1,1'b0,1'b0, 1'b1, 1'b0, `rst2};
            
          endcase
        end

//MOV (1 reg to another)
            5'b11000 : begin
              case (state)
                `wait : {next, reg_sel, en_B,waiting} <= {`mov1,2'b00,1'b1,1'b0};
                `mov1 : {next,sel_B,sel_A,en_C} <= {`mov2,1'b0,1'b1,1'b1};
                `mov2 : {next, wb_sel,w_en,reg_sel, en_C,en_B} <= {`finish ,2'b00,1'b1,2'b01,1'b0,1'b0};
                `finish : {waiting, signal, en_C, w_en, load_pc, signal, next} <= { 1'b1,1'b0, 1'b0,1'b0, 1'b1, 1'b0, `rst2};
              endcase

            end
//ADD/AND           
            5'b101x0 : begin
            case (state)
            `wait : {next,reg_sel,en_A,en_B,waiting} <= {`loadB, 2'b00,1'b0,1'b1,1'b0};
            `loadB : {next,reg_sel,en_A,en_B} <= {`loadA, 2'b10, 1'b1,1'b0};
            `loadA : {next, sel_A,sel_B,en_C} <= {`cal, 1'b0,1'b0,1'b1};
            `cal : {next, wb_sel,w_en,reg_sel,en_A,en_C,en_B} <= {`finish, 2'b00, 1'b1,2'b01,1'b0,1'b0,1'b0};
            `finish : { waiting, signal, w_en, load_pc, next} <= {1'b1,1'b0, 1'b0, 1'b1, `rst2};
            endcase

              
            end
//CMP
            5'b10101 : begin
              case (state)
            `wait : {next,reg_sel,en_A,en_B,waiting} <= {`loadB, 2'b00,1'b0,1'b1,1'b0};
            `loadB : {next,reg_sel,en_A,en_B} <= {`loadA, 2'b10, 1'b1,1'b0};
            `loadA : {next, sel_A,sel_B, en_status} <= {`enable, 1'b0,1'b0,1'b1}; //status should output on the next rising edge, waiting goes high early
            `enable : { en_status, signal,en_A,en_C,en_B,waiting, load_pc, signal, next} <= { 1'b0, 1'b0,1'b0,1'b0,1'b0,1'b1, 1'b1, `rst2};


              endcase

            end
//MVN           
            5'b10111 : begin
              case (state)
              `wait : {next,reg_sel,en_A,en_B,waiting} <= {`loadB, 2'b00,1'b0,1'b1,1'b0};
              `loadB : {next, sel_A,sel_B,en_C} <= {`cal, 1'b0,1'b0,1'b1};
              `cal : {next, wb_sel,w_en,reg_sel} <= {`finish, 2'b00, 1'b1,2'b01};
              `finish : { waiting, signal, w_en,en_A,en_C,en_B, load_pc, next} <= { 1'b1,1'b0,1'b0,1'b0,1'b0,1'b0, 1'b1, `rst2};
              endcase
            end

//Halt
            5'b11100 : begin
            
            case (state)
            `wait : {next, halt} <= {`wait, 1'b0};
            endcase
			end

//LDR
			5'b01100 : begin
			  case (state)
			  `wait : {next, reg_sel, en_A, sel_B, waiting} <= {`loadA, 2'b10, 1'b1, 1'b1, 1'b0};
			  `loadA : {next, en_A, en_C, sel_B} <= {`cal, 1'b0, 1'b1, 1'b0};
			  `cal : {next, en_C, load_addr, sel_addr} <= {`loadAddr, 1'b0, 1'b1, 1'b0};
			  `loadAddr : {next, load_addr, sel_addr} <= {`loadRAM, 1'b0, 1'b1};
			  `loadRAM : {next,reg_sel,w_en,wb_sel} <= {`movI_one, 2'b01,1'b1,2'b11};
			  `movI_one : {next, w_en, waiting} <= {`rst2, 1'b0, 1'b1};
			  endcase
            end
			
//STR
			5'b10000 : begin
			  case (state)
			  `wait : {next, reg_sel, en_A, sel_B, waiting} <= {`loadA, 2'b10, 1'b1, 1'b1, 1'b0};
			  `loadA : {next, en_A, en_C, sel_B} <= {`cal, 1'b0, 1'b1, 1'b0};
			  `cal : {next, en_C, load_addr, sel_addr} <= {`loadAddr, 1'b0, 1'b1, 1'b0};
			  `loadAddr : {next, load_addr, reg_sel, en_A} <= {`mov1, 1'b0, 2'b01, 1'b1};
			  `mov1 : {next, en_A, en_C} <= {`mov2, 1'b0, 1'b1};
			  `mov2 : {next, en_C, ram_w_en} <= {`loadRAM, 1'b0, 1'b1};
			  `loadRAM : {next, ram_w_en, waiting} <= {`rst2, 1'b0, 1'b1};
			  endcase
            end

            default : next <= `wait; //might be an issue
            
            
			endcase
    end 
	end
    

endmodule: controller

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
  regfile r(clk, w_data, w_addr, w_en, r_addr, r_data);
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

module regfile(input logic clk, input logic [15:0] w_data, input logic [2:0] w_addr, input logic w_en, input logic [2:0] r_addr, output logic [15:0] r_data);
    logic [15:0] m[0:7];
    assign r_data = m[r_addr];
    always_ff @(posedge clk) if (w_en) m[w_addr] <= w_data;
endmodule: regfile

module shifter(input [15:0] shift_in, input [1:0] shift_op, output reg [15:0] shift_out);
  // your implementation here
  always_comb begin
    case (shift_op)
    2'b00 : shift_out = shift_in;
    2'b01 : shift_out = shift_in << 1;
    2'b10 : shift_out = shift_in >> 1;
    2'b11 : shift_out = $signed(shift_in) >>> 1;
    
    endcase
  end
endmodule: shifter

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
