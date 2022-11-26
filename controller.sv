module controller(input clk, input rst_n, input start,
                  input [2:0] opcode, input [1:0] ALU_op, input [1:0] shift_op,
                  input Z, input N, input V,
                  output reg waiting,
                  output reg [1:0] reg_sel, output reg[1:0] wb_sel, output reg w_en, output reg load_ir, output reg load_pc, output reg clear_pc, output reg load_addr
                  output reg en_A, output reg en_B, output reg en_C, output reg en_status, output reg rm_w_en,
                  output reg sel_A, output reg sel_B);

//write to Rd
`define enable 4'd0
//Load B w/ Rm and set waiting =1
`define loadB 4'd1
//Load A with Rn
`define loadA 4'd2

//move immediate states
`define movI_one 4'd3

`define finish 4'd4


//MOV states
`define mov1 4'd5
`define mov2 4'd6


//ALU states
`define cal 4'd8


//wait
`define wait 4'd12

reg [3:0] next;
reg signal =0;
reg [3:0] state;
reg [4:0] instruction;
assign state = next;


always_ff @( posedge clk ) begin 
  if(start==1)begin
    signal<=1;
    next <= `wait;
    en_status <= 0;
    instruction <= {opcode,ALU_op};
  end 
  if(signal ==1) begin 
		if (~rst_n) begin //reset signals if reset is hit
			next <= `wait;
      waiting <= 1'b1;
      w_en <= 1'b0;
      wb_sel <= 2'b00;
      en_A <= 1'b0;
      en_B <= 1'b0;
      sel_A <= 1'b0;
      sel_B <= 1'b0;
      en_status <= 1'b0;
      en_C <= 1'b0;


		end else begin
			casex (instruction)
//move immediate
				5'b11010 : begin

          case (state)
              `wait : {next,reg_sel,w_en,wb_sel,waiting} <= {`movI_one, 2'b10,1'b1,2'b10, 1'b0};
              `movI_one : {next, waiting, signal,w_en} <= {`wait, 1'b1,1'b0,1'b0};
            
          endcase
        end

//MOV (1 reg to another)
            5'b11000 : begin
              case (state)
                `wait : {next, reg_sel, en_B,waiting} <= {`mov1,2'b00,1'b1,1'b0};
                `mov1 : {next,sel_B,sel_A,en_C} <= {`mov2,1'b0,1'b1,1'b1};
                `mov2 : {next, wb_sel,w_en,reg_sel, en_C,en_B} <= {`finish ,2'b00,1'b1,2'b01,1'b0,1'b0};
                `finish : {next, waiting, signal, en_C, w_en} <= {`wait, 1'b1,1'b0, 1'b0,1'b0};
              endcase

            end
//ADD/AND           
            5'b101x0 : begin
            case (state)
            `wait : {next,reg_sel,en_A,en_B,waiting} <= {`loadB, 2'b00,1'b0,1'b1,1'b0};
            `loadB : {next,reg_sel,en_A,en_B} <= {`loadA, 2'b10, 1'b1,1'b0};
            `loadA : {next, sel_A,sel_B,en_C} <= {`cal, 1'b0,1'b0,1'b1};
            `cal : {next, wb_sel,w_en,reg_sel,en_A,en_C,en_B} <= {`finish, 2'b00, 1'b1,2'b01,1'b0,1'b0,1'b0};
            `finish : {next, waiting, signal, w_en} <= {`wait, 1'b1,1'b0, 1'b0};
            endcase

              
            end
//CMP
            5'b10101 : begin
              case (state)
            `wait : {next,reg_sel,en_A,en_B,waiting} <= {`loadB, 2'b00,1'b0,1'b1,1'b0};
            `loadB : {next,reg_sel,en_A,en_B} <= {`loadA, 2'b10, 1'b1,1'b0};
            `loadA : {next, sel_A,sel_B, en_status} <= {`enable, 1'b0,1'b0,1'b1}; //status should output on the next rising edge, waiting goes high early
            `enable : {next, en_status, signal,en_A,en_C,en_B,waiting} <= {`wait, 1'b0, 1'b0,1'b0,1'b0,1'b0,1'b1};


              endcase

            end
//MVN           
            5'b10111 : begin
              case (state)
              `wait : {next,reg_sel,en_A,en_B,waiting} <= {`loadB, 2'b00,1'b0,1'b1,1'b0};
              `loadB : {next, sel_A,sel_B,en_C} <= {`cal, 1'b0,1'b0,1'b1};
              `cal : {next, wb_sel,w_en,reg_sel} <= {`finish, 2'b00, 1'b1,2'b01};
              `finish : {next, waiting, signal, w_en,en_A,en_C,en_B} <= {`wait, 1'b1,1'b0,1'b0,1'b0,1'b0,1'b0};
              endcase
            end

            default : next <= `wait;
            
            
			endcase
    end 
	end
	end

endmodule: controller