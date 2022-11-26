module idecoder(input [15:0] ir, input [1:0] reg_sel,
                output reg [2:0] opcode, output reg[1:0] ALU_op, output reg[1:0] shift_op,
				output reg [15:0] sximm5, output reg [15:0] sximm8,
                output reg[2:0] r_addr, output reg[2:0] w_addr);
  
   always_comb begin
        
        opcode = ir[15:13];
        ALU_op = ir[12:11];

        //sign extended values
        sximm5 = {ir[4],ir[4],ir[4],ir[4],ir[4],ir[4],ir[4],ir[4],ir[4],ir[4],ir[4],ir[4:0]};
        sximm8 = {ir[7],ir[7],ir[7],ir[7],ir[7],ir[7],ir[7],ir[7],ir[7:0]};

        shift_op = ir[4:3];

        case(reg_sel)
                2'b10: {r_addr,w_addr} = {ir[10:8],ir[10:8]};
                2'b01: {r_addr,w_addr} = {ir[7:5],ir[7:5]};
                2'b00: {r_addr,w_addr} = {ir[2:0],ir[2:0]};
                default: {r_addr,w_addr} = {6'b000000};
        endcase
            

  end

 

endmodule: idecoder