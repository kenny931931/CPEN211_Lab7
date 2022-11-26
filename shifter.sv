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