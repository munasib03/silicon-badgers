module alu_eq (
  input  logic [63:0] a,
  input  logic [63:0] b,
  input  logic [6:0]  sig,
  output logic        eq,
  output logic        error
);
  always_comb begin
    eq = 1'b0;
    error = 1'b0;

    case (sig)
      7'd16: begin // EQ.F16
        error = (&a[14:10]) || (&b[14:10]);
        if (!error)
          eq = (a[15:0] == b[15:0]) ||
               ((a[14:0] == 15'b0) && (b[14:0] == 15'b0));
      end

      7'd17: begin // EQ.F32
        error = (&a[30:23]) || (&b[30:23]);
        if (!error)
          eq = (a[31:0] == b[31:0]) ||
               ((a[30:0] == 31'b0) && (b[30:0] == 31'b0));
      end

      7'd18, 7'd19: eq = (a[31:0] == b[31:0]); // I32/U32
      7'd20, 7'd21: eq = (a == b);             // I64/U64

      default: error = 1'b1;
    endcase
  end
endmodule
