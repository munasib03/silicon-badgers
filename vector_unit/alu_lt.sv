module alu_lt (
  input  logic [63:0] a,
  input  logic [63:0] b,
  input  logic [6:0]  sig,
  output logic        lt,
  output logic        error
);
  always_comb begin
    lt = 1'b0;
    error = 1'b0;

    case (sig)
      7'd24: begin // LT.F16
        error = (&a[14:10]) || (&b[14:10]);
        if (!error) begin
          // Both zero signs compare equal.
          if ((a[14:0] == 15'b0) && (b[14:0] == 15'b0))
            lt = 1'b0;
          else if (a[15] != b[15])
            lt = a[15];
          else if (a[15])
            lt = (a[14:0] > b[14:0]); // Both negative.
          else
            lt = (a[14:0] < b[14:0]); // Both positive.
        end
      end

      7'd25: begin // LT.F32
        error = (&a[30:23]) || (&b[30:23]);
        if (!error) begin
          if ((a[30:0] == 31'b0) && (b[30:0] == 31'b0))
            lt = 1'b0;
          else if (a[31] != b[31])
            lt = a[31];
          else if (a[31])
            lt = (a[30:0] > b[30:0]);
          else
            lt = (a[30:0] < b[30:0]);
        end
      end

      7'd26: lt = ($signed(a[31:0]) < $signed(b[31:0])); // I32
      7'd27: lt = (a[31:0] < b[31:0]);                  // U32
      7'd28: lt = ($signed(a) < $signed(b));             // I64
      7'd29: lt = (a < b);                              // U64

      default: error = 1'b1;
    endcase
  end
endmodule
