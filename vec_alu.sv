module vec_alu (
    input logic [63:0] a,
    input logic [63:0] b,
    input logic [6:0] sig,
    output logic error,
    output logic predicate_result,
    output logic [63:0] result
);

  logic overflow;
  logic [64:0] wide_result;
  logic [15:0] fp16_result;
  logic [31:0] fp32_result;
  logic sub;

  assign sub = sig[3];

  fp16_addsub fp16_addsub_inst (
    .a(a[15:0]),
    .b(b[15:0]),
    .sub(sub),
    .d(fp16_result)
  );

  fp32_addsub fp32_addsub_inst (
    .a(a[31:0]),
    .b(b[31:0]),
    .sub(sub),
    .d(fp32_result)
  );

  always_comb begin
    result = 64'b0;
    error = 1'b0;
    wide_result = 65'b0;
    predicate_result = 1'b0;

    case (sig)

      7'b000_0000, 7'b000_1000: begin
        result = {48'b0, fp16_result};
      end

      7'b000_0001, 7'b000_1001: begin
        result = {32'b0, fp32_result};
      end

      7'b000_0010, 7'b000_1010: begin
        // ADD.I32 / SUB.I32
        if (sub)
          wide_result = {33'b0, a[31:0]} - {33'b0, b[31:0]};
        else
          wide_result = {33'b0, a[31:0]} + {33'b0, b[31:0]};
          error = sub ? ((a[31] != b[31]) && (a[31] != wide_result[31])) : 
                        ((a[31] == b[31]) && (a[31] != wide_result[31]));
          if (!error)
            result = {32'b0, wide_result[31:0]};
      end

      7'b000_0011, 7'b000_1011: begin
        // ADD.U32 / SUB.U32
        if (sub)
          wide_result = {33'b0, a[31:0]} - {33'b0, b[31:0]};
        else
          wide_result = {33'b0, a[31:0]} + {33'b0, b[31:0]};
          error = sub ? (a[31:0] < b[31:0]) : wide_result[32];
          
          if (!error)
            result = {32'b0, wide_result[31:0]};
      end

      7'b000_0100, 7'b000_1100: begin
        // ADD.I64 / SUB.I64
        if (sub)
          wide_result = {1'b0, a} - {1'b0, b};
        else
          wide_result = {1'b0, a} + {1'b0, b};

          error = sub ? ((a[63] != b[63]) && (a[63] != wide_result[63])) : 
                        ((a[63] == b[63]) && (a[63] != wide_result[63]));

          if (!error)
            result = wide_result[63:0];
      end

      7'b000_0101, 7'b000_1101: begin
          // ADD.U64 / SUB.U64
          if (sub)
            wide_result = {1'b0, a} - {1'b0, b};
          else
            wide_result = {1'b0, a} + {1'b0, b};

            error = sub ? (a < b) : wide_result[64];

            if (!error)
              result = wide_result[63:0];
      end

      default: begin
        error = 1'b1;
      end

    endcase
end

endmodule