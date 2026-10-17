module vec_alu (
  input logic [63:0]  a,
  input logic [63:0]  b,
  input logic [6:0]   sig,
  output logic        error,
  output logic        predicate_result,
  output logic [63:0] result
);

  logic overflow;
  logic [64:0] wide_result;
  logic [15:0] fp16_result;
  logic [31:0] fp32_result;
  
  logic sub;

  logic eq_result;
  logic lt_result;

  integer i;
  logic found;
  logic [6:0] clz_count;

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

  alu_eq alu_eq_inst (
    .a(a),
    .b(b),
    .sig(sig),
    .eq(eq_result)
  );

  alu_lt lt_inst (
    .a(a),
    .b(b),
    .sig(sig),
    .lt(lt_result)
  );

  always_comb begin
    result = 64'b0;
    error = 1'b0;
    wide_result = 65'b0;
    predicate_result = 1'b0;
    clz_count = 7'b0;
    found = 1'b0;

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

      7'b001_0000, 7'b001_0001, 7'b001_0010, 7'b001_0011, 7'b001_0100, 7'b001_0101: begin
        predicate_result = eq_result;
      end

      7'b001_1000, 7'b001_1001, 7'b001_1010, 7'b001_1011, 7'b001_1100, 7'b001_1101: begin
        predicate_result = lt_result;
      end

      // AND.I32 / AND.U32
      7'b010_1010, 7'b010_1011: begin
        result = {32'b0, (a[31:0] & b[31:0])};
      end

      // AND.I64 / AND.U64
      7'b010_1100, 7'b010_1101: begin
        result = a & b;
      end

      // XOR.I32 / XOR.U32
      7'b011_0010, 7'b011_0011: begin
        result = {32'b0, (a[31:0] ^ b[31:0])};
      end

      // XOR.I64 / XOR.U64
      7'b011_0100, 7'b011_0101: begin
        result = a ^ b;
      end

      // SHL.I32 / SHL.U32
      7'b011_1010,
      7'b011_1011: begin
        if (b[31:0] >= 32)
          result = 64'b0;
        else
          result = {32'b0, (a[31:0] << b[31:0])};
      end

      // SHL.I64 / SHL.U64
      7'b011_1100,
      7'b011_1101: begin
        if (b[31:0] >= 64)
          result = 64'b0;
        else
          result = a << b[31:0];
      end

      // SHR.I32 (arithmetic right shift)
      7'b100_0010: begin
        if (b[31:0] >= 32)
          result = a[31] ? {32'b0, 32'hFFFFFFFF} : 64'b0;
        else
          result = {32'b0, ($signed(a[31:0]) >>> b[31:0])};
      end

      // SHR.U32 (logical right shift)
      7'b100_0011: begin
        if (b[31:0] >= 32)
          result = 64'b0;
        else
          result = {32'b0, (a[31:0] >> b[31:0])};
      end

      // SHR.I64 (arithmetic right shift)
      7'b100_0100: begin
        if (b[31:0] >= 64)
          result = a[63] ? 64'hFFFFFFFFFFFFFFFF : 64'b0;
        else
          result = $signed(a) >>> b[31:0];
      end

      // SHR.U64 (logical right shift)
      7'b100_0101: begin
        if (b[31:0] >= 64)
          result = 64'b0;
        else
          result = a >> b[31:0];
      end

      7'b100_1010, 7'b100_1011: begin
        // CLZ I32 / CLZ U32
        for (i = 31; i >= 0; i = i - 1) begin
          if (!found) begin
            if (a[i] == 1'b0)
              clz_count = clz_count + 1;
            else
              found = 1;
          end
        end
        result = {57'b0, clz_count};
      end

      7'b100_1100, 7'b100_1101: begin
        // CLZ I64 / CLZ U64
        for (i = 63; i >= 0; i = i - 1) begin
          if (!found) begin
            if (a[i] == 1'b0)
              clz_count = clz_count + 1;
            else
              found = 1;
          end
        end
        result = {57'b0, clz_count};
      end

      default: begin
        error = 1'b1;
        predicate_result = 1'b0;
        result = 64'b0;
      end

    endcase
end

endmodule
