module vec_alu (
  input logic [63:0]  a,
  input logic [63:0]  b,
  input logic [6:0]   sig,
  input logic         select_predicate,
  input logic         strict,           // 1: saturation is an error, 0: write clamped value
  output logic        error,
  output logic        predicate_result,
  output logic        saturated_add,    // ADD_SAT clamped
  output logic        saturated_sub,    // SUB_SAT clamped
  output logic [63:0] result
);

  // ALU SIG encodings (Signatures sheet, SIG = FN*8 + TYPE)
  localparam logic [6:0] ADD_F16        = 7'b000_0000;
  localparam logic [6:0] ADD_F32        = 7'b000_0001;
  localparam logic [6:0] ADD_I32        = 7'b000_0010;
  localparam logic [6:0] ADD_U32        = 7'b000_0011;
  localparam logic [6:0] ADD_I64        = 7'b000_0100;
  localparam logic [6:0] ADD_U64        = 7'b000_0101;

  localparam logic [6:0] SUB_F16        = 7'b000_1000;
  localparam logic [6:0] SUB_F32        = 7'b000_1001;
  localparam logic [6:0] SUB_I32        = 7'b000_1010;
  localparam logic [6:0] SUB_U32        = 7'b000_1011;
  localparam logic [6:0] SUB_I64        = 7'b000_1100;
  localparam logic [6:0] SUB_U64        = 7'b000_1101;

  localparam logic [6:0] EQ_F16         = 7'b001_0000;
  localparam logic [6:0] EQ_F32         = 7'b001_0001;
  localparam logic [6:0] EQ_I32         = 7'b001_0010;
  localparam logic [6:0] EQ_U32         = 7'b001_0011;
  localparam logic [6:0] EQ_I64         = 7'b001_0100;
  localparam logic [6:0] EQ_U64         = 7'b001_0101;

  localparam logic [6:0] LT_F16         = 7'b001_1000;
  localparam logic [6:0] LT_F32         = 7'b001_1001;
  localparam logic [6:0] LT_I32         = 7'b001_1010;
  localparam logic [6:0] LT_U32         = 7'b001_1011;
  localparam logic [6:0] LT_I64         = 7'b001_1100;
  localparam logic [6:0] LT_U64         = 7'b001_1101;

  localparam logic [6:0] SELECT_F16     = 7'b010_0000;
  localparam logic [6:0] SELECT_F32     = 7'b010_0001;
  localparam logic [6:0] SELECT_I32     = 7'b010_0010;
  localparam logic [6:0] SELECT_U32     = 7'b010_0011;
  localparam logic [6:0] SELECT_I64     = 7'b010_0100;
  localparam logic [6:0] SELECT_U64     = 7'b010_0101;

  localparam logic [6:0] AND_I32        = 7'b010_1010;
  localparam logic [6:0] AND_U32        = 7'b010_1011;
  localparam logic [6:0] AND_I64        = 7'b010_1100;
  localparam logic [6:0] AND_U64        = 7'b010_1101;

  localparam logic [6:0] XOR_I32        = 7'b011_0010;
  localparam logic [6:0] XOR_U32        = 7'b011_0011;
  localparam logic [6:0] XOR_I64        = 7'b011_0100;
  localparam logic [6:0] XOR_U64        = 7'b011_0101;

  localparam logic [6:0] SHL_I32        = 7'b011_1010;
  localparam logic [6:0] SHL_U32        = 7'b011_1011;
  localparam logic [6:0] SHL_I64        = 7'b011_1100;
  localparam logic [6:0] SHL_U64        = 7'b011_1101;

  localparam logic [6:0] SHR_I32        = 7'b100_0010;
  localparam logic [6:0] SHR_U32        = 7'b100_0011;
  localparam logic [6:0] SHR_I64        = 7'b100_0100;
  localparam logic [6:0] SHR_U64        = 7'b100_0101;

  localparam logic [6:0] CLZ_I32        = 7'b100_1010;
  localparam logic [6:0] CLZ_U32        = 7'b100_1011;
  localparam logic [6:0] CLZ_I64        = 7'b100_1100;
  localparam logic [6:0] CLZ_U64        = 7'b100_1101;

  localparam logic [6:0] ADD_SAT_I32    = 7'b101_0010;
  localparam logic [6:0] ADD_SAT_I64    = 7'b101_0100;

  localparam logic [6:0] SUB_SAT_I32    = 7'b101_1010;
  localparam logic [6:0] SUB_SAT_I64    = 7'b101_1100;

  localparam logic [6:0] RSHIFT_RNE_I32 = 7'b110_0010;
  localparam logic [6:0] RSHIFT_RNE_I64 = 7'b110_0100;

  logic overflow;
  logic [64:0] wide_result;
  logic [15:0] fp16_result;
  logic [31:0] fp32_result;
  
  logic sub;

  logic eq_result;
  logic lt_result;
  logic eq_error, lt_error;

  integer i;
  logic found;
  logic [6:0] clz_count;

  logic [63:0] rne_shifted;
  logic rne_half, rne_sticky, rne_round;

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
    .eq(eq_result),
    .error(eq_error)
  );

  alu_lt lt_inst (
    .a(a),
    .b(b),
    .sig(sig),
    .lt(lt_result),
    .error(lt_error)
  );

  always_comb begin
    result = 64'b0;
    error = 1'b0;
    wide_result = 65'b0;
    predicate_result = 1'b0;
    clz_count = 7'b0;
    found = 1'b0;
    overflow = 1'b0;
    saturated_add = 1'b0;
    saturated_sub = 1'b0;
    rne_shifted = 64'b0;
    rne_half = 1'b0;
    rne_sticky = 1'b0;
    rne_round = 1'b0;

    case (sig)

      ADD_F16, SUB_F16: begin
        // ADD.F16 / SUB.F16: inputs and result must be finite
        error = (&a[14:10]) || (&b[14:10]) || (&fp16_result[14:10]);
        if (!error)
          result = {48'b0, fp16_result};
      end

      ADD_F32, SUB_F32: begin
        // ADD.F32 / SUB.F32: inputs and result must be finite
        error = (&a[30:23]) || (&b[30:23]) || (&fp32_result[30:23]);
        if (!error)
          result = {32'b0, fp32_result};
      end

      ADD_I32, SUB_I32: begin
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

      ADD_U32, SUB_U32: begin
        // ADD.U32 / SUB.U32
        if (sub)
          wide_result = {33'b0, a[31:0]} - {33'b0, b[31:0]};
        else
          wide_result = {33'b0, a[31:0]} + {33'b0, b[31:0]};

        error = sub ? (a[31:0] < b[31:0]) : wide_result[32];

        if (!error)
          result = {32'b0, wide_result[31:0]};
      end

      ADD_I64, SUB_I64: begin
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

      ADD_U64, SUB_U64: begin
        // ADD.U64 / SUB.U64
        if (sub)
          wide_result = {1'b0, a} - {1'b0, b};
        else
          wide_result = {1'b0, a} + {1'b0, b};

        error = sub ? (a < b) : wide_result[64];

        if (!error)
          result = wide_result[63:0];
      end

      EQ_F16, EQ_F32, EQ_I32, EQ_U32, EQ_I64, EQ_U64: begin
        error = eq_error;
        predicate_result = !error && eq_result;
      end

      LT_F16, LT_F32, LT_I32, LT_U32, LT_I64, LT_U64: begin
        error = lt_error;
        predicate_result = !error && lt_result;
      end

      // SELECT.F16: both alternatives must be finite
      SELECT_F16: begin
        error = (&a[14:10]) || (&b[14:10]);
        if (!error)
          result = {48'b0, (select_predicate ? a[15:0] : b[15:0])};
      end

      // SELECT.F32: both alternatives must be finite
      SELECT_F32: begin
        error = (&a[30:23]) || (&b[30:23]);
        if (!error)
          result = {32'b0, (select_predicate ? a[31:0] : b[31:0])};
      end

      // SELECT.I32 / SELECT.U32
      SELECT_I32, SELECT_U32: begin
        result = {32'b0, (select_predicate ? a[31:0] : b[31:0])};
      end

      // SELECT.I64 / SELECT.U64
      SELECT_I64, SELECT_U64: begin
        result = select_predicate ? a : b;
      end

      // AND.I32 / AND.U32
      AND_I32, AND_U32: begin
        result = {32'b0, (a[31:0] & b[31:0])};
      end

      // AND.I64 / AND.U64
      AND_I64, AND_U64: begin
        result = a & b;
      end

      // XOR.I32 / XOR.U32
      XOR_I32, XOR_U32: begin
        result = {32'b0, (a[31:0] ^ b[31:0])};
      end

      // XOR.I64 / XOR.U64
      XOR_I64, XOR_U64: begin
        result = a ^ b;
      end

      // SHL.I32 / SHL.U32
      SHL_I32, SHL_U32: begin
        if (b[31:0] >= 32)
          result = 64'b0;
        else
          result = {32'b0, (a[31:0] << b[31:0])};
      end

      // SHL.I64 / SHL.U64
      SHL_I64, SHL_U64: begin
        if (b[31:0] >= 64)
          result = 64'b0;
        else
          result = a << b[31:0];
      end

      // SHR.I32 (arithmetic right shift)
      SHR_I32: begin
        if (b[31:0] >= 32)
          result = a[31] ? {32'b0, 32'hFFFFFFFF} : 64'b0;
        else
          result = {32'b0, ($signed(a[31:0]) >>> b[31:0])};
      end

      // SHR.U32 (logical right shift)
      SHR_U32: begin
        if (b[31:0] >= 32)
          result = 64'b0;
        else
          result = {32'b0, (a[31:0] >> b[31:0])};
      end

      // SHR.I64 (arithmetic right shift)
      SHR_I64: begin
        if (b[31:0] >= 64)
          result = a[63] ? 64'hFFFFFFFFFFFFFFFF : 64'b0;
        else
          result = $signed(a) >>> b[31:0];
      end

      // SHR.U64 (logical right shift)
      SHR_U64: begin
        if (b[31:0] >= 64)
          result = 64'b0;
        else
          result = a >> b[31:0];
      end

      CLZ_I32, CLZ_U32: begin
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

      CLZ_I64, CLZ_U64: begin
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

      ADD_SAT_I32, SUB_SAT_I32: begin
        // ADD_SAT.I32 / SUB_SAT.I32
        if (sub)
          wide_result = {33'b0, a[31:0]} - {33'b0, b[31:0]};
        else
          wide_result = {33'b0, a[31:0]} + {33'b0, b[31:0]};

        overflow = sub ? ((a[31] != b[31]) && (a[31] != wide_result[31])) :
                         ((a[31] == b[31]) && (a[31] != wide_result[31]));
        saturated_add = overflow && !sub;
        saturated_sub = overflow && sub;
        error = overflow && strict;

        // Overflow direction follows the sign of A.
        if (!error)
          result = overflow ? {32'b0, (a[31] ? 32'h8000_0000 : 32'h7FFF_FFFF)} :
                              {32'b0, wide_result[31:0]};
      end

      ADD_SAT_I64, SUB_SAT_I64: begin
        // ADD_SAT.I64 / SUB_SAT.I64
        if (sub)
          wide_result = {1'b0, a} - {1'b0, b};
        else
          wide_result = {1'b0, a} + {1'b0, b};

        overflow = sub ? ((a[63] != b[63]) && (a[63] != wide_result[63])) :
                         ((a[63] == b[63]) && (a[63] != wide_result[63]));
        saturated_add = overflow && !sub;
        saturated_sub = overflow && sub;
        error = overflow && strict;

        if (!error)
          result = overflow ? (a[63] ? 64'h8000_0000_0000_0000 : 64'h7FFF_FFFF_FFFF_FFFF) :
                              wide_result[63:0];
      end

      // RSHIFT_RNE.I32: A / 2^B rounded to nearest, ties to even
      RSHIFT_RNE_I32: begin
        if (b[31:0] >= 32)
          // |A| / 2^32 is at most 0.5, which ties to 0.
          result = 64'b0;
        else if (b[31:0] == 0)
          result = {32'b0, a[31:0]};
        else begin
          rne_shifted = {32'b0, ($signed(a[31:0]) >>> b[4:0])};
          rne_half = a[b[4:0] - 5'd1];
          rne_sticky = |(a[31:0] & ((32'd1 << (b[4:0] - 5'd1)) - 32'd1));
          rne_round = rne_half && (rne_sticky || rne_shifted[0]);
          result = {32'b0, (rne_shifted[31:0] + {31'b0, rne_round})};
        end
      end

      // RSHIFT_RNE.I64: A / 2^B rounded to nearest, ties to even
      RSHIFT_RNE_I64: begin
        if (b[31:0] >= 64)
          // |A| / 2^64 is at most 0.5, which ties to 0.
          result = 64'b0;
        else if (b[31:0] == 0)
          result = a;
        else begin
          rne_shifted = $signed(a) >>> b[5:0];
          rne_half = a[b[5:0] - 6'd1];
          rne_sticky = |(a & ((64'd1 << (b[5:0] - 6'd1)) - 64'd1));
          rne_round = rne_half && (rne_sticky || rne_shifted[0]);
          result = rne_shifted + {63'b0, rne_round};
        end
      end

      default: begin
        error = 1'b1;
        predicate_result = 1'b0;
        result = 64'b0;
      end

    endcase
end

endmodule
