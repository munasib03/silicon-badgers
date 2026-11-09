// PRODUCT family: FMA (F16, F32, F16->F32), WMAC, MULSHIFT, MUL_CHECK.
// Every product is formed exactly before it is rounded, checked, or clamped.
module vec_product (
  input logic [63:0]  a,
  input logic [63:0]  b,
  input logic [63:0]  c,                // FMA/WMAC addend, MULSHIFT shift count
  input logic [6:0]   sig,
  input logic         strict,           // 1: saturation is an error, 0: write clamped value
  output logic        error,
  output logic        saturated,        // MULSHIFT clamped (product stage)
  output logic [63:0] result
);

  // PRODUCT SIG encodings (Signatures sheet)
  localparam logic [6:0] FMA_F16          = 7'b000_0000;
  localparam logic [6:0] FMA_F32          = 7'b000_0001;
  localparam logic [6:0] FMA_F16_F32      = 7'b000_0010;
  localparam logic [6:0] WMAC_I16_I32     = 7'b000_0011;
  localparam logic [6:0] WMAC_I32_I64     = 7'b000_0100;
  localparam logic [6:0] MULSHIFT_I32_I16 = 7'b000_0101;
  localparam logic [6:0] MULSHIFT_I32_U16 = 7'b000_0110;
  localparam logic [6:0] MUL_CHECK_I64    = 7'b000_0111;
  localparam logic [6:0] MUL_CHECK_U64    = 7'b000_1000;

  localparam logic signed [63:0] I32_MAX = 64'sd2147483647;
  localparam logic signed [63:0] I32_MIN = -64'sd2147483648;

  logic [15:0] fma_f16_result;
  logic [31:0] fma_f32_result;
  logic [31:0] fma_f16_f32_result;
  logic fma_f16_error, fma_f32_error, fma_f16_f32_error;

  logic signed [33:0]  wmac32;          // I16*I16 + I32, exact
  logic signed [65:0]  wmac64;          // I32*I32 + I64, exact
  logic signed [63:0]  mulshift_product;
  logic signed [63:0]  mulshift_shifted, mulshift_rounded;
  logic                mulshift_half, mulshift_sticky, mulshift_round, mulshift_overflow;
  logic signed [127:0] mul_signed;
  logic [127:0]        mul_unsigned;

  fp_fma #(
    .AB_EXP(5), .AB_FRAC(10),
    .CD_EXP(5), .CD_FRAC(10)
  ) fma_f16_inst (
    .a(a[15:0]),
    .b(b[15:0]),
    .c(c[15:0]),
    .d(fma_f16_result),
    .error(fma_f16_error)
  );

  fp_fma #(
    .AB_EXP(8), .AB_FRAC(23),
    .CD_EXP(8), .CD_FRAC(23)
  ) fma_f32_inst (
    .a(a[31:0]),
    .b(b[31:0]),
    .c(c[31:0]),
    .d(fma_f32_result),
    .error(fma_f32_error)
  );

  // F16 A/B, F32 C/D
  fp_fma #(
    .AB_EXP(5), .AB_FRAC(10),
    .CD_EXP(8), .CD_FRAC(23)
  ) fma_f16_f32_inst (
    .a(a[15:0]),
    .b(b[15:0]),
    .c(c[31:0]),
    .d(fma_f16_f32_result),
    .error(fma_f16_f32_error)
  );

  always_comb begin
    result = 64'b0;
    error = 1'b0;
    saturated = 1'b0;

    wmac32 = '0;
    wmac64 = '0;
    mulshift_product = '0;
    mulshift_shifted = '0;
    mulshift_rounded = '0;
    mulshift_half = 1'b0;
    mulshift_sticky = 1'b0;
    mulshift_round = 1'b0;
    mulshift_overflow = 1'b0;
    mul_signed = '0;
    mul_unsigned = '0;

    case (sig)

      FMA_F16: begin
        error = fma_f16_error;
        if (!error)
          result = {48'b0, fma_f16_result};
      end

      FMA_F32: begin
        error = fma_f32_error;
        if (!error)
          result = {32'b0, fma_f32_result};
      end

      FMA_F16_F32: begin
        error = fma_f16_f32_error;
        if (!error)
          result = {32'b0, fma_f16_f32_result};
      end

      // WMAC.I16_I32: D = A*B + C exactly; error if it does not fit I32.
      WMAC_I16_I32: begin
        wmac32 = $signed({{18{a[15]}}, a[15:0]}) * $signed({{18{b[15]}}, b[15:0]}) +
                 $signed({{2{c[31]}}, c[31:0]});
        error = (wmac32 > 34'sd2147483647) || (wmac32 < -34'sd2147483648);
        if (!error)
          result = {32'b0, wmac32[31:0]};
      end

      // WMAC.I32_I64: D = A*B + C exactly; error if it does not fit I64.
      WMAC_I32_I64: begin
        wmac64 = $signed({{34{a[31]}}, a[31:0]}) * $signed({{34{b[31]}}, b[31:0]}) +
                 $signed({{2{c[63]}}, c});
        error = (wmac64[65:63] != 3'b000) && (wmac64[65:63] != 3'b111);
        if (!error)
          result = wmac64[63:0];
      end

      // MULSHIFT: exact 48-bit product, divide by 2^C with ties to even, clamp to I32.
      MULSHIFT_I32_I16, MULSHIFT_I32_U16: begin
        if (sig == MULSHIFT_I32_I16)
          mulshift_product = $signed({{32{a[31]}}, a[31:0]}) * $signed({{48{b[15]}}, b[15:0]});
        else
          mulshift_product = $signed({{32{a[31]}}, a[31:0]}) * $signed({48'b0, b[15:0]});

        if (c[31:0] > 32'd63) begin
          // Shift counts are limited to 0-63.
          error = 1'b1;
        end else begin
          if (c[5:0] == 6'd0) begin
            mulshift_rounded = mulshift_product;
          end else begin
            // |product| < 2^47, so a 64-bit arithmetic shift covers every count.
            mulshift_shifted = mulshift_product >>> c[5:0];
            mulshift_half = mulshift_product[c[5:0] - 6'd1];
            mulshift_sticky = |(mulshift_product & ((64'sd1 <<< (c[5:0] - 6'd1)) - 64'sd1));
            mulshift_round = mulshift_half && (mulshift_sticky || mulshift_shifted[0]);
            mulshift_rounded = mulshift_shifted + $signed({63'b0, mulshift_round});
          end

          mulshift_overflow = (mulshift_rounded > I32_MAX) || (mulshift_rounded < I32_MIN);
          saturated = mulshift_overflow;
          error = mulshift_overflow && strict;

          if (!error) begin
            if (mulshift_rounded > I32_MAX)
              result = {32'b0, 32'h7FFF_FFFF};
            else if (mulshift_rounded < I32_MIN)
              result = {32'b0, 32'h8000_0000};
            else
              result = {32'b0, mulshift_rounded[31:0]};
          end
        end
      end

      // MUL_CHECK.I64: exact 128-bit product; error if it does not fit I64.
      MUL_CHECK_I64: begin
        mul_signed = $signed({{64{a[63]}}, a}) * $signed({{64{b[63]}}, b});
        error = (mul_signed[127:63] != {65{1'b0}}) && (mul_signed[127:63] != {65{1'b1}});
        if (!error)
          result = mul_signed[63:0];
      end

      // MUL_CHECK.U64: exact 128-bit product; error if it does not fit U64.
      MUL_CHECK_U64: begin
        mul_unsigned = {64'b0, a} * {64'b0, b};
        error = (mul_unsigned[127:64] != 64'b0);
        if (!error)
          result = mul_unsigned[63:0];
      end

      default: begin
        error = 1'b1;
        result = 64'b0;
      end

    endcase
  end

endmodule
