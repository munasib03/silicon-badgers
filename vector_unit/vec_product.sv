module vec_product (
  input logic [63:0]  a,
  input logic [63:0]  b,
  input logic [63:0]  c,                // addend
  input logic [6:0]   sig,
  output logic        error,
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

  logic [15:0] fma_f16_result;
  logic [31:0] fma_f32_result;
  logic [31:0] fma_f16_f32_result;
  logic fma_f16_error, fma_f32_error, fma_f16_f32_error;

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

      // WMAC, MULSHIFT, MUL_CHECK: not implemented yet.
      default: begin
        error = 1'b1;
        result = 64'b0;
      end

    endcase
  end

endmodule
