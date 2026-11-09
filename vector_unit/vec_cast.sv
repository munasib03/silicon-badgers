// CAST family: convert the exact value A * 2^e and round once to the destination.
// SIG = FROM*8 + TO with type numbers F16=0, F32=1, I8=2, U8=3, I16=4, U16=5, I32=6, I64=7.
// B is the signed I32 exponent e in -255..255; float-to-float requires e = 0.
// Integer results clamp and record a conversion saturation (an error in strict mode).
// Float results that round to infinity are an error.
module vec_cast (
  input  logic [63:0] a,
  input  logic [63:0] b,                 // I32 exponent e
  input  logic [6:0]  sig,
  input  logic        strict,            // 1: saturation is an error, 0: write clamped value
  output logic        error,
  output logic        saturated,         // conversion stage clamped
  output logic [63:0] result
);

  // CAST type numbers
  localparam logic [2:0] C_F16 = 3'd0;
  localparam logic [2:0] C_F32 = 3'd1;
  localparam logic [2:0] C_I8  = 3'd2;
  localparam logic [2:0] C_U8  = 3'd3;
  localparam logic [2:0] C_I16 = 3'd4;
  localparam logic [2:0] C_U16 = 3'd5;
  localparam logic [2:0] C_I32 = 3'd6;
  localparam logic [2:0] C_I64 = 3'd7;

  // CAST SIG encodings (Signatures sheet, SIG = FROM*8 + TO)
  localparam logic [6:0] CAST_F16_F16 = 7'd0;
  localparam logic [6:0] CAST_F16_F32 = 7'd1;
  localparam logic [6:0] CAST_F16_I32 = 7'd6;
  localparam logic [6:0] CAST_F16_I64 = 7'd7;
  localparam logic [6:0] CAST_F32_F16 = 7'd8;
  localparam logic [6:0] CAST_F32_F32 = 7'd9;
  localparam logic [6:0] CAST_F32_I32 = 7'd14;
  localparam logic [6:0] CAST_F32_I64 = 7'd15;
  localparam logic [6:0] CAST_I8_I32  = 7'd22;
  localparam logic [6:0] CAST_U8_I32  = 7'd30;
  localparam logic [6:0] CAST_I16_I32 = 7'd38;
  localparam logic [6:0] CAST_U16_I32 = 7'd46;
  localparam logic [6:0] CAST_I32_F16 = 7'd48;
  localparam logic [6:0] CAST_I32_F32 = 7'd49;
  localparam logic [6:0] CAST_I32_I8  = 7'd50;
  localparam logic [6:0] CAST_I32_U8  = 7'd51;
  localparam logic [6:0] CAST_I32_I16 = 7'd52;
  localparam logic [6:0] CAST_I32_U16 = 7'd53;
  localparam logic [6:0] CAST_I32_I32 = 7'd54;
  localparam logic [6:0] CAST_I32_I64 = 7'd55;
  localparam logic [6:0] CAST_I64_F16 = 7'd56;
  localparam logic [6:0] CAST_I64_F32 = 7'd57;
  localparam logic [6:0] CAST_I64_I32 = 7'd62;
  localparam logic [6:0] CAST_I64_I64 = 7'd63;

  logic [2:0] from_type, to_type;
  logic legal_sig, from_float, to_float;

  // Exact source value: (-1)^sign * mag * 2^source_exponent
  logic        sign;
  logic [63:0] mag;
  integer      source_exponent, scale, total_exponent;
  logic        source_error, scale_error;

  // Integer destination
  logic [127:0] extended;
  logic [128:0] int_mag;                 // rounded magnitude
  logic         int_huge;                // magnitude is at least 2^64
  logic         guard_bit, sticky_bit;
  logic [128:0] limit_positive, limit_negative;
  logic         clamp;
  logic [63:0]  int_value;
  integer       right_shift;

  // Float destination
  logic [15:0] f16_value;
  logic [31:0] f32_value;
  logic        f16_overflow, f32_overflow;

  assign from_type = sig[5:3];
  assign to_type   = sig[2:0];
  assign from_float = (from_type == C_F16) || (from_type == C_F32);
  assign to_float   = (to_type == C_F16) || (to_type == C_F32);

  always_comb begin
    case (sig)
      CAST_F16_F16, CAST_F16_F32, CAST_F16_I32, CAST_F16_I64,
      CAST_F32_F16, CAST_F32_F32, CAST_F32_I32, CAST_F32_I64,
      CAST_I8_I32, CAST_U8_I32, CAST_I16_I32, CAST_U16_I32,
      CAST_I32_F16, CAST_I32_F32, CAST_I32_I8, CAST_I32_U8,
      CAST_I32_I16, CAST_I32_U16, CAST_I32_I32, CAST_I32_I64,
      CAST_I64_F16, CAST_I64_F32, CAST_I64_I32, CAST_I64_I64: legal_sig = 1'b1;
      default:                                                legal_sig = 1'b0;
    endcase
  end

  // Source decode.
  always_comb begin
    sign = 1'b0;
    mag = 64'b0;
    source_exponent = 0;
    source_error = 1'b0;

    case (from_type)
      C_F16: begin
        sign = a[15];
        mag = {53'b0, (a[14:10] != 0), a[9:0]};
        source_exponent = ((a[14:10] == 0) ? 1 : int'(a[14:10])) - 25;
        source_error = &a[14:10];        // NaN or infinity
      end

      C_F32: begin
        sign = a[31];
        mag = {40'b0, (a[30:23] != 0), a[22:0]};
        source_exponent = ((a[30:23] == 0) ? 1 : int'(a[30:23])) - 150;
        source_error = &a[30:23];
      end

      C_I8: begin
        sign = a[7];
        mag = {56'b0, (a[7] ? -a[7:0] : a[7:0])};
      end

      C_U8:  mag = {56'b0, a[7:0]};

      C_I16: begin
        sign = a[15];
        mag = {48'b0, (a[15] ? -a[15:0] : a[15:0])};
      end

      C_U16: mag = {48'b0, a[15:0]};

      C_I32: begin
        sign = a[31];
        mag = {32'b0, (a[31] ? -a[31:0] : a[31:0])};
      end

      default: begin // C_I64; the minimum's magnitude 2^63 still fits 64 bits
        sign = a[63];
        mag = a[63] ? -a : a;
      end
    endcase
  end

  // B is a signed I32 exponent in -255..255; float-to-float requires zero.
  always_comb begin
    scale = int'($signed(b[31:0]));
    scale_error = (scale < -255) || (scale > 255) || (from_float && to_float && (scale != 0));
    total_exponent = source_exponent + scale;
  end

  // Integer destination: round the magnitude to an integer (ties to even), then clamp.
  always_comb begin
    extended = {64'b0, mag};
    int_mag = '0;
    int_huge = 1'b0;
    guard_bit = 1'b0;
    sticky_bit = 1'b0;
    right_shift = 0;

    if (mag != 0) begin
      if (total_exponent >= 64) begin
        int_huge = 1'b1;
      end else if (total_exponent >= 0) begin
        int_mag = {1'b0, extended << total_exponent};
      end else if (total_exponent >= -65) begin
        right_shift = -total_exponent;
        int_mag = {1'b0, extended >> right_shift};
        guard_bit = extended[right_shift-1];
        for (int i = 0; i < 64; i++) begin
          if (i < right_shift-1)
            sticky_bit = sticky_bit | mag[i];
        end
        int_mag = int_mag + 129'(guard_bit && (sticky_bit || int_mag[0]));
      end
      // Below 2^-65 the magnitude is under a quarter and rounds to zero.
    end
  end

  // Destination range, as magnitudes on each side of zero.
  always_comb begin
    case (to_type)
      C_I8:    begin limit_positive = 129'd127;        limit_negative = 129'd128;        end
      C_U8:    begin limit_positive = 129'd255;        limit_negative = 129'd0;          end
      C_I16:   begin limit_positive = 129'd32767;      limit_negative = 129'd32768;      end
      C_U16:   begin limit_positive = 129'd65535;      limit_negative = 129'd0;          end
      C_I32:   begin limit_positive = 129'd2147483647; limit_negative = 129'd2147483648; end
      default: begin
        limit_positive = {66'b0, 63'h7FFF_FFFF_FFFF_FFFF};
        limit_negative = {65'b0, 64'h8000_0000_0000_0000};
      end
    endcase

    clamp = int_huge || (sign ? (int_mag > limit_negative) : (int_mag > limit_positive));

    if (clamp)
      int_value = sign ? -limit_negative[63:0] : limit_positive[63:0];
    else
      int_value = sign ? -int_mag[63:0] : int_mag[63:0];
  end

  fp_round #(.EXP(5), .FRAC(10)) round_f16_inst (
    .sign(sign),
    .mag(mag),
    .exponent(total_exponent),
    .sticky_in(1'b0),
    .d(f16_value),
    .overflow(f16_overflow)
  );

  fp_round #(.EXP(8), .FRAC(23)) round_f32_inst (
    .sign(sign),
    .mag(mag),
    .exponent(total_exponent),
    .sticky_in(1'b0),
    .d(f32_value),
    .overflow(f32_overflow)
  );

  always_comb begin
    result = 64'b0;
    saturated = 1'b0;
    error = !legal_sig || source_error || scale_error;

    if (!error) begin
      if (to_float) begin
        // Float results must be finite.
        if (to_type == C_F16) begin
          error = f16_overflow;
          result = {48'b0, f16_value};
        end else begin
          error = f32_overflow;
          result = {32'b0, f32_value};
        end
      end else begin
        saturated = clamp;
        error = clamp && strict;
        // Fill the low bits of the destination; zero the rest of the cell.
        case (to_type)
          C_I8, C_U8:   result = {56'b0, int_value[7:0]};
          C_I16, C_U16: result = {48'b0, int_value[15:0]};
          C_I32:        result = {32'b0, int_value[31:0]};
          default:      result = int_value;
        endcase
      end
    end

    if (error)
      result = 64'b0;
  end

endmodule
