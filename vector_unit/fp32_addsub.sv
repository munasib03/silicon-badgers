// Combinational IEEE 754 binary32 add/subtract, round-to-nearest ties-to-even.
// sub=0: D=A+B; sub=1: D=A-B.
// Inputs must be finite. Overflow produces infinity (outside finite-result contract).
module fp32_addsub (
  input  logic [31:0] a,
  input  logic [31:0] b,
  input  logic        sub,
  output logic [31:0] d
);
  logic sign_a, sign_b, sign_result;
  logic [7:0] exp_a, exp_b;
  logic [22:0] frac_a, frac_b;
  logic [23:0] sig_a, sig_b;
  logic [7:0] shift_a, shift_b;
  logic [279:0] mag_a, mag_b, magnitude;
  logic [23:0] kept;
  logic [24:0] rounded;
  logic guard_bit, sticky_bit, round_up;
  integer msb, right_shift, exponent_result;

  always_comb begin
    sign_a = a[31];
    sign_b = b[31] ^ sub;
    exp_a = a[30:23];
    exp_b = b[30:23];
    frac_a = a[22:0];
    frac_b = b[22:0];

    // The implicit leading bit is zero for subnormals and zero.
    sig_a = {(exp_a != 0), frac_a};
    sig_b = {(exp_b != 0), frac_b};

    // Exact magnitudes in units of 2^-149.
    // Normal: (2^23 + fraction) * 2^(exp - 1).
    // Subnormal: fraction.
    shift_a = (exp_a == 0) ? 8'd0 : exp_a - 8'd1;
    shift_b = (exp_b == 0) ? 8'd0 : exp_b - 8'd1;
    mag_a = {256'b0, sig_a} << shift_a;
    mag_b = {256'b0, sig_b} << shift_b;

    sign_result = 1'b0;
    magnitude = 280'd0;
    kept = 24'd0;
    rounded = 25'd0;
    guard_bit = 1'b0;
    sticky_bit = 1'b0;
    round_up = 1'b0;
    msb = -1;
    right_shift = 0;
    exponent_result = 0;
    d = 32'h00000000;

    if (sign_a == sign_b) begin
      magnitude = mag_a + mag_b;
      sign_result = sign_a;
    end else if (mag_a >= mag_b) begin
      magnitude = mag_a - mag_b;
      sign_result = sign_a;
    end else begin
      magnitude = mag_b - mag_a;
      sign_result = sign_b;
    end

    if (magnitude == 0) begin
      // Exact cancellation gives +0 under RNE, except -0 + -0.
      d = {((mag_a == 0) && (mag_b == 0) && sign_a && sign_b), 31'b0};
    end else begin
      // Find highest set bit of the exact magnitude.
      for (int i = 0; i < 280; i++) begin
        if (magnitude[i])
          msb = i;
      end

      if (msb < 23) begin
        // Exact subnormal (all input values are multiples of 2^-149).
        d = {sign_result, 8'b00000000, magnitude[22:0]};
      end else begin
        // Retain 24 significant bits including the implicit one.
      right_shift = msb - 23;
      kept = 24'(magnitude >> right_shift);

      // Guard and sticky encode all discarded bits.
      if (right_shift > 0) begin
          guard_bit = magnitude[right_shift-1];
          for (int i = 0; i < 280; i++) begin
              if (i < right_shift-1)
                  sticky_bit = sticky_bit | magnitude[i];
          end
      end
        round_up = guard_bit && (sticky_bit || kept[0]);
        rounded = {1'b0, kept} + {24'b0, round_up};
        exponent_result = msb - 22;

        // Rounding can overflow the significand into the next exponent.
        if (rounded[24]) begin
            rounded = rounded >> 1;
            exponent_result = exponent_result + 1;
        end

        if (exponent_result >= 255) begin
            // Overflow is outside the finite-result contract.
            d = {sign_result, 8'hFF, 23'b0};
        end else begin
            d = {sign_result, 8'(exponent_result), rounded[22:0]};
        end
      end
    end
  end
endmodule
