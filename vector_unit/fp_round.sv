// Round an exact value to an IEEE 754 format, round-to-nearest ties-to-even.
// Value = (-1)^sign * mag * 2^exponent, plus a positive amount smaller than one
// unit of mag's last bit when sticky_in is set (used for inexact quotients and roots).
// When sticky_in can be set, mag must carry at least FRAC+3 significant bits so the
// sticky amount always lies below the guard bit.
// Subnormals are kept. A value that rounds to zero keeps its sign.
// overflow: the result rounds to infinity (d is then not meaningful).
module fp_round #(
  parameter int EXP  = 5,
  parameter int FRAC = 10
) (
  input  logic            sign,
  input  logic [63:0]     mag,
  input  integer          exponent,
  input  logic            sticky_in,
  output logic [EXP+FRAC:0] d,
  output logic            overflow
);
  localparam int BIAS    = (1 << (EXP - 1)) - 1;
  localparam int UNIT    = 1 - BIAS - FRAC;      // smallest subnormal step: F16 -24, F32 -149
  localparam int EXP_MAX = (1 << EXP) - 1;

  logic [127:0] extended;
  logic [FRAC+1:0] kept, rounded;
  logic guard_bit, sticky_bit, round_up;
  integer msb, lsb_exponent, right_shift, exponent_field;

  always_comb begin
    extended = {64'b0, mag};
    kept = '0;
    rounded = '0;
    guard_bit = 1'b0;
    sticky_bit = sticky_in;
    round_up = 1'b0;
    msb = -1;
    lsb_exponent = UNIT;
    right_shift = 0;
    exponent_field = 0;
    overflow = 1'b0;
    d = {sign, {(EXP+FRAC){1'b0}}};

    for (int i = 0; i < 64; i++) begin
      if (mag[i])
        msb = i;
    end

    if (msb >= 0) begin
      // Exponent of the result's last bit: FRAC+1 significant bits, but never
      // below the smallest subnormal step.
      lsb_exponent = (msb + exponent - FRAC > UNIT) ? msb + exponent - FRAC : UNIT;
      right_shift = lsb_exponent - exponent;

      if (right_shift <= 0) begin
        // Exact: mag already fits in FRAC+1 bits at this exponent.
        kept = (FRAC+2)'(mag << (-right_shift));
      end else if (right_shift > 65) begin
        // mag < 2^64, so the value is below a quarter of the smallest step.
        sticky_bit = 1'b1;
      end else begin
        kept = (FRAC+2)'(extended >> right_shift);
        guard_bit = extended[right_shift-1];
        for (int i = 0; i < 64; i++) begin
          if (i < right_shift-1)
            sticky_bit = sticky_bit | mag[i];
        end
      end

      round_up = guard_bit && (sticky_bit || kept[0]);
      rounded = kept + (FRAC+2)'(round_up);

      // Rounding can carry into a new bit: renormalize.
      if (rounded[FRAC+1]) begin
        rounded = rounded >> 1;
        lsb_exponent = lsb_exponent + 1;
      end

      // Normal results have the implicit bit set; otherwise subnormal or zero.
      exponent_field = rounded[FRAC] ? lsb_exponent - UNIT + 1 : 0;

      if (exponent_field >= EXP_MAX)
        overflow = 1'b1;
      else
        d = {sign, exponent_field[EXP-1:0], rounded[FRAC-1:0]};
    end
  end

endmodule
