// Sequential reciprocal square root D = 1/sqrt(A) for an F32 input, rounded once to
// F16 and to F32 (nearest, ties to even). F16 inputs are widened exactly by the caller.
//
// With A = m * 2^(2k) (m an integer significand, exponent made even):
//   1/sqrt(A) = (2^N / sqrt(m)) * 2^(-N-k)
// r = floor(2^N / sqrt(m)) is the largest integer with r*r*m <= 2^(2N). It is found
// one bit per cycle, high bit first: keep a trial bit if trial*trial*m <= 2^(2N).
// No division is needed. The result is exact only when r*r*m == 2^(2N); otherwise the
// sticky bit is set, so the single rounding is correct.
//
// m is in [2^23, 2^25), so r is in (2^26.5, 2^27.5]: 28 root bits, 28 cycles.
// a must stay stable from start until finished.
// error: A is NaN/Inf, zero (either sign), or negative.
module fp_rsqrt (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        start,
  input  logic [31:0] a,
  output logic        busy,
  output logic        finished,
  output logic [15:0] d16,
  output logic        error16,
  output logic [31:0] d32,
  output logic        error32
);
  localparam int N         = 39;           // r has at least 26 = FRAC+3 bits for F32
  localparam int ROOT_BITS = 28;

  localparam logic [80:0] TARGET = 81'd1 << (2 * N);

  logic [7:0]  exp_a;
  logic [23:0] sig_a;
  logic [24:0] m;
  integer      msb, e_a;
  logic        input_error;

  logic [ROOT_BITS-1:0] root, trial;
  logic [80:0]          product;
  logic                 exact;
  logic [4:0]           bit_index;

  logic overflow16, overflow32;

  // Normalize A and make its exponent even (combinational from the held input).
  always_comb begin
    exp_a = a[30:23];
    sig_a = {(exp_a != 0), a[22:0]};
    e_a = ((exp_a == 0) ? 1 : int'(exp_a)) - 150;              // A = sig_a * 2^e_a

    msb = 0;
    for (int i = 0; i < 24; i++) begin
      if (sig_a[i]) msb = i;
    end
    m = {1'b0, sig_a} << (23 - msb);
    e_a = e_a - (23 - msb);

    if (e_a[0]) begin
      m = m << 1;
      e_a = e_a - 1;
    end

    input_error = (&exp_a) || (sig_a == 0) || a[31];
  end

  // Trial: the current root with the next bit set.
  assign trial   = root | ({{(ROOT_BITS-1){1'b0}}, 1'b1} << bit_index);
  assign product = 81'(trial) * 81'(trial) * 81'(m);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      root <= '0;
      exact <= 1'b0;
      bit_index <= '0;
      busy <= 1'b0;
      finished <= 1'b0;
    end else if (start) begin
      root <= '0;
      exact <= 1'b0;
      bit_index <= 5'(ROOT_BITS - 1);
      busy <= 1'b1;
      finished <= 1'b0;
    end else if (busy) begin
      if (product <= TARGET) begin
        root <= trial;
        exact <= (product == TARGET);
      end

      if (bit_index == 0) begin
        busy <= 1'b0;
        finished <= 1'b1;
      end else begin
        bit_index <= bit_index - 1'b1;
      end
    end
  end

  fp_round #(.EXP(5), .FRAC(10)) round_f16_inst (
    .sign(1'b0),
    .mag(64'(root)),
    .exponent(-N - (e_a >>> 1)),
    .sticky_in(!exact),
    .d(d16),
    .overflow(overflow16)
  );

  fp_round #(.EXP(8), .FRAC(23)) round_f32_inst (
    .sign(1'b0),
    .mag(64'(root)),
    .exponent(-N - (e_a >>> 1)),
    .sticky_in(!exact),
    .d(d32),
    .overflow(overflow32)
  );

  assign error16 = input_error || overflow16;
  assign error32 = input_error || overflow32;

endmodule
