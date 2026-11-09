// Sequential unsigned divider: restoring shift-subtract, one quotient bit per cycle.
// (Binary long division, as described in https://projectf.io/posts/division-in-verilog/)
//
// Each cycle shifts the next dividend bit into the accumulator. If the accumulator is
// then at least the divisor, the divisor is subtracted and a 1 enters the quotient,
// otherwise a 0. Before the shift the accumulator is below the divisor, so one
// subtraction per step is enough.
//
// iterations: how many low dividend bits are significant (1..WIDTH). The dividend is
// pre-shifted so leading zero bits are skipped: a 32-bit divide takes 32 cycles.
// The caller handles a zero divisor (the quotient is then all ones).
//
// start (one cycle) -> busy for `iterations` cycles -> finished stays high, with
// quotient and remainder held, until the next start.
module div_iter #(
  parameter int WIDTH = 64
) (
  input  logic                       clk,
  input  logic                       rst_n,
  input  logic                       start,
  input  logic [$clog2(WIDTH+1)-1:0] iterations,
  input  logic [WIDTH-1:0]           dividend,
  input  logic [WIDTH-1:0]           divisor,
  output logic                       busy,
  output logic                       finished,
  output logic [WIDTH-1:0]           quotient,
  output logic [WIDTH-1:0]           remainder
);
  localparam int CW = $clog2(WIDTH+1);

  logic [WIDTH:0]   acc;             // partial remainder, one bit wider than the divisor
  logic [WIDTH-1:0] quo;             // remaining dividend bits shift out on the left,
                                     // quotient bits shift in on the right
  logic [WIDTH-1:0] div_reg;
  logic [CW-1:0]    count;

  logic [WIDTH:0]   acc_shifted;
  logic             fits;

  assign acc_shifted = {acc[WIDTH-1:0], quo[WIDTH-1]};
  assign fits        = (acc_shifted >= {1'b0, div_reg});

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      acc <= '0;
      quo <= '0;
      div_reg <= '0;
      count <= '0;
      busy <= 1'b0;
      finished <= 1'b0;
    end else if (start) begin
      acc <= '0;
      quo <= dividend << (WIDTH[CW-1:0] - iterations);
      div_reg <= divisor;
      count <= iterations;
      busy <= (iterations != 0);
      finished <= (iterations == 0);
    end else if (busy) begin
      if (fits) begin
        acc <= acc_shifted - {1'b0, div_reg};
        quo <= {quo[WIDTH-2:0], 1'b1};
      end else begin
        acc <= acc_shifted;
        quo <= {quo[WIDTH-2:0], 1'b0};
      end

      count <= count - 1'b1;
      if (count == 1) begin
        busy <= 1'b0;
        finished <= 1'b1;
      end
    end
  end

  assign quotient  = quo;
  assign remainder = acc[WIDTH-1:0];

endmodule
