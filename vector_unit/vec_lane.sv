// One SIMD lane: operand decode, lane enable, ALU, and this lane's write request.
// Instruction-wide decisions (legality, commit on any-lane error, scalar-once,
// SELECT's lowest-enabled-lane rule for scalar D) belong to the lane array.
module vec_lane #(
  parameter int LANE_INDEX = 0
) (
  input  logic [6:0]  sig,
  input  logic [4:0]  a_code,            // A field: 0-7 V, 8-15 S, 16-19 constants
  input  logic [4:0]  b_code,            // B field: same encoding as A
  input  logic [4:0]  c_code,            // C field: SELECT choice predicate P0..P7
  input  logic [2:0]  pm,                // execution mask predicate P0..P7
  input  logic [31:0] vl,                // active vector length
  input  logic        strict,            // 1: saturation is an error

  input  logic [63:0] v_reg [8],         // this lane's cell of V0..V7
  input  logic [63:0] s_reg [8],         // S0..S7, shared by all lanes
  input  logic [7:0]  p_bits,            // this lane's bit of P0..P7

  output logic        enable,            // lane is below VL and its PM bit is true
  output logic        error,             // only raised by enabled lanes
  output logic        saturated_add,
  output logic        saturated_sub,
  output logic        write_predicate,   // 1: write predicate_result to P[D], 0: write result to D
  output logic        predicate_result,
  output logic [63:0] result
);

  localparam logic [4:0] K_ZERO     = 5'd16;
  localparam logic [4:0] K_NEG_ZERO = 5'd17;
  localparam logic [4:0] K_ONE      = 5'd18;
  localparam logic [4:0] K_ONES     = 5'd19;

  logic [7:0]  p_live;
  logic [63:0] a, b;
  logic [63:0] k_neg_zero, k_one, k_ones;

  logic alu_error;
  logic alu_predicate_result;
  logic alu_saturated_add, alu_saturated_sub;

  // P0 is always true.
  assign p_live = {p_bits[7:1], 1'b1};

  assign enable = (LANE_INDEX[31:0] < vl) && p_live[pm];

  // EQ and LT (FN 2, 3) are the only ALU operations that write a predicate.
  assign write_predicate = (sig[6:4] == 3'b001);

  // Constants depend on the operation type (SIG[2:0]).
  always_comb begin
    case (sig[2:0])
      3'd0: begin // F16
        k_neg_zero = 64'h0000_0000_0000_8000;
        k_one      = 64'h0000_0000_0000_3C00;
        k_ones     = 64'h0000_0000_0000_FFFF;
      end

      3'd1: begin // F32
        k_neg_zero = 64'h0000_0000_8000_0000;
        k_one      = 64'h0000_0000_3F80_0000;
        k_ones     = 64'h0000_0000_FFFF_FFFF;
      end

      3'd4, 3'd5: begin // I64 / U64
        k_neg_zero = 64'b0;
        k_one      = 64'd1;
        k_ones     = 64'hFFFF_FFFF_FFFF_FFFF;
      end

      default: begin // I32 / U32
        k_neg_zero = 64'b0;
        k_one      = 64'd1;
        k_ones     = 64'h0000_0000_FFFF_FFFF;
      end
    endcase
  end

  // Operand codes 20-31 are rejected by the decoder before issue.
  always_comb begin
    case (a_code)
      K_ZERO:     a = 64'b0;
      K_NEG_ZERO: a = k_neg_zero;
      K_ONE:      a = k_one;
      K_ONES:     a = k_ones;
      default:    a = a_code[3] ? s_reg[a_code[2:0]] : v_reg[a_code[2:0]];
    endcase

    case (b_code)
      K_ZERO:     b = 64'b0;
      K_NEG_ZERO: b = k_neg_zero;
      K_ONE:      b = k_one;
      K_ONES:     b = k_ones;
      default:    b = b_code[3] ? s_reg[b_code[2:0]] : v_reg[b_code[2:0]];
    endcase
  end

  vec_alu vec_alu_inst (
    .a(a),
    .b(b),
    .sig(sig),
    .select_predicate(p_live[c_code[2:0]]),
    .strict(strict),
    .error(alu_error),
    .predicate_result(alu_predicate_result),
    .saturated_add(alu_saturated_add),
    .saturated_sub(alu_saturated_sub),
    .result(result)
  );

  // Disabled lanes perform no error check or counter update.
  assign error = enable && alu_error;
  assign saturated_add = enable && alu_saturated_add;
  assign saturated_sub = enable && alu_saturated_sub;
  assign predicate_result = alu_predicate_result;

endmodule
