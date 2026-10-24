// One SIMD lane: lane-local V0..V7, operand muxes, lane enable, execution units,
// and writeback. Instruction-wide decisions (legality, commit on any-lane error,
// scalar-once, S and P writes) belong to vec_simd.
module vec_lane #(
  parameter int LANE_INDEX = 0
) (
  input  logic        clk,
  input  logic        rst_n,

  input  logic [2:0]  family,
  input  logic [6:0]  sig,
  input  logic [3:0]  d_code,            // D field: 0-7 V, 8-15 S
  input  logic [4:0]  a_code,            // A field: 0-7 V, 8-15 S, 16-19 constants
  input  logic [4:0]  b_code,            // B field: same encoding as A
  input  logic [4:0]  c_code,            // C field: FMA addend (as A) or SELECT predicate P0..P7
  input  logic [2:0]  pm,                // execution mask predicate P0..P7
  input  logic [31:0] vl,                // active vector length
  input  logic        strict,            // 1: saturation is an error
  input  logic        commit,            // command passed every lane's checks

  input  logic [63:0] s_reg [8],         // S0..S7, broadcast to all lanes
  input  logic [7:0]  p_bits,            // this lane's bit of P0..P7

  output logic        enable,            // lane is below VL and its PM bit is true
  output logic        error,             // only raised by enabled lanes
  output logic        saturated_add,
  output logic        saturated_sub,
  output logic        predicate_result,
  output logic [63:0] result
);

  localparam logic [2:0] FAMILY_ALU     = 3'b001;
  localparam logic [2:0] FAMILY_PRODUCT = 3'b010;

  localparam logic [6:0] FMA_F16_F32 = 7'b000_0010;

  // Operand types, same codes as ALU SIG[2:0].
  localparam logic [2:0] F16 = 3'd0;
  localparam logic [2:0] F32 = 3'd1;
  localparam logic [2:0] I64 = 3'd4;
  localparam logic [2:0] U64 = 3'd5;

  localparam logic [4:0] K_ZERO     = 5'd16;
  localparam logic [4:0] K_NEG_ZERO = 5'd17;
  localparam logic [4:0] K_ONE      = 5'd18;
  localparam logic [4:0] K_ONES     = 5'd19;

  logic [63:0] v_reg [8];

  logic [7:0]  p_live;
  logic [2:0]  a_type, b_type, c_type;
  logic [63:0] a, b, c;
  logic        write_predicate;

  logic        alu_error, alu_saturated_add, alu_saturated_sub;
  logic [63:0] alu_result;
  logic        product_error;
  logic [63:0] product_result;

  // Constants depend on the operand's type.
  function automatic logic [63:0] constant(input logic [4:0] code, input logic [2:0] op_type);
    case (code)
      K_NEG_ZERO: begin
        case (op_type)
          F16:     constant = 64'h0000_0000_0000_8000;
          F32:     constant = 64'h0000_0000_8000_0000;
          default: constant = 64'b0;
        endcase
      end

      K_ONE: begin
        case (op_type)
          F16:     constant = 64'h0000_0000_0000_3C00;
          F32:     constant = 64'h0000_0000_3F80_0000;
          default: constant = 64'd1;
        endcase
      end

      K_ONES: begin
        case (op_type)
          F16:      constant = 64'h0000_0000_0000_FFFF;
          I64, U64: constant = 64'hFFFF_FFFF_FFFF_FFFF;
          default:  constant = 64'h0000_0000_FFFF_FFFF;
        endcase
      end

      default: constant = 64'b0; // K_ZERO
    endcase
  endfunction

  // P0 is always true.
  assign p_live = {p_bits[7:1], 1'b1};

  assign enable = (LANE_INDEX[31:0] < vl) && p_live[pm];

  // EQ and LT (ALU FN 2, 3) are the only operations that write a predicate.
  assign write_predicate = (family == FAMILY_ALU) && (sig[6:4] == 3'b001);

  // ALU operands all use SIG[2:0]. FMA_F16_F32 has F16 A/B and an F32 addend.
  always_comb begin
    a_type = sig[2:0];
    b_type = sig[2:0];
    c_type = sig[2:0];
    if ((family == FAMILY_PRODUCT) && (sig == FMA_F16_F32)) begin
      a_type = F16;
      b_type = F16;
      c_type = F32;
    end
  end

  // Operand codes 20-31 are rejected by vec_decoder.
  always_comb begin
    if (a_code >= K_ZERO)
      a = constant(a_code, a_type);
    else
      a = a_code[3] ? s_reg[a_code[2:0]] : v_reg[a_code[2:0]];

    if (b_code >= K_ZERO)
      b = constant(b_code, b_type);
    else
      b = b_code[3] ? s_reg[b_code[2:0]] : v_reg[b_code[2:0]];

    if (c_code >= K_ZERO)
      c = constant(c_code, c_type);
    else
      c = c_code[3] ? s_reg[c_code[2:0]] : v_reg[c_code[2:0]];
  end

  vec_alu vec_alu_inst (
    .a(a),
    .b(b),
    .sig(sig),
    .select_predicate(p_live[c_code[2:0]]),
    .strict(strict),
    .error(alu_error),
    .predicate_result(predicate_result),
    .saturated_add(alu_saturated_add),
    .saturated_sub(alu_saturated_sub),
    .result(alu_result)
  );

  vec_product vec_product_inst (
    .a(a),
    .b(b),
    .c(c),
    .sig(sig),
    .error(product_error),
    .result(product_result)
  );

  always_comb begin
    case (family)
      FAMILY_PRODUCT: begin
        result = product_result;
        error = enable && product_error;
      end

      default: begin // FAMILY_ALU
        result = alu_result;
        error = enable && alu_error;
      end
    endcase
  end

  // Disabled lanes perform no error check or counter update.
  assign saturated_add = enable && (family == FAMILY_ALU) && alu_saturated_add;
  assign saturated_sub = enable && (family == FAMILY_ALU) && alu_saturated_sub;

  // Writeback: vector destinations only. Registers start at zero.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < 8; i++)
        v_reg[i] <= 64'b0;
    end else if (commit && enable && !write_predicate && !d_code[3]) begin
      v_reg[d_code[2:0]] <= result;
    end
  end

endmodule
