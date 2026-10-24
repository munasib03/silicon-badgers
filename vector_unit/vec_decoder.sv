// Command decoder: splits the 32-bit word and rejects illegal commands before
// any lane writes. Covers the CONTROL and ALU families; other families are illegal
// until their execution units exist.
// Word: FAMILY[31:29] | SIG[28:22] | D[21:18] | A[17:13] | B[12:8] | C[7:3] | PM[2:0]
module vec_decoder (
  input  logic [31:0] command,
  output logic [2:0]  family,
  output logic [6:0]  sig,
  output logic [3:0]  d_code,
  output logic [4:0]  a_code,
  output logic [4:0]  b_code,
  output logic [4:0]  c_code,
  output logic [2:0]  pm,
  output logic        is_control,
  output logic        is_alu,
  output logic        predicate_dest,    // ALU result is a predicate written to P[D]
  output logic        scalar_dest,       // ALU numeric result is written to S[D-8]
  output logic        illegal
);

  localparam logic [2:0] FAMILY_CONTROL = 3'b000;
  localparam logic [2:0] FAMILY_ALU     = 3'b001;

  localparam logic [6:0] SETVL = 7'd0;
  localparam logic [6:0] PAND  = 7'd1;
  localparam logic [6:0] PNOT  = 7'd2;

  localparam logic [4:0] K_NEG_ZERO = 5'd17;
  localparam logic [4:0] K_ONES     = 5'd19;

  logic [3:0] fn;
  logic [2:0] alu_type;
  logic fp_type;
  logic select_op, clz_op;
  logic legal_sig;

  // A numeric operand: register or constant, and the constant must suit the type.
  function automatic logic operand_ok(input logic [4:0] code, input logic fp);
    operand_ok = (code <= K_ONES) &&
                 !((code == K_NEG_ZERO) && !fp) &&   // floating negative zero
                 !((code == K_ONES) && fp);          // integer all-ones
  endfunction

  assign family = command[31:29];
  assign sig    = command[28:22];
  assign d_code = command[21:18];
  assign a_code = command[17:13];
  assign b_code = command[12:8];
  assign c_code = command[7:3];
  assign pm     = command[2:0];

  // ALU SIG = FN*8 + TYPE
  assign fn       = sig[6:3];
  assign alu_type = sig[2:0];
  assign fp_type  = (alu_type == 3'd0) || (alu_type == 3'd1);

  assign is_control = (family == FAMILY_CONTROL);
  assign is_alu     = (family == FAMILY_ALU);

  assign select_op      = (fn == 4'd4);
  assign clz_op         = (fn == 4'd9);
  assign predicate_dest = is_alu && ((fn == 4'd2) || (fn == 4'd3)); // EQ / LT
  assign scalar_dest    = is_alu && !predicate_dest && d_code[3];

  always_comb begin
    illegal = 1'b0;
    legal_sig = 1'b0;

    case (family)

      FAMILY_CONTROL: begin
        case (sig)
          // A is a U32 scalar or constant; D, B, C unused.
          SETVL: illegal = (d_code != 0) || (b_code != 0) || (c_code != 0) ||
                           (a_code < 5'd8) || (a_code > K_ONES) || (a_code == K_NEG_ZERO);

          // D = A AND B over predicates; D must be P1..P7.
          PAND:  illegal = (d_code == 0) || (d_code > 4'd7) ||
                           (a_code > 5'd7) || (b_code > 5'd7) || (c_code != 0);

          // D = NOT A over predicates; D must be P1..P7.
          PNOT:  illegal = (d_code == 0) || (d_code > 4'd7) ||
                           (a_code > 5'd7) || (b_code != 0) || (c_code != 0);

          default: illegal = 1'b1;
        endcase

        // CONTROL commands require PM = 0.
        illegal = illegal || (pm != 0);
      end

      FAMILY_ALU: begin
        // Signatures sheet: which TYPEs each FN supports.
        case (fn)
          4'd0, 4'd1, 4'd2, 4'd3, 4'd4: legal_sig = (alu_type <= 3'd5);                       // ADD SUB EQ LT SELECT
          4'd5, 4'd6, 4'd7, 4'd8, 4'd9: legal_sig = (alu_type >= 3'd2) && (alu_type <= 3'd5); // AND XOR SHL SHR CLZ
          4'd10, 4'd11, 4'd12:          legal_sig = (alu_type == 3'd2) || (alu_type == 3'd4); // ADD_SAT SUB_SAT RSHIFT_RNE
          default:                      legal_sig = 1'b0;
        endcase

        illegal = !legal_sig ||
                  !operand_ok(a_code, fp_type) ||
                  (clz_op ? (b_code != 0) : !operand_ok(b_code, fp_type)) ||
                  (select_op ? (c_code > 5'd7) : (c_code != 0)) ||
                  // Comparisons write P1..P7; P0 is read-only.
                  (predicate_dest && ((d_code == 0) || (d_code > 4'd7))) ||
                  // A scalar result needs scalar or constant inputs.
                  (scalar_dest && ((a_code < 5'd8) || (!clz_op && (b_code < 5'd8))));
      end

      default: illegal = 1'b1;

    endcase
  end

endmodule
