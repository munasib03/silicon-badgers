// Command decoder: splits the 32-bit word, gives each operand its type, and rejects
// illegal commands before any lane writes.
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
  output logic [3:0]  a_type,            // operand types, used for type-dependent constants
  output logic [3:0]  b_type,
  output logic [3:0]  c_type,
  output logic        is_control,
  output logic        is_alu,
  output logic        is_product,
  output logic        is_reduce,
  output logic        is_cast,
  output logic        is_special,
  output logic        is_data,
  output logic        predicate_dest,    // ALU result is a predicate written to P[D]
  output logic        scalar_dest,       // numeric result is written to S[D-8] (not REDUCE)
  output logic        second_dest,       // DIVMOD: the remainder is written to C
  output logic        multi_cycle,       // DIV, RCP, DIVMOD, DIV_RNE, RSQRT: sequential units
  output logic        illegal
);

  localparam logic [2:0] FAMILY_CONTROL = 3'b000;
  localparam logic [2:0] FAMILY_ALU     = 3'b001;
  localparam logic [2:0] FAMILY_PRODUCT = 3'b010;
  localparam logic [2:0] FAMILY_REDUCE  = 3'b011;
  localparam logic [2:0] FAMILY_CAST    = 3'b100;
  localparam logic [2:0] FAMILY_SPECIAL = 3'b101;
  localparam logic [2:0] FAMILY_DATA    = 3'b110;

  // Operand types (shared with vec_lane)
  localparam logic [3:0] T_F16   = 4'd0;
  localparam logic [3:0] T_F32   = 4'd1;
  localparam logic [3:0] T_I8    = 4'd2;
  localparam logic [3:0] T_U8    = 4'd3;
  localparam logic [3:0] T_I16   = 4'd4;
  localparam logic [3:0] T_U16   = 4'd5;
  localparam logic [3:0] T_I32   = 4'd6;
  localparam logic [3:0] T_U32   = 4'd7;
  localparam logic [3:0] T_I64   = 4'd8;
  localparam logic [3:0] T_U64   = 4'd9;
  localparam logic [3:0] T_RAW16 = 4'd10;
  localparam logic [3:0] T_RAW32 = 4'd11;
  localparam logic [3:0] T_RAW64 = 4'd12;
  localparam logic [3:0] T_NONE  = 4'd15;

  // CONTROL
  localparam logic [6:0] SETVL = 7'd0;
  localparam logic [6:0] PAND  = 7'd1;
  localparam logic [6:0] PNOT  = 7'd2;

  // PRODUCT
  localparam logic [6:0] FMA_F16          = 7'd0;
  localparam logic [6:0] FMA_F32          = 7'd1;
  localparam logic [6:0] FMA_F16_F32      = 7'd2;
  localparam logic [6:0] WMAC_I16_I32     = 7'd3;
  localparam logic [6:0] WMAC_I32_I64     = 7'd4;
  localparam logic [6:0] MULSHIFT_I32_I16 = 7'd5;
  localparam logic [6:0] MULSHIFT_I32_U16 = 7'd6;
  localparam logic [6:0] MUL_CHECK_I64    = 7'd7;
  localparam logic [6:0] MUL_CHECK_U64    = 7'd8;

  // SPECIAL
  localparam logic [6:0] DIV_F16     = 7'd0;
  localparam logic [6:0] DIV_F32     = 7'd1;
  localparam logic [6:0] RCP_F16     = 7'd2;
  localparam logic [6:0] RCP_F32     = 7'd3;
  localparam logic [6:0] DIVMOD_I32  = 7'd4;
  localparam logic [6:0] DIVMOD_U32  = 7'd5;
  localparam logic [6:0] DIVMOD_I64  = 7'd6;
  localparam logic [6:0] DIVMOD_U64  = 7'd7;
  localparam logic [6:0] DIV_RNE_I32 = 7'd8;
  localparam logic [6:0] DIV_RNE_U32 = 7'd9;
  localparam logic [6:0] DIV_RNE_I64 = 7'd10;
  localparam logic [6:0] DIV_RNE_U64 = 7'd11;
  localparam logic [6:0] ISQRT_U32   = 7'd12;
  localparam logic [6:0] ISQRT_U64   = 7'd13;
  localparam logic [6:0] EXP_F16     = 7'd14;
  localparam logic [6:0] EXP_F32     = 7'd15;
  localparam logic [6:0] RSQRT_F16   = 7'd16;
  localparam logic [6:0] RSQRT_F32   = 7'd17;

  // DATA
  localparam logic [6:0] COPY16    = 7'd0;
  localparam logic [6:0] COPY32    = 7'd1;
  localparam logic [6:0] COPY64    = 7'd2;
  localparam logic [6:0] PERMUTE16 = 7'd3;
  localparam logic [6:0] PERMUTE32 = 7'd4;
  localparam logic [6:0] PERMUTE64 = 7'd5;
  localparam logic [6:0] IOTA_U32  = 7'd6;

  localparam logic [4:0] K_ZERO     = 5'd16;
  localparam logic [4:0] K_NEG_ZERO = 5'd17;
  localparam logic [4:0] K_ONES     = 5'd19;

  logic [3:0] fn;
  logic [2:0] alu_type;
  logic select_op, clz_op, shift_op;
  logic legal_sig;
  logic uses_b, uses_c;                  // B / C are numeric inputs
  logic d_scalar;

  // ALU TYPE field -> operand type
  function automatic logic [3:0] alu_operand_type(input logic [2:0] t);
    case (t)
      3'd0:    alu_operand_type = T_F16;
      3'd1:    alu_operand_type = T_F32;
      3'd2:    alu_operand_type = T_I32;
      3'd3:    alu_operand_type = T_U32;
      3'd4:    alu_operand_type = T_I64;
      default: alu_operand_type = T_U64;
    endcase
  endfunction

  // CAST type number -> operand type
  function automatic logic [3:0] cast_operand_type(input logic [2:0] t);
    case (t)
      3'd0:    cast_operand_type = T_F16;
      3'd1:    cast_operand_type = T_F32;
      3'd2:    cast_operand_type = T_I8;
      3'd3:    cast_operand_type = T_U8;
      3'd4:    cast_operand_type = T_I16;
      3'd5:    cast_operand_type = T_U16;
      3'd6:    cast_operand_type = T_I32;
      default: cast_operand_type = T_I64;
    endcase
  endfunction

  // A numeric operand: a V/S register or a constant that suits the type.
  function automatic logic operand_ok(input logic [4:0] code, input logic [3:0] t);
    logic fp;
    fp = (t == T_F16) || (t == T_F32);
    operand_ok = (t != T_NONE) &&
                 (code <= K_ONES) &&
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

  assign is_control = (family == FAMILY_CONTROL);
  assign is_alu     = (family == FAMILY_ALU);
  assign is_product = (family == FAMILY_PRODUCT);
  assign is_reduce  = (family == FAMILY_REDUCE);
  assign is_cast    = (family == FAMILY_CAST);
  assign is_special = (family == FAMILY_SPECIAL);
  assign is_data    = (family == FAMILY_DATA);

  assign select_op = (fn == 4'd4);
  assign clz_op    = (fn == 4'd9);
  assign shift_op  = (fn == 4'd7) || (fn == 4'd8) || (fn == 4'd12);   // SHL SHR RSHIFT_RNE

  assign d_scalar       = d_code[3];
  assign predicate_dest = is_alu && ((fn == 4'd2) || (fn == 4'd3)); // EQ / LT
  assign second_dest    = is_special && (sig >= DIVMOD_I32) && (sig <= DIVMOD_U64);
  assign multi_cycle    = is_special && ((sig <= DIV_RNE_U64) || (sig == RSQRT_F16) || (sig == RSQRT_F32));
  assign scalar_dest    = d_scalar && !predicate_dest &&
                          (is_alu || is_product || is_cast || is_special || is_data);

  // Operand types per signature (Signatures sheet).
  always_comb begin
    a_type = T_NONE;
    b_type = T_NONE;
    c_type = T_NONE;

    case (family)
      FAMILY_ALU: begin
        a_type = alu_operand_type(alu_type);
        b_type = clz_op ? T_NONE : (shift_op ? T_U32 : alu_operand_type(alu_type));
      end

      FAMILY_PRODUCT: begin
        case (sig)
          FMA_F16:          begin a_type = T_F16; b_type = T_F16; c_type = T_F16; end
          FMA_F32:          begin a_type = T_F32; b_type = T_F32; c_type = T_F32; end
          FMA_F16_F32:      begin a_type = T_F16; b_type = T_F16; c_type = T_F32; end
          WMAC_I16_I32:     begin a_type = T_I16; b_type = T_I16; c_type = T_I32; end
          WMAC_I32_I64:     begin a_type = T_I32; b_type = T_I32; c_type = T_I64; end
          MULSHIFT_I32_I16: begin a_type = T_I32; b_type = T_I16; c_type = T_U32; end
          MULSHIFT_I32_U16: begin a_type = T_I32; b_type = T_U16; c_type = T_U32; end
          MUL_CHECK_I64:    begin a_type = T_I64; b_type = T_I64; end
          MUL_CHECK_U64:    begin a_type = T_U64; b_type = T_U64; end
          default: ;
        endcase
      end

      FAMILY_REDUCE: begin
        // SUM: F16 F32 I32 I64 U32 U64; MAX/MIN: F16 F32 I32 U32 I64 U64
        case (sig)
          7'd0, 7'd6, 7'd12:  a_type = T_F16;
          7'd1, 7'd7, 7'd13:  a_type = T_F32;
          7'd2, 7'd8, 7'd14:  a_type = T_I32;
          7'd3, 7'd10, 7'd16: a_type = T_I64;
          7'd4, 7'd9, 7'd15:  a_type = T_U32;
          7'd5, 7'd11, 7'd17: a_type = T_U64;
          default: ;
        endcase
      end

      FAMILY_CAST: begin
        a_type = cast_operand_type(sig[5:3]);
        b_type = T_I32;                          // exponent e
      end

      FAMILY_SPECIAL: begin
        case (sig)
          DIV_F16:                 begin a_type = T_F16; b_type = T_F16; end
          DIV_F32:                 begin a_type = T_F32; b_type = T_F32; end
          RCP_F16, EXP_F16,
          RSQRT_F16:               a_type = T_F16;
          RCP_F32, EXP_F32,
          RSQRT_F32:               a_type = T_F32;
          DIVMOD_I32, DIV_RNE_I32: begin a_type = T_I32; b_type = T_I32; end
          DIVMOD_U32, DIV_RNE_U32: begin a_type = T_U32; b_type = T_U32; end
          DIVMOD_I64, DIV_RNE_I64: begin a_type = T_I64; b_type = T_I64; end
          DIVMOD_U64, DIV_RNE_U64: begin a_type = T_U64; b_type = T_U64; end
          ISQRT_U32:               a_type = T_U32;
          ISQRT_U64:               a_type = T_U64;
          default: ;
        endcase
      end

      FAMILY_DATA: begin
        case (sig)
          COPY16:    a_type = T_RAW16;
          COPY32:    a_type = T_RAW32;
          COPY64:    a_type = T_RAW64;
          PERMUTE16: begin a_type = T_RAW16; b_type = T_U32; end
          PERMUTE32: begin a_type = T_RAW32; b_type = T_U32; end
          PERMUTE64: begin a_type = T_RAW64; b_type = T_U32; end
          default: ;
        endcase
      end

      default: ;
    endcase
  end

  always_comb begin
    illegal = 1'b0;
    legal_sig = 1'b0;
    uses_b = (b_type != T_NONE);
    uses_c = (c_type != T_NONE);

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
                  !operand_ok(a_code, a_type) ||
                  (clz_op ? (b_code != 0) : !operand_ok(b_code, b_type)) ||
                  (select_op ? (c_code > 5'd7) : (c_code != 0)) ||
                  // Comparisons write P1..P7; P0 is read-only.
                  (predicate_dest && ((d_code == 0) || (d_code > 4'd7))) ||
                  // A scalar result needs scalar or constant inputs.
                  (scalar_dest && ((a_code < 5'd8) || (!clz_op && (b_code < 5'd8))));
      end

      FAMILY_PRODUCT: begin
        illegal = (sig > MUL_CHECK_U64) ||
                  !operand_ok(a_code, a_type) ||
                  !operand_ok(b_code, b_type) ||
                  (uses_c ? !operand_ok(c_code, c_type) : (c_code != 0)) ||
                  (scalar_dest && ((a_code < 5'd8) || (b_code < 5'd8) ||
                                   (uses_c && (c_code < 5'd8))));
      end

      FAMILY_REDUCE: begin
        // The result is always a scalar; B and C are unused.
        illegal = (sig > 7'd17) ||
                  !d_scalar ||
                  !operand_ok(a_code, a_type) ||
                  (b_code != 0) || (c_code != 0);
      end

      FAMILY_CAST: begin
        case (sig)
          7'd0, 7'd1, 7'd6, 7'd7, 7'd8, 7'd9, 7'd14, 7'd15,
          7'd22, 7'd30, 7'd38, 7'd46,
          7'd48, 7'd49, 7'd50, 7'd51, 7'd52, 7'd53, 7'd54, 7'd55,
          7'd56, 7'd57, 7'd62, 7'd63: legal_sig = 1'b1;
          default:                    legal_sig = 1'b0;
        endcase

        illegal = !legal_sig ||
                  !operand_ok(a_code, a_type) ||
                  !operand_ok(b_code, b_type) ||
                  (c_code != 0) ||
                  (scalar_dest && ((a_code < 5'd8) || (b_code < 5'd8)));
      end

      FAMILY_SPECIAL: begin
        illegal = (sig > RSQRT_F32) ||
                  !operand_ok(a_code, a_type) ||
                  (uses_b ? !operand_ok(b_code, b_type) : (b_code != 0)) ||
                  (scalar_dest && ((a_code < 5'd8) || (uses_b && (b_code < 5'd8))));

        if (second_dest)
          // DIVMOD: C is the remainder register; distinct from D, and both
          // vector or both scalar.
          illegal = illegal || (c_code > 5'd15) ||
                    (c_code[3:0] == d_code) || (c_code[3] != d_code[3]);
        else
          illegal = illegal || (c_code != 0);
      end

      FAMILY_DATA: begin
        case (sig)
          COPY16, COPY32, COPY64:
            illegal = !operand_ok(a_code, a_type) || (b_code != 0) || (c_code != 0) ||
                      (scalar_dest && (a_code < 5'd8));

          // PERMUTE reads and writes vector registers; B is a U32 lane index.
          PERMUTE16, PERMUTE32, PERMUTE64:
            illegal = (d_code > 4'd7) || (a_code > 5'd7) ||
                      !operand_ok(b_code, b_type) || (c_code != 0);

          // IOTA writes a vector register and has no inputs.
          IOTA_U32:
            illegal = (d_code > 4'd7) || (a_code != 0) || (b_code != 0) || (c_code != 0);

          default: illegal = 1'b1;
        endcase
      end

      default: illegal = 1'b1;   // FAMILY 111 is invalid

    endcase
  end

endmodule
