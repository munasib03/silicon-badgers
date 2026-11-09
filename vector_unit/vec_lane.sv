// One SIMD lane: lane-local V0..V7, operand muxes, lane enable, execution units,
// and writeback. Instruction-wide decisions (legality, commit on any-lane error,
// scalar-once, S and P writes, REDUCE, PERMUTE routing) belong to vec_simd.
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
  input  logic [4:0]  c_code,            // C field: numeric (as A), SELECT predicate, or DIVMOD remainder D
  input  logic [3:0]  a_type,            // operand types from vec_decoder
  input  logic [3:0]  b_type,
  input  logic [3:0]  c_type,
  input  logic [2:0]  pm,                // execution mask predicate P0..P7
  input  logic [31:0] vl,                // active vector length
  input  logic        strict,            // 1: saturation is an error
  input  logic        commit,            // command passed every lane's checks
  input  logic        start,             // first cycle of a multi-cycle SPECIAL operation

  input  logic [63:0] s_reg [8],         // S0..S7, broadcast to all lanes
  input  logic [7:0]  p_bits,            // this lane's bit of P0..P7
  input  logic [63:0] permute_data,      // PERMUTE: A from the lane this lane's B selects

  output logic        enable,            // lane is below VL and its PM bit is true
  output logic        finished,          // multi-cycle units are done (high for other ops)
  output logic        error,             // only raised by enabled lanes
  output logic        saturated_add,
  output logic        saturated_sub,
  output logic        saturated_product,
  output logic        saturated_convert,
  output logic        predicate_result,
  output logic [63:0] a_value,           // operand A (REDUCE input, PERMUTE source)
  output logic [63:0] b_value,           // operand B (PERMUTE index)
  output logic [63:0] result,
  output logic [63:0] result2            // DIVMOD remainder
);

  localparam logic [2:0] FAMILY_ALU     = 3'b001;
  localparam logic [2:0] FAMILY_PRODUCT = 3'b010;
  localparam logic [2:0] FAMILY_CAST    = 3'b100;
  localparam logic [2:0] FAMILY_SPECIAL = 3'b101;
  localparam logic [2:0] FAMILY_DATA    = 3'b110;

  // Operand types (shared with vec_decoder)
  localparam logic [3:0] T_F16   = 4'd0;
  localparam logic [3:0] T_F32   = 4'd1;
  localparam logic [3:0] T_I8    = 4'd2;
  localparam logic [3:0] T_U8    = 4'd3;
  localparam logic [3:0] T_I16   = 4'd4;
  localparam logic [3:0] T_U16   = 4'd5;
  localparam logic [3:0] T_I64   = 4'd8;
  localparam logic [3:0] T_U64   = 4'd9;
  localparam logic [3:0] T_RAW16 = 4'd10;
  localparam logic [3:0] T_RAW64 = 4'd12;

  // DATA
  localparam logic [6:0] COPY16    = 7'd0;
  localparam logic [6:0] COPY32    = 7'd1;
  localparam logic [6:0] COPY64    = 7'd2;
  localparam logic [6:0] PERMUTE16 = 7'd3;
  localparam logic [6:0] PERMUTE32 = 7'd4;
  localparam logic [6:0] PERMUTE64 = 7'd5;
  localparam logic [6:0] IOTA_U32  = 7'd6;

  localparam logic [6:0] DIVMOD_I32 = 7'd4;
  localparam logic [6:0] DIVMOD_U64 = 7'd7;

  localparam logic [4:0] K_ZERO     = 5'd16;
  localparam logic [4:0] K_NEG_ZERO = 5'd17;
  localparam logic [4:0] K_ONE      = 5'd18;
  localparam logic [4:0] K_ONES     = 5'd19;

  logic [63:0] v_reg [8];

  logic [7:0]  p_live;
  logic [63:0] a, b, c;
  logic        write_predicate, write_second;

  logic        alu_error, alu_saturated_add, alu_saturated_sub;
  logic [63:0] alu_result;
  logic        product_error, product_saturated;
  logic [63:0] product_result;
  logic        cast_error, cast_saturated;
  logic [63:0] cast_result;
  logic        special_error;
  logic [63:0] special_result, special_result2;
  logic [63:0] data_result;

  // Constants depend on the operand's type.
  function automatic logic [63:0] constant(input logic [4:0] code, input logic [3:0] op_type);
    case (code)
      K_NEG_ZERO: begin
        case (op_type)
          T_F16:   constant = 64'h0000_0000_0000_8000;
          T_F32:   constant = 64'h0000_0000_8000_0000;
          default: constant = 64'b0;
        endcase
      end

      K_ONE: begin
        case (op_type)
          T_F16:   constant = 64'h0000_0000_0000_3C00;
          T_F32:   constant = 64'h0000_0000_3F80_0000;
          default: constant = 64'd1;
        endcase
      end

      // Integer all-ones fills the type's width.
      K_ONES: begin
        case (op_type)
          T_I8, T_U8:                 constant = 64'h0000_0000_0000_00FF;
          T_F16, T_I16, T_U16,
          T_RAW16:                    constant = 64'h0000_0000_0000_FFFF;
          T_I64, T_U64, T_RAW64:      constant = 64'hFFFF_FFFF_FFFF_FFFF;
          default:                    constant = 64'h0000_0000_FFFF_FFFF;
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

  // DIVMOD also writes its remainder to C.
  assign write_second = (family == FAMILY_SPECIAL) && (sig >= DIVMOD_I32) && (sig <= DIVMOD_U64);

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

  assign a_value = a;
  assign b_value = b;

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
    .strict(strict),
    .error(product_error),
    .saturated(product_saturated),
    .result(product_result)
  );

  vec_cast vec_cast_inst (
    .a(a),
    .b(b),
    .sig(sig),
    .strict(strict),
    .error(cast_error),
    .saturated(cast_saturated),
    .result(cast_result)
  );

  vec_special vec_special_inst (
    .clk(clk),
    .rst_n(rst_n),
    .start(start),
    .a(a),
    .b(b),
    .sig(sig),
    .finished(finished),
    .error(special_error),
    .result(special_result),
    .result2(special_result2)
  );

  // DATA: raw moves with no numeric checks. PERMUTE's source comes from vec_simd.
  always_comb begin
    case (sig)
      COPY16:    data_result = {48'b0, a[15:0]};
      COPY32:    data_result = {32'b0, a[31:0]};
      COPY64:    data_result = a;
      PERMUTE16: data_result = {48'b0, permute_data[15:0]};
      PERMUTE32: data_result = {32'b0, permute_data[31:0]};
      PERMUTE64: data_result = permute_data;
      IOTA_U32:  data_result = {32'b0, LANE_INDEX[31:0]};
      default:   data_result = 64'b0;
    endcase
  end

  always_comb begin
    result2 = 64'b0;
    case (family)
      FAMILY_PRODUCT: begin
        result = product_result;
        error = enable && product_error;
      end

      FAMILY_CAST: begin
        result = cast_result;
        error = enable && cast_error;
      end

      FAMILY_SPECIAL: begin
        result = special_result;
        result2 = special_result2;
        error = enable && special_error;
      end

      FAMILY_DATA: begin
        // PERMUTE index errors are checked in vec_simd.
        result = data_result;
        error = 1'b0;
      end

      FAMILY_ALU: begin
        result = alu_result;
        error = enable && alu_error;
      end

      default: begin // CONTROL and REDUCE are handled in vec_simd.
        result = 64'b0;
        error = 1'b0;
      end
    endcase
  end

  // Disabled lanes perform no error check or counter update.
  assign saturated_add     = enable && (family == FAMILY_ALU) && alu_saturated_add;
  assign saturated_sub     = enable && (family == FAMILY_ALU) && alu_saturated_sub;
  assign saturated_product = enable && (family == FAMILY_PRODUCT) && product_saturated;
  assign saturated_convert = enable && (family == FAMILY_CAST) && cast_saturated;

  // Writeback: vector destinations only. Registers start at zero. All inputs were
  // read combinationally above, so outputs may reuse input registers.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < 8; i++)
        v_reg[i] <= 64'b0;
    end else if (commit && enable) begin
      if (!write_predicate && !d_code[3])
        v_reg[d_code[2:0]] <= result;
      // DIVMOD remainder; vec_decoder guarantees C != D and C is V when D is V.
      if (write_second && !c_code[3])
        v_reg[c_code[2:0]] <= result2;
    end
  end

endmodule
