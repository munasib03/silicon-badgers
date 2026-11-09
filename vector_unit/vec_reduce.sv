// REDUCE family: SUM, MAX, MIN across lanes into one scalar.
// SUM uses the spec's fixed tree: adjacent logical positions are paired, then their
// partial sums, padded with zeros to the next power of two of VL. Masked lanes
// supply zero. Positions at or beyond that power of two are not part of the tree
// (they pass the other side through), so the result does not depend on NUM_LANES.
// MAX/MIN keep the lowest enabled lane on ties; +0 and -0 tie.
// With no enabled lane every reduction writes zero.
module vec_reduce #(
  parameter int NUM_LANES = 8
) (
  input  logic [63:0]          lane_value [NUM_LANES],   // each lane's A operand
  input  logic [NUM_LANES-1:0] lane_enable,
  input  logic [31:0]          vl,
  input  logic [6:0]           sig,
  output logic                 error,
  output logic [63:0]          result
);

  // REDUCE SIG encodings (Signatures sheet)
  localparam logic [6:0] SUM_F16_F32 = 7'd0;
  localparam logic [6:0] SUM_F32_F32 = 7'd1;
  localparam logic [6:0] SUM_I32_I64 = 7'd2;
  localparam logic [6:0] SUM_I64_I64 = 7'd3;
  localparam logic [6:0] SUM_U32_U64 = 7'd4;
  localparam logic [6:0] SUM_U64_U64 = 7'd5;
  localparam logic [6:0] MAX_F16     = 7'd6;
  localparam logic [6:0] MAX_F32     = 7'd7;
  localparam logic [6:0] MAX_I32     = 7'd8;
  localparam logic [6:0] MAX_U32     = 7'd9;
  localparam logic [6:0] MAX_I64     = 7'd10;
  localparam logic [6:0] MAX_U64     = 7'd11;
  localparam logic [6:0] MIN_F16     = 7'd12;
  localparam logic [6:0] MIN_F32     = 7'd13;
  localparam logic [6:0] MIN_I32     = 7'd14;
  localparam logic [6:0] MIN_U32     = 7'd15;
  localparam logic [6:0] MIN_I64     = 7'd16;
  localparam logic [6:0] MIN_U64     = 7'd17;

  // Tree size: NUM_LANES rounded up to a power of two. Heap layout: node 1 is the
  // root, node n has children 2n and 2n+1, leaves are TREE..2*TREE-1.
  localparam int LEVELS = (NUM_LANES > 1) ? $clog2(NUM_LANES) : 0;
  localparam int TREE   = 1 << LEVELS;

  logic [2*TREE-1:0] present;              // node lies inside the logical tree
  logic [31:0]       fsum  [2*TREE];       // F32 partial sums
  logic [31:0]       fadd  [2*TREE];       // adder outputs
  logic [64:0]       isum  [2*TREE];       // integer partial sums, one extra bit
  logic [31:0]       tree_size;            // next power of two >= VL

  logic sum_float, sum_signed, sum_from32, is_max, is_min;
  logic float_error, int_overflow;
  logic [2:0] cmp_type;                    // 0 F16, 1 F32, 2 I32, 3 U32, 4 I64, 5 U64

  logic        found;
  logic [63:0] best_value, best_key, key;

  // Exact F16 -> F32 widening.
  function automatic logic [31:0] f16_to_f32(input logic [15:0] h);
    integer msb;
    logic [10:0] shifted;
    f16_to_f32 = {h[15], 31'b0};
    if (h[14:10] == 5'd31) begin
      f16_to_f32 = {h[15], 8'hFF, h[9:0], 13'b0};
    end else if (h[14:10] != 5'd0) begin
      f16_to_f32 = {h[15], ({3'b0, h[14:10]} + 8'd112), h[9:0], 13'b0};
    end else if (h[9:0] != 10'd0) begin
      msb = 0;
      for (int i = 0; i < 10; i++) begin
        if (h[i]) msb = i;
      end
      shifted = {1'b0, h[9:0]} << (10 - msb);
      f16_to_f32 = {h[15], 8'(msb + 103), shifted[9:0], 13'b0};
    end
  endfunction

  // Unsigned ordering key; +0 and -0 map to the same key.
  function automatic logic [63:0] order_key(input logic [63:0] v, input logic [2:0] t);
    case (t)
      3'd0: order_key = (v[14:0] == 0) ? 64'h8000 :
                        (v[15] ? {48'b0, ~v[15:0]} : {48'b0, 1'b1, v[14:0]});
      3'd1: order_key = (v[30:0] == 0) ? 64'h8000_0000 :
                        (v[31] ? {32'b0, ~v[31:0]} : {32'b0, 1'b1, v[30:0]});
      3'd2: order_key = {32'b0, ~v[31], v[30:0]};
      3'd3: order_key = {32'b0, v[31:0]};
      3'd4: order_key = {~v[63], v[62:0]};
      default: order_key = v;
    endcase
  endfunction

  assign sum_float  = (sig == SUM_F16_F32) || (sig == SUM_F32_F32);
  assign sum_signed = (sig == SUM_I32_I64) || (sig == SUM_I64_I64);
  assign sum_from32 = (sig == SUM_I32_I64) || (sig == SUM_U32_U64);
  assign is_max = (sig >= MAX_F16) && (sig <= MAX_U64);
  assign is_min = (sig >= MIN_F16) && (sig <= MIN_U64);
  assign cmp_type = is_max ? 3'(sig - MAX_F16) : 3'(sig - MIN_F16);

  always_comb begin
    tree_size = 32'd1;
    for (int i = 0; i < 32; i++) begin
      if (tree_size < vl)
        tree_size = tree_size << 1;
    end
  end

  // Leaves.
  genvar n;
  generate
    for (n = 0; n < TREE; n++) begin : leaves
      if (n < NUM_LANES) begin : lane
        assign present[TREE+n] = (n < tree_size);
        assign fsum[TREE+n] = !lane_enable[n] ? 32'b0 :
                              (sig == SUM_F16_F32) ? f16_to_f32(lane_value[n][15:0]) :
                                                     lane_value[n][31:0];
        assign isum[TREE+n] = !lane_enable[n] ? 65'b0 :
                              !sum_from32     ? {sum_signed && lane_value[n][63], lane_value[n]} :
                              sum_signed      ? {{33{lane_value[n][31]}}, lane_value[n][31:0]} :
                                                {33'b0, lane_value[n][31:0]};
      end else begin : pad
        assign present[TREE+n] = (n < tree_size);
        assign fsum[TREE+n] = 32'b0;
        assign isum[TREE+n] = 65'b0;
      end
      assign fadd[TREE+n] = 32'b0;
    end

    // Internal nodes: add when both sides are in the tree, else pass one through.
    for (n = 1; n < TREE; n++) begin : nodes
      fp32_addsub add_inst (
        .a(fsum[2*n]),
        .b(fsum[2*n+1]),
        .sub(1'b0),
        .d(fadd[n])
      );

      assign present[n] = present[2*n] || present[2*n+1];
      assign fsum[n] = (present[2*n] && present[2*n+1]) ? fadd[n] :
                       present[2*n] ? fsum[2*n] : fsum[2*n+1];
      assign isum[n] = (present[2*n] && present[2*n+1]) ? (isum[2*n] + isum[2*n+1]) :
                       present[2*n] ? isum[2*n] : isum[2*n+1];
    end
  endgenerate

  assign present[0] = 1'b0;
  assign fsum[0] = 32'b0;
  assign fadd[0] = 32'b0;
  assign isum[0] = 65'b0;

  // Every enabled float input must be finite; so must every partial sum.
  // Every integer partial sum must fit the destination.
  always_comb begin
    float_error = 1'b0;
    int_overflow = 1'b0;
    for (int i = 0; i < NUM_LANES; i++) begin
      if (lane_enable[i] && sum_float && (&fsum[TREE+i][30:23]))
        float_error = 1'b1;
    end
    for (int i = 1; i < TREE; i++) begin
      if (present[2*i] && present[2*i+1]) begin
        if (&fadd[i][30:23])
          float_error = 1'b1;
        if (sum_signed ? (isum[i][64] != isum[i][63]) : isum[i][64])
          int_overflow = 1'b1;
      end
    end
  end

  // MAX / MIN: scan in lane order and replace only on a strictly better key,
  // so ties keep the lowest enabled lane.
  always_comb begin
    found = 1'b0;
    best_value = 64'b0;
    best_key = 64'b0;
    key = 64'b0;
    for (int i = 0; i < NUM_LANES; i++) begin
      key = order_key(lane_value[i], cmp_type);
      if (lane_enable[i]) begin
        if (!found || (is_max ? (key > best_key) : (key < best_key))) begin
          best_value = lane_value[i];
          best_key = key;
        end
        found = 1'b1;
      end
    end
  end

  always_comb begin
    error = 1'b0;
    result = 64'b0;

    if (sum_float) begin
      error = float_error;
      result = {32'b0, fsum[1]};
    end else if (sig <= SUM_U64_U64) begin
      error = int_overflow;
      result = isum[1][63:0];
    end else if (is_max || is_min) begin
      // Float inputs in enabled lanes must be finite.
      for (int i = 0; i < NUM_LANES; i++) begin
        if (lane_enable[i] && (((cmp_type == 3'd0) && (&lane_value[i][14:10])) ||
                               ((cmp_type == 3'd1) && (&lane_value[i][30:23]))))
          error = 1'b1;
      end
      case (cmp_type)
        3'd0:       result = {48'b0, best_value[15:0]};
        3'd1, 3'd2,
        3'd3:       result = {32'b0, best_value[31:0]};
        default:    result = best_value;
      endcase
    end else begin
      error = 1'b1;
    end

    // No enabled lane: write zero (positive zero for floats).
    if (lane_enable == '0)
      result = 64'b0;
    if (error)
      result = 64'b0;
  end

endmodule
