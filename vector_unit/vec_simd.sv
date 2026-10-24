// SIMD unit top: command decoder, central S/P registers and VL, and N lanes.
// Single cycle: a command presented with valid is checked and committed on the
// next clock edge. Any fault writes nothing and blocks the sequence until reset.
module vec_simd #(
  parameter int NUM_LANES = 8
) (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        valid,             // command presented this cycle
  input  logic [31:0] command,
  input  logic        strict,            // set once per compute sequence
  output logic        done,              // command committed this cycle
  output logic        error,             // command faulted (or sequence already faulted)
  output logic        faulted,           // sticky: sequence invalid until reset
  output logic [$clog2(NUM_LANES+1)-1:0] saturated_add_count,
  output logic [$clog2(NUM_LANES+1)-1:0] saturated_sub_count
);

  localparam int COUNT_WIDTH = $clog2(NUM_LANES+1);

  localparam logic [6:0] SETVL = 7'd0;
  localparam logic [6:0] PAND  = 7'd1;
  localparam logic [6:0] PNOT  = 7'd2;

  localparam logic [4:0] K_ZERO = 5'd16;
  localparam logic [4:0] K_ONE  = 5'd18;
  localparam logic [4:0] K_ONES = 5'd19;

  logic [2:0] family;
  logic [6:0] sig;
  logic [3:0] d_code;
  logic [4:0] a_code, b_code, c_code;
  logic [2:0] pm;
  logic is_control, is_alu, predicate_dest, scalar_dest, illegal;

  logic [63:0]          s_reg [8];
  logic [NUM_LANES-1:0] p_reg [8];
  logic [31:0]          vl;

  logic [NUM_LANES-1:0] below_vl;
  logic [NUM_LANES-1:0] lane_enable, lane_error;
  logic [NUM_LANES-1:0] lane_saturated_add, lane_saturated_sub;
  logic [NUM_LANES-1:0] lane_predicate_result;
  logic [63:0]          lane_result [NUM_LANES];

  logic        any_enabled;
  logic [63:0] scalar_result;
  logic [31:0] setvl_value;
  logic        setvl_error;
  logic        fault, commit;

  vec_decoder vec_decoder_inst (
    .command(command),
    .family(family),
    .sig(sig),
    .d_code(d_code),
    .a_code(a_code),
    .b_code(b_code),
    .c_code(c_code),
    .pm(pm),
    .is_control(is_control),
    .is_alu(is_alu),
    .predicate_dest(predicate_dest),
    .scalar_dest(scalar_dest),
    .illegal(illegal)
  );

  genvar i, j;
  generate
    for (i = 0; i < NUM_LANES; i++) begin : lanes
      logic [7:0] p_bits;

      for (j = 0; j < 8; j++) begin : p_slice
        assign p_bits[j] = p_reg[j][i];
      end

      assign below_vl[i] = (i < vl);

      vec_lane #(.LANE_INDEX(i)) vec_lane_inst (
        .clk(clk),
        .rst_n(rst_n),
        .sig(sig),
        .d_code(d_code),
        .a_code(a_code),
        .b_code(b_code),
        .c_code(c_code),
        .pm(pm),
        .vl(vl),
        .strict(strict),
        .commit(commit && is_alu),
        .s_reg(s_reg),
        .p_bits(p_bits),
        .enable(lane_enable[i]),
        .error(lane_error[i]),
        .saturated_add(lane_saturated_add[i]),
        .saturated_sub(lane_saturated_sub[i]),
        .predicate_result(lane_predicate_result[i]),
        .result(lane_result[i])
      );
    end
  endgenerate

  // Scalar D executes once: every enabled lane sees the same scalar inputs, so take
  // the lowest enabled lane. This also gives scalar SELECT its lowest-lane C bit.
  always_comb begin
    any_enabled = |lane_enable;
    scalar_result = 64'b0;
    for (int k = NUM_LANES - 1; k >= 0; k--) begin
      if (lane_enable[k])
        scalar_result = lane_result[k];
    end
  end

  // SETVL reads a U32 scalar or constant and must not exceed the lane count.
  always_comb begin
    case (a_code)
      K_ZERO:  setvl_value = 32'd0;
      K_ONE:   setvl_value = 32'd1;
      K_ONES:  setvl_value = 32'hFFFF_FFFF;
      default: setvl_value = s_reg[a_code[2:0]][31:0];
    endcase
    setvl_error = is_control && (sig == SETVL) && (setvl_value > NUM_LANES[31:0]);
  end

  // Check every enabled lane before committing any output.
  assign fault  = valid && !faulted && (illegal || setvl_error || (is_alu && |lane_error));
  assign commit = valid && !faulted && !fault;
  assign done   = commit;
  assign error  = valid && (faulted || fault);

  // Saturation is recorded in strict mode too. Scalar D counts once.
  always_comb begin
    saturated_add_count = '0;
    saturated_sub_count = '0;
    if (valid && !faulted && !illegal && is_alu) begin
      if (scalar_dest) begin
        saturated_add_count = (|lane_saturated_add) ? 1 : 0;
        saturated_sub_count = (|lane_saturated_sub) ? 1 : 0;
      end else begin
        for (int k = 0; k < NUM_LANES; k++) begin
          saturated_add_count = saturated_add_count + COUNT_WIDTH'(lane_saturated_add[k]);
          saturated_sub_count = saturated_sub_count + COUNT_WIDTH'(lane_saturated_sub[k]);
        end
      end
    end
  end

  // Central state. V writes happen inside each lane. Registers start at zero;
  // P0 is always true and is never written (the decoder rejects D = P0).
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int k = 0; k < 8; k++) begin
        s_reg[k] <= 64'b0;
        p_reg[k] <= '0;
      end
      p_reg[0] <= '1;
      vl <= 32'd0;
      faulted <= 1'b0;
    end else begin
      if (fault)
        faulted <= 1'b1;

      if (commit && is_control) begin
        case (sig)
          SETVL: vl <= setvl_value;

          // Update lanes below VL; higher bits keep their values.
          PAND: p_reg[d_code[2:0]] <= (p_reg[a_code[2:0]] & p_reg[b_code[2:0]] & below_vl) |
                                      (p_reg[d_code[2:0]] & ~below_vl);

          PNOT: p_reg[d_code[2:0]] <= (~p_reg[a_code[2:0]] & below_vl) |
                                      (p_reg[d_code[2:0]] & ~below_vl);

          default: ;
        endcase
      end

      if (commit && is_alu) begin
        // EQ / LT: enabled lanes write their bit; disabled lanes keep theirs.
        if (predicate_dest)
          p_reg[d_code[2:0]] <= (lane_predicate_result & lane_enable) |
                                (p_reg[d_code[2:0]] & ~lane_enable);
        // Scalar D: no enabled lane means no write.
        else if (scalar_dest && any_enabled)
          s_reg[d_code[2:0]] <= scalar_result;
      end
    end
  end

endmodule
