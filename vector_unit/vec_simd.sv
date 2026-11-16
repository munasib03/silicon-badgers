// SIMD unit top: command decoder, central S/P registers and VL, N lanes, and the
// cross-lane units (REDUCE, PERMUTE routing).
//
// Handshake: the issuer presents command with valid and holds both until done or
// error is high (sampled at a clock edge).
// - Most commands take one cycle: they are checked and committed on the next edge.
// - DIV, RCP, DIVMOD, DIV_RNE and RSQRT use sequential units. The first cycle starts
//   them and raises busy; done (or error) comes in the cycle they finish:
//   32/64 cycles for 32/64-bit integer division, 50 for FP DIV/RCP, 28 for RSQRT.
//   An illegal command reports error immediately without starting.
// Any fault writes nothing and blocks the sequence until reset.
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
  output logic        busy,              // a multi-cycle command is running
  // Saturations recorded this cycle, per stage (also counted in strict mode).
  output logic [$clog2(NUM_LANES+1)-1:0] saturated_add_count,
  output logic [$clog2(NUM_LANES+1)-1:0] saturated_sub_count,
  output logic [$clog2(NUM_LANES+1)-1:0] saturated_product_count,
  output logic [$clog2(NUM_LANES+1)-1:0] saturated_convert_count
);

  localparam int COUNT_WIDTH = $clog2(NUM_LANES+1);

  localparam logic [6:0] SETVL = 7'd0;
  localparam logic [6:0] PAND  = 7'd1;
  localparam logic [6:0] PNOT  = 7'd2;

  localparam logic [6:0] PERMUTE16 = 7'd3;
  localparam logic [6:0] PERMUTE64 = 7'd5;

  localparam logic [4:0] K_ZERO = 5'd16;
  localparam logic [4:0] K_ONE  = 5'd18;
  localparam logic [4:0] K_ONES = 5'd19;

  logic [2:0] family;
  logic [6:0] sig;
  logic [3:0] d_code;
  logic [4:0] a_code, b_code, c_code;
  logic [3:0] a_type, b_type, c_type;
  logic [2:0] pm;
  logic is_control, is_alu, is_product, is_reduce, is_cast, is_special, is_data;
  logic predicate_dest, scalar_dest, second_dest, multi_cycle, illegal;
  logic running;                        // multi-cycle units are working
  logic unit_start;                     // starts the lanes' sequential units
  logic exec_ready;                     // results of this command are available
  logic lane_command;                   // executed by the lanes
  logic is_permute;

  logic [31:0]          vl;

  // Register file interface signals
  logic [NUM_LANES-1:0][63:0] rf_rdata_a;
  logic [NUM_LANES-1:0][63:0] rf_rdata_b;
  logic [NUM_LANES-1:0][63:0] rf_rdata_c;
  logic [NUM_LANES-1:0]       rf_rdata_pm;
  logic [NUM_LANES-1:0]       rf_rdata_p_a;
  logic [NUM_LANES-1:0]       rf_rdata_p_b;
  logic [NUM_LANES-1:0]       rf_rdata_p_c;
  logic [31:0]                rf_vl_out;

  logic [3:0]                 rf_waddr_d;
  logic                       rf_wren_v_d;
  logic                       rf_wren_s_d;
  logic                       rf_wren_p_d;
  logic [1:0]                 rf_wsize_d;
  logic [NUM_LANES-1:0]       rf_lane_mask_d;
  logic [NUM_LANES-1:0][63:0] rf_wdata_d;
  logic [NUM_LANES-1:0]       rf_wdata_p_d;

  logic [3:0]                 rf_waddr_c;
  logic                       rf_wren_v_c;
  logic                       rf_wren_s_c;
  logic [1:0]                 rf_wsize_c;
  logic [NUM_LANES-1:0][63:0] rf_wdata_c;

  logic                       rf_setvl_en;
  logic [31:0]                rf_setvl_val;
  logic                       rf_divmod_conflict;

  logic [NUM_LANES-1:0] below_vl;
  logic [NUM_LANES-1:0] lane_enable, lane_error, lane_finished;
  logic [NUM_LANES-1:0] lane_saturated_add, lane_saturated_sub;
  logic [NUM_LANES-1:0] lane_saturated_product, lane_saturated_convert;
  logic [NUM_LANES-1:0] lane_predicate_result;
  logic [63:0]          lane_result  [NUM_LANES];
  logic [63:0]          lane_result2 [NUM_LANES];
  logic [63:0]          lane_a_value [NUM_LANES];
  logic [63:0]          lane_b_value [NUM_LANES];
  logic [63:0]          permute_data [NUM_LANES];
  logic [NUM_LANES-1:0] permute_index_bad;

  logic        any_enabled;
  logic [63:0] scalar_result, scalar_result2;
  logic [31:0] setvl_value;
  logic        setvl_error, permute_error;
  logic        reduce_error;
  logic [63:0] reduce_result;
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
    .a_type(a_type),
    .b_type(b_type),
    .c_type(c_type),
    .is_control(is_control),
    .is_alu(is_alu),
    .is_product(is_product),
    .is_reduce(is_reduce),
    .is_cast(is_cast),
    .is_special(is_special),
    .is_data(is_data),
    .predicate_dest(predicate_dest),
    .scalar_dest(scalar_dest),
    .second_dest(second_dest),
    .multi_cycle(multi_cycle),
    .illegal(illegal)
  );

  assign is_permute = is_data && (sig >= PERMUTE16) && (sig <= PERMUTE64);

  assign vl = rf_vl_out;

  register_file #(
    .DATA_WIDTH(64),
    .NUM_LANES(NUM_LANES)
  ) register_file_inst (
    .clk(clk),
    .rst_n(rst_n),
    .raddr_a(a_code),
    .raddr_b(b_code),
    .raddr_c(c_code),
    .pm_sel(pm),
    .rdata_a(rf_rdata_a),
    .rdata_b(rf_rdata_b),
    .rdata_c(rf_rdata_c),
    .rdata_pm(rf_rdata_pm),
    .rdata_p_a(rf_rdata_p_a),
    .rdata_p_b(rf_rdata_p_b),
    .rdata_p_c(rf_rdata_p_c),
    .vl_out(rf_vl_out),
    .waddr_d(rf_waddr_d),
    .wren_v_d(rf_wren_v_d),
    .wren_s_d(rf_wren_s_d),
    .wren_p_d(rf_wren_p_d),
    .wsize_d(rf_wsize_d),
    .lane_mask_d(rf_lane_mask_d),
    .wdata_d(rf_wdata_d),
    .wdata_p_d(rf_wdata_p_d),
    .waddr_c(rf_waddr_c),
    .wren_v_c(rf_wren_v_c),
    .wren_s_c(rf_wren_s_c),
    .wsize_c(rf_wsize_c),
    .wdata_c(rf_wdata_c),
    .setvl_en(rf_setvl_en),
    .setvl_val(rf_setvl_val),
    .divmod_conflict(rf_divmod_conflict)
  );

  genvar i;
  generate
    for (i = 0; i < NUM_LANES; i++) begin : lanes
      assign below_vl[i] = (i < vl);

      // PERMUTE: this lane reads A from the lane its B selects. Every lane's A is
      // read before any write, so in-place permutation works.
      assign permute_index_bad[i] = (lane_b_value[i][31:0] >= NUM_LANES[31:0]);
      assign permute_data[i] = permute_index_bad[i] ? 64'b0 :
                               lane_a_value[lane_b_value[i][31:0]];

      vec_lane #(.LANE_INDEX(i)) vec_lane_inst (
        .clk(clk),
        .rst_n(rst_n),
        .family(family),
        .sig(sig),
        .d_code(d_code),
        .a_code(a_code),
        .b_code(b_code),
        .c_code(c_code),
        .a_type(a_type),
        .b_type(b_type),
        .c_type(c_type),
        .pm(pm),
        .vl(vl),
        .strict(strict),
        .commit(commit && lane_command),
        .start(unit_start),
        .rdata_a(rf_rdata_a[i]),
        .rdata_b(rf_rdata_b[i]),
        .rdata_c(rf_rdata_c[i]),
        .rdata_pm(rf_rdata_pm[i]),
        .rdata_p_c(rf_rdata_p_c[i]),
        .permute_data(permute_data[i]),
        .enable(lane_enable[i]),
        .finished(lane_finished[i]),
        .error(lane_error[i]),
        .saturated_add(lane_saturated_add[i]),
        .saturated_sub(lane_saturated_sub[i]),
        .saturated_product(lane_saturated_product[i]),
        .saturated_convert(lane_saturated_convert[i]),
        .predicate_result(lane_predicate_result[i]),
        .a_value(lane_a_value[i]),
        .b_value(lane_b_value[i]),
        .result(lane_result[i]),
        .result2(lane_result2[i])
      );
    end
  endgenerate

  // REDUCE across lanes into one scalar.
  vec_reduce #(.NUM_LANES(NUM_LANES)) vec_reduce_inst (
    .lane_value(lane_a_value),
    .lane_enable(lane_enable),
    .vl(vl),
    .sig(sig),
    .error(reduce_error),
    .result(reduce_result)
  );

  // Scalar D executes once: every enabled lane sees the same scalar inputs, so take
  // the lowest enabled lane. This also gives scalar SELECT its lowest-lane C bit.
  always_comb begin
    any_enabled = |lane_enable;
    scalar_result = 64'b0;
    scalar_result2 = 64'b0;
    for (int k = NUM_LANES - 1; k >= 0; k--) begin
      if (lane_enable[k]) begin
        scalar_result = lane_result[k];
        scalar_result2 = lane_result2[k];
      end
    end
  end

  // SETVL reads a U32 scalar or constant and must not exceed the lane count.
  always_comb begin
    case (a_code)
      K_ZERO:  setvl_value = 32'd0;
      K_ONE:   setvl_value = 32'd1;
      K_ONES:  setvl_value = 32'hFFFF_FFFF;
      default: setvl_value = rf_rdata_a[0][31:0];
    endcase
    setvl_error = is_control && (sig == SETVL) && (setvl_value > NUM_LANES[31:0]);
  end

  // An enabled lane's index must be below the physical lane count; the source
  // lane itself may be disabled.
  assign permute_error = is_permute && |(permute_index_bad & lane_enable);

  assign lane_command = is_alu || is_product || is_cast || is_special || is_data;

  // Multi-cycle commands: start the units in the first cycle, then wait. All lanes
  // run the same operation, so they finish together.
  assign unit_start = valid && !faulted && !illegal && multi_cycle && !running;
  assign exec_ready = !multi_cycle || (running && (&lane_finished));
  assign busy       = running;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      running <= 1'b0;
    else if (unit_start)
      running <= 1'b1;
    else if (running && (&lane_finished))
      running <= 1'b0;
  end

  // Check every enabled lane before committing any output.
  assign fault  = valid && !faulted &&
                  (illegal ||
                   (exec_ready && (setvl_error || permute_error ||
                                   (lane_command && |lane_error) ||
                                   (is_reduce && reduce_error))));
  assign commit = valid && !faulted && !fault && exec_ready;
  assign done   = commit;
  assign error  = valid && (faulted || fault);

  // Saturation is recorded in strict mode too. Scalar D counts once.
  always_comb begin
    saturated_add_count = '0;
    saturated_sub_count = '0;
    saturated_product_count = '0;
    saturated_convert_count = '0;
    if (valid && !faulted && !illegal && exec_ready && lane_command) begin
      if (scalar_dest) begin
        saturated_add_count     = COUNT_WIDTH'(|lane_saturated_add);
        saturated_sub_count     = COUNT_WIDTH'(|lane_saturated_sub);
        saturated_product_count = COUNT_WIDTH'(|lane_saturated_product);
        saturated_convert_count = COUNT_WIDTH'(|lane_saturated_convert);
      end else begin
        for (int k = 0; k < NUM_LANES; k++) begin
          saturated_add_count     = saturated_add_count + COUNT_WIDTH'(lane_saturated_add[k]);
          saturated_sub_count     = saturated_sub_count + COUNT_WIDTH'(lane_saturated_sub[k]);
          saturated_product_count = saturated_product_count + COUNT_WIDTH'(lane_saturated_product[k]);
          saturated_convert_count = saturated_convert_count + COUNT_WIDTH'(lane_saturated_convert[k]);
        end
      end
    end
  end

  // Central register file writeback and control signals
  assign rf_waddr_d = d_code;
  assign rf_waddr_c = c_code[3:0];
  assign rf_wsize_d = 2'b11;
  assign rf_wsize_c = 2'b11;

  assign rf_setvl_en  = commit && is_control && (sig == SETVL);
  assign rf_setvl_val = setvl_value;

  assign rf_lane_mask_d = is_control ? below_vl : lane_enable;

  assign rf_wren_v_d = commit && lane_command && !predicate_dest && !scalar_dest;
  assign rf_wren_s_d = commit && (is_reduce || (lane_command && scalar_dest && any_enabled));
  assign rf_wren_p_d = commit && ((is_control && ((sig == PAND) || (sig == PNOT))) ||
                                  (lane_command && predicate_dest));

  assign rf_wren_v_c = commit && lane_command && second_dest && !c_code[3];
  assign rf_wren_s_c = commit && lane_command && second_dest && c_code[3] && any_enabled;

  always_comb begin
    for (int k = 0; k < NUM_LANES; k++) begin
      rf_wdata_d[k] = lane_result[k];
    end
    if (is_reduce) begin
      rf_wdata_d[0] = reduce_result;
    end else if (scalar_dest) begin
      rf_wdata_d[0] = scalar_result;
    end
  end

  always_comb begin
    for (int k = 0; k < NUM_LANES; k++) begin
      rf_wdata_c[k] = lane_result2[k];
    end
    if (scalar_dest) begin
      rf_wdata_c[0] = scalar_result2;
    end
  end

  always_comb begin
    if (sig == PNOT)
      rf_wdata_p_d = ~rf_rdata_p_a;
    else if (sig == PAND)
      rf_wdata_p_d = rf_rdata_p_a & rf_rdata_p_b;
    else
      rf_wdata_p_d = lane_predicate_result;
  end

  // Sticky fault flag: any fault invalidates sequence until reset.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      faulted <= 1'b0;
    end else if (fault) begin
      faulted <= 1'b1;
    end
  end

endmodule
