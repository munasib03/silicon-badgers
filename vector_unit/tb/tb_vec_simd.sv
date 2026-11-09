// Simple directed testbench for vec_simd (NUM_LANES = 4).
// There is no register load port yet, so every value is built with commands,
// starting from all-zero registers. Results are checked through the hierarchy.
//
// Build and run (from vector_unit/):
//   $ verilator --binary -j 0 -Wno-fatal -Wno-lint -Wno-style --top-module tb_vec_simd \
//     tb/tb_vec_simd.sv *.sv && ./obj_dir/Vtb_vec_simd
`timescale 1ns / 1ps

module tb_vec_simd;

  localparam int NUM_LANES = 4;
  localparam int CW = $clog2(NUM_LANES+1);

  // Families
  localparam logic [2:0] CONTROL = 3'd0, ALU = 3'd1, PRODUCT = 3'd2, REDUCE = 3'd3,
                         CAST = 3'd4, SPECIAL = 3'd5, DATA = 3'd6;
  // Operand codes
  localparam logic [4:0] V0 = 5'd0, V1 = 5'd1, V2 = 5'd2, V3 = 5'd3, V4 = 5'd4,
                         V5 = 5'd5, V6 = 5'd6, V7 = 5'd7;
  localparam logic [4:0] S0 = 5'd8, S1 = 5'd9, S2 = 5'd10, S3 = 5'd11, S4 = 5'd12;
  localparam logic [4:0] K_ZERO = 5'd16, K_NEG_ZERO = 5'd17, K_ONE = 5'd18, K_ONES = 5'd19;
  localparam logic [4:0] P1 = 5'd1, P2 = 5'd2;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  logic valid = 1'b0;
  logic [31:0] command = '0;
  logic strict = 1'b0;
  logic done, error, faulted, busy;
  logic [CW-1:0] sat_add, sat_sub, sat_product, sat_convert;

  int errors = 0;
  int checks = 0;
  int last_cycles;
  logic last_error;
  logic [CW-1:0] last_sat_add;

  always #5 clk = ~clk;

  vec_simd #(.NUM_LANES(NUM_LANES)) dut (
    .clk(clk),
    .rst_n(rst_n),
    .valid(valid),
    .command(command),
    .strict(strict),
    .done(done),
    .error(error),
    .faulted(faulted),
    .busy(busy),
    .saturated_add_count(sat_add),
    .saturated_sub_count(sat_sub),
    .saturated_product_count(sat_product),
    .saturated_convert_count(sat_convert)
  );

  // FAMILY[31:29] | SIG[28:22] | D[21:18] | A[17:13] | B[12:8] | C[7:3] | PM[2:0]
  function automatic logic [31:0] enc(input logic [2:0] family, input logic [6:0] sig,
                                      input logic [4:0] d, input logic [4:0] a,
                                      input logic [4:0] b, input logic [4:0] c,
                                      input logic [2:0] pm = 3'd0);
    return {family, sig, d[3:0], a, b, c, pm};
  endfunction

  // Hierarchical register access into register_file.
  function automatic logic [63:0] v(input int lane, input int r);
    return dut.register_file_inst.v_reg[r][lane];
  endfunction

  // Present a command, hold it until done or error, then let it commit.
  task automatic run(input logic [31:0] cmd, input string name, input bit expect_err = 1'b0);
    int cycles = 1;
    @(negedge clk);
    command = cmd;
    valid = 1'b1;
    #1;
    while (!(done || error)) begin
      @(negedge clk);
      #1;
      cycles++;
      if (cycles > 500) begin
        $display("FAIL %-28s timed out", name);
        errors++;
        valid = 1'b0;
        return;
      end
    end
    last_cycles = cycles;
    last_error = error;
    last_sat_add = sat_add;
    checks++;
    if (error != expect_err) begin
      $display("FAIL %-28s error=%0b expected %0b", name, error, expect_err);
      errors++;
    end
    @(posedge clk);          // commit edge
    #1;
    valid = 1'b0;
  endtask

  task automatic check_v(input int r, input logic [63:0] want [NUM_LANES], input string name);
    for (int l = 0; l < NUM_LANES; l++) begin
      checks++;
      if (v(l, r) !== want[l]) begin
        $display("FAIL %-28s V%0d[%0d] = %h expected %h", name, r, l, v(l, r), want[l]);
        errors++;
      end
    end
  endtask

  task automatic check_s(input int r, input logic [63:0] want, input string name);
    checks++;
    if (dut.register_file_inst.s_reg[r] !== want) begin
      $display("FAIL %-28s S%0d = %h expected %h", name, r, dut.register_file_inst.s_reg[r], want);
      errors++;
    end
  endtask

  task automatic check(input bit ok, input string name);
    checks++;
    if (!ok) begin
      $display("FAIL %s", name);
      errors++;
    end
  endtask

  initial begin
    repeat (2) @(posedge clk);
    rst_n = 1'b1;

    // ---------------------------------------------------------------- setup
    // VL starts at 0, so scalar ops would not run: SETVL 1 first.
    run(enc(CONTROL, 7'd0, 5'd0, K_ONE, 5'd0, 5'd0), "SETVL 1");
    check(dut.vl == 1, "VL == 1");
    run(enc(ALU, 7'd3, S0, K_ONE, K_ONE, 5'd0), "S0 = 1 + 1 (ADD.U32)");
    run(enc(ALU, 7'd3, S0, S0, S0, 5'd0), "S0 = S0 + S0");
    check_s(0, 64'd4, "S0 = 4");
    run(enc(CONTROL, 7'd0, 5'd0, S0, 5'd0, 5'd0), "SETVL S0");
    check(dut.vl == 4, "VL == 4");
    run(enc(ALU, 7'd11, S1, S0, K_ONE, 5'd0), "S1 = S0 - 1 (SUB.U32)");
    check_s(1, 64'd3, "S1 = 3");

    // ---------------------------------------------------------------- DATA / ALU
    run(enc(DATA, 7'd6, V0, 5'd0, 5'd0, 5'd0), "IOTA V0");
    check_v(0, '{0, 1, 2, 3}, "IOTA");

    run(enc(ALU, 7'd2, V1, V0, K_ONE, 5'd0), "V1 = V0 + 1 (ADD.I32)");
    check_v(1, '{1, 2, 3, 4}, "ADD.I32");

    run(enc(ALU, 7'd10, V6, S1, V0, 5'd0), "V6 = 3 - V0 (SUB.I32)");
    check_v(6, '{3, 2, 1, 0}, "SUB.I32 scalar-vector");

    run(enc(DATA, 7'd4, V2, V1, V6, 5'd0), "PERMUTE32 V2 = V1[V6]");
    check_v(2, '{4, 3, 2, 1}, "PERMUTE32 reverse");

    // EQ.I32 P1 = (V0 == 1), then ADD masked by P1.
    run(enc(ALU, 7'd18, P1, V0, K_ONE, 5'd0), "P1 = (V0 == 1) (EQ.I32)");
    check(dut.register_file_inst.p_reg[1] == 4'b0010, "P1 == 0010");
    run(enc(ALU, 7'd2, V2, V2, K_ONE, 5'd0, 3'd1), "V2 = V2 + 1 masked by P1");
    check_v(2, '{4, 4, 2, 1}, "masked ADD");

    // ---------------------------------------------------------------- CAST / PRODUCT
    run(enc(CAST, 7'd49, V3, V1, K_ZERO, 5'd0), "V3 = F32(V1) (CAST.I32.F32)");
    check_v(3, '{64'h3F80_0000, 64'h4000_0000, 64'h4040_0000, 64'h4080_0000}, "CAST.I32.F32");

    run(enc(PRODUCT, 7'd1, V4, V3, V3, K_NEG_ZERO), "V4 = V3*V3 + -0 (FMA.F32)");
    check_v(4, '{64'h3F80_0000, 64'h4080_0000, 64'h4110_0000, 64'h4180_0000}, "FMA.F32 squares");

    run(enc(CAST, 7'd14, V7, V4, K_ZERO, 5'd0), "V7 = I32(V4) (CAST.F32.I32)");
    check_v(7, '{1, 4, 9, 16}, "CAST.F32.I32");

    // ---------------------------------------------------------------- SPECIAL (multi-cycle)
    run(enc(SPECIAL, 7'd1, V5, V4, V3, 5'd0), "V5 = V4 / V3 (DIV.F32)");
    check_v(5, '{64'h3F80_0000, 64'h4000_0000, 64'h4040_0000, 64'h4080_0000}, "DIV.F32");
    check(last_cycles > 40, $sformatf("DIV.F32 is multi-cycle (%0d cycles)", last_cycles));

    run(enc(SPECIAL, 7'd4, V5, V7, S1, V6), "V5, V6 = V7 divmod 3 (DIVMOD.I32)");
    check_v(5, '{0, 1, 3, 5}, "DIVMOD quotient");
    check_v(6, '{1, 1, 0, 1}, "DIVMOD remainder");

    run(enc(SPECIAL, 7'd17, V5, V4, 5'd0, 5'd0), "V5 = RSQRT(V4) (RSQRT.F32)");
    check_v(5, '{64'h3F80_0000, 64'h3F00_0000, 64'h3EAA_AAAB, 64'h3E80_0000}, "RSQRT.F32");

    run(enc(SPECIAL, 7'd15, V5, K_ONE, 5'd0, 5'd0), "V5 = EXP(1.0) (EXP.F32)");
    check_v(5, '{64'h402D_F854, 64'h402D_F854, 64'h402D_F854, 64'h402D_F854}, "EXP.F32 e");

    // ---------------------------------------------------------------- REDUCE
    run(enc(REDUCE, 7'd2, S2, V7, 5'd0, 5'd0), "S2 = SUM(V7) (SUM.I32_I64)");
    check_s(2, 64'd30, "SUM = 1+4+9+16");
    run(enc(REDUCE, 7'd7, S3, V4, 5'd0, 5'd0), "S3 = MAX(V4) (MAX.F32)");
    check_s(3, 64'h4180_0000, "MAX = 16.0");

    // ---------------------------------------------------------------- saturation
    // ADD_SAT.I32: INT_MAX + 1 clamps (all-ones >> 1 = 0x7FFFFFFF via SHR.U32).
    run(enc(ALU, 7'd67, V7, K_ONES, K_ONE, 5'd0), "V7 = 0xFFFFFFFF >> 1 (SHR.U32)");
    run(enc(ALU, 7'd82, V7, V7, K_ONE, 5'd0), "V7 = V7 +sat 1 (ADD_SAT.I32)");
    check_v(7, '{64'h7FFF_FFFF, 64'h7FFF_FFFF, 64'h7FFF_FFFF, 64'h7FFF_FFFF}, "ADD_SAT clamps");
    check(last_sat_add == 3'd4, $sformatf("4 add saturations (got %0d)", last_sat_add));

    // ---------------------------------------------------------------- predicates / SELECT
    // V0 = {0,1,2,3}, V1 = {1,2,3,4}, P1 = 0010 (bit i = lane i).
    run(enc(ALU, 7'd26, P2, V0, K_ONE, 5'd0), "P2 = (V0 < 1) (LT.I32)");
    check(dut.register_file_inst.p_reg[2] == 4'b0001, "P2 == 0001");
    run(enc(ALU, 7'd26, P2, V0, S1, 5'd0), "P2 = (V0 < 3) (LT.I32)");
    check(dut.register_file_inst.p_reg[2] == 4'b0111, "P2 == 0111");
    run(enc(CONTROL, 7'd1, 5'd3, P1, P2, 5'd0), "PAND P3 = P1 & P2");
    check(dut.register_file_inst.p_reg[3] == 4'b0010, "P3 == 0010");
    run(enc(CONTROL, 7'd2, 5'd4, P2, 5'd0, 5'd0), "PNOT P4 = ~P2");
    check(dut.register_file_inst.p_reg[4] == 4'b1000, "P4 == 1000");
    // PNOT only updates lanes below VL.
    run(enc(CONTROL, 7'd0, 5'd0, K_ONE, 5'd0, 5'd0), "SETVL 1");
    run(enc(CONTROL, 7'd2, 5'd4, 5'd4, 5'd0, 5'd0), "PNOT P4 = ~P4 (VL = 1)");
    check(dut.register_file_inst.p_reg[4] == 4'b1001, "P4 == 1001 (lanes >= VL kept)");
    run(enc(CONTROL, 7'd0, 5'd0, S0, 5'd0, 5'd0), "SETVL S0 (4)");

    run(enc(ALU, 7'd34, V5, V0, V1, P2), "V5 = P2 ? V0 : V1 (SELECT.I32)");
    check_v(5, '{0, 1, 2, 4}, "SELECT.I32");
    // Scalar SELECT reads C at the lowest enabled lane.
    run(enc(ALU, 7'd34, S4, S0, S1, P1, 3'd4), "S4 = P1 ? S0 : S1, PM = P4");
    check_s(4, 64'd3, "scalar SELECT lane 0 (P1[0] = 0)");
    run(enc(ALU, 7'd34, S4, S0, S1, P1, 3'd3), "S4 = P1 ? S0 : S1, PM = P3");
    check_s(4, 64'd4, "scalar SELECT lane 1 (P1[1] = 1)");

    // Scalar DIVMOD writes both S[D] and S[C].
    run(enc(SPECIAL, 7'd4, S4, S2, S0, 5'd13), "S4, S5 = S2 divmod S0 (30 / 4)");
    check_s(4, 64'd7, "scalar DIVMOD quotient");
    check_s(5, 64'd2, "scalar DIVMOD remainder");

    // ---------------------------------------------------------------- faults
    run(32'hE000_0000, "family 111 is illegal", 1'b1);
    check(faulted, "fault is sticky");
    run(enc(ALU, 7'd2, V1, V0, K_ONE, 5'd0), "command after fault", 1'b1);

    // Reset clears the fault; a zero divisor faults without writing.
    rst_n = 1'b0;
    @(posedge clk);
    rst_n = 1'b1;
    run(enc(CONTROL, 7'd0, 5'd0, K_ONE, 5'd0, 5'd0), "SETVL 1 after reset");
    run(enc(DATA, 7'd6, V0, 5'd0, 5'd0, 5'd0), "IOTA V0 (lane 0 = 0)");
    run(enc(ALU, 7'd2, V1, K_ONE, K_ONE, 5'd0), "V1 = 2");
    run(enc(SPECIAL, 7'd4, V2, V1, V0, V3), "DIVMOD by zero", 1'b1);
    check(v(0, 2) == 0 && v(0, 3) == 0, "DIVMOD by zero wrote nothing");

    // SETVL above the lane count faults and leaves VL unchanged.
    rst_n = 1'b0;
    @(posedge clk);
    rst_n = 1'b1;
    run(enc(CONTROL, 7'd0, 5'd0, K_ONE, 5'd0, 5'd0), "SETVL 1 after reset");
    run(enc(ALU, 7'd3, S0, K_ONE, K_ONE, 5'd0), "S0 = 2");
    run(enc(ALU, 7'd3, S0, S0, S0, 5'd0), "S0 = 4");
    run(enc(ALU, 7'd3, S0, S0, K_ONE, 5'd0), "S0 = 5");
    run(enc(CONTROL, 7'd0, 5'd0, S0, 5'd0, 5'd0), "SETVL 5 > NUM_LANES", 1'b1);
    check(dut.vl == 1, "VL unchanged after SETVL fault");

    $display("");
    if (errors == 0)
      $display("PASS: %0d checks", checks);
    else
      $display("FAILED: %0d of %0d checks", errors, checks);
    $finish;
  end

endmodule
