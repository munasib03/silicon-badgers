// Central register file: V0-V7 (one DATA_WIDTH element per lane), S0-S7, P0-P7
// (one bit per lane, P0 always true) and VL.
//
// Reads are combinational. Writes happen on the clock edge, so a command can read
// and write the same register (all inputs are read before any output is written).
// There is no forwarding: the unit is single-cycle, and forwarding a result back
// into the operands that produce it would form a combinational loop.
//
// Constants (operand codes 16-19) depend on the operand type, so the lanes make
// them; these read ports return 0 for those codes.
module register_file #(
    parameter int DATA_WIDTH = 64,
    parameter int NUM_LANES  = 8
) (
    input  logic                                        clk,
    input  logic                                        rst_n,

    input  logic [4:0]                                  raddr_a,  // operand code of A: 0-7 V, 8-15 S
    input  logic [4:0]                                  raddr_b,  // operand code of B
    input  logic [4:0]                                  raddr_c,  // operand code of C
    input  logic [2:0]                                  pm_sel,   // execution mask predicate P0..P7

    output logic [NUM_LANES-1:0][DATA_WIDTH-1:0]        rdata_a,  // V: per lane, S: broadcast
    output logic [NUM_LANES-1:0][DATA_WIDTH-1:0]        rdata_b,
    output logic [NUM_LANES-1:0][DATA_WIDTH-1:0]        rdata_c,
    output logic [NUM_LANES-1:0]                        rdata_pm, // P[pm_sel]
    output logic [NUM_LANES-1:0]                        rdata_p_a, // P[A] (PAND, PNOT)
    output logic [NUM_LANES-1:0]                        rdata_p_b, // P[B] (PAND)
    output logic [NUM_LANES-1:0]                        rdata_p_c, // P[C] (SELECT)
    output logic [31:0]                                 vl_out,

    // Lane mask shared by both write ports (V and P writes only).
    input  logic [NUM_LANES-1:0]                        lane_mask,

    // Write port D
    input  logic [3:0]                                  waddr_d,
    input  logic                                        wren_v_d,
    input  logic                                        wren_s_d,
    input  logic                                        wren_p_d,
    input  logic [1:0]                                  wsize_d,  // 0: 16 bits, 1: 32 bits, else 64
    input  logic [NUM_LANES-1:0][DATA_WIDTH-1:0]        wdata_d,  // S writes take lane 0
    input  logic [NUM_LANES-1:0]                        wdata_p_d,

    // Write port C (DIVMOD remainder)
    input  logic [3:0]                                  waddr_c,
    input  logic                                        wren_v_c,
    input  logic                                        wren_s_c,
    input  logic                                        wren_p_c,
    input  logic [1:0]                                  wsize_c,
    input  logic [NUM_LANES-1:0][DATA_WIDTH-1:0]        wdata_c,
    input  logic [NUM_LANES-1:0]                        wdata_p_c,

    // VL is stored as given; the top level faults a value above NUM_LANES.
    input  logic                                        setvl_en,
    input  logic [31:0]                                 setvl_val
);

    logic [NUM_LANES-1:0][DATA_WIDTH-1:0] v_reg [0:7];
    logic [DATA_WIDTH-1:0]                s_reg [0:7];
    logic [NUM_LANES-1:0]                 p_reg [0:7];
    logic [31:0]                          vl_reg;

    function automatic [DATA_WIDTH-1:0] format_wdata(
        input logic [DATA_WIDTH-1:0] data,
        input logic [1:0]            size
    );
        case (size)
            2'b00:   format_wdata = {{(DATA_WIDTH-16){1'b0}}, data[15:0]};
            2'b01:   format_wdata = {{(DATA_WIDTH-32){1'b0}}, data[31:0]};
            default: format_wdata = data;
        endcase
    endfunction

    // Codes 0-7 read a V register, 8-15 broadcast an S register, anything else is 0.
    function automatic logic [NUM_LANES-1:0][DATA_WIDTH-1:0] read_operand(input logic [4:0] addr);
        if (addr <= 5'd7)
            read_operand = v_reg[addr[2:0]];
        else if (addr <= 5'd15)
            for (int i = 0; i < NUM_LANES; i++) read_operand[i] = s_reg[addr[2:0]];
        else
            read_operand = '0;
    endfunction

    // P0 always reads as true; codes above 7 read as 0.
    function automatic logic [NUM_LANES-1:0] read_predicate(input logic [4:0] addr);
        if (addr > 5'd7)
            read_predicate = '0;
        else if (addr[2:0] == 3'd0)
            read_predicate = '1;
        else
            read_predicate = p_reg[addr[2:0]];
    endfunction

    always_comb begin
        rdata_a   = read_operand(raddr_a);
        rdata_b   = read_operand(raddr_b);
        rdata_c   = read_operand(raddr_c);
        rdata_pm  = read_predicate({2'b00, pm_sel});
        rdata_p_a = read_predicate(raddr_a);
        rdata_p_b = read_predicate(raddr_b);
        rdata_p_c = read_predicate(raddr_c);
    end

    assign vl_out = vl_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int r = 0; r < 8; r++) begin
                v_reg[r] <= '0;
                s_reg[r] <= '0;
                p_reg[r] <= '0;
            end
            p_reg[0] <= '1;
            vl_reg   <= '0;
        end else begin
            p_reg[0] <= '1;

            if (wren_v_d) begin
                for (int lane = 0; lane < NUM_LANES; lane++) begin
                    if (lane_mask[lane])
                        v_reg[waddr_d[2:0]][lane] <= format_wdata(wdata_d[lane], wsize_d);
                end
            end

            if (wren_s_d)
                s_reg[waddr_d[2:0]] <= format_wdata(wdata_d[0], wsize_d);

            if (wren_p_d && (waddr_d[2:0] != 3'd0)) begin
                for (int lane = 0; lane < NUM_LANES; lane++) begin
                    if (lane_mask[lane])
                        p_reg[waddr_d[2:0]][lane] <= wdata_p_d[lane];
                end
            end

            // The decoder guarantees C != D when both ports write.
            if (wren_v_c) begin
                for (int lane = 0; lane < NUM_LANES; lane++) begin
                    if (lane_mask[lane])
                        v_reg[waddr_c[2:0]][lane] <= format_wdata(wdata_c[lane], wsize_c);
                end
            end

            if (wren_s_c)
                s_reg[waddr_c[2:0]] <= format_wdata(wdata_c[0], wsize_c);

            if (wren_p_c && (waddr_c[2:0] != 3'd0)) begin
                for (int lane = 0; lane < NUM_LANES; lane++) begin
                    if (lane_mask[lane])
                        p_reg[waddr_c[2:0]][lane] <= wdata_p_c[lane];
                end
            end

            if (setvl_en)
                vl_reg <= setvl_val;
        end
    end

endmodule
