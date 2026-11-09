`timescale 1ns / 1ps

module register_file #(
    parameter int DATA_WIDTH = 64,
    parameter int NUM_LANES  = 8,
) (
    input  logic                                        clk,
    input  logic                                        rst_n,

    input  logic [4:0]                                  raddr_a,  //Address of A 5 bits
    input  logic [4:0]                                  raddr_b,  //Address of B 5 bits
    input  logic [4:0]                                  raddr_c,  //Address of C 5 bits 
    input  logic [2:0]                                  pm_sel,   //Which predicate bit to be selected from P0 to P7

    output logic [NUM_LANES-1:0][DATA_WIDTH-1:0]        rdata_a,  //
    output logic [NUM_LANES-1:0][DATA_WIDTH-1:0]        rdata_b,
    output logic [NUM_LANES-1:0][DATA_WIDTH-1:0]        rdata_c,
    output logic [NUM_LANES-1:0]                        rdata_pm,
    output logic [NUM_LANES-1:0]                        rdata_p_a,
    output logic [NUM_LANES-1:0]                        rdata_p_b,
    output logic [NUM_LANES-1:0]                        rdata_p_c,
    output logic [31:0]                         vl_out,

    input  logic [3:0]                                  waddr_d,
    input  logic                                        wren_v_d,
    input  logic                                        wren_s_d,
    input  logic                                        wren_p_d,
    input  logic [1:0]                                  wsize_d,
    input  logic [NUM_LANES-1:0]                        lane_mask_d,
    input  logic [NUM_LANES-1:0][DATA_WIDTH-1:0]        wdata_d,
    input  logic [NUM_LANES-1:0]                        wdata_p_d,

    input  logic [3:0]                                  waddr_c,
    input  logic                                        wren_v_c,
    input  logic                                        wren_s_c,
    input  logic                                        wren_p_c,
    input  logic [1:0]                                  wsize_c,
    input  logic [NUM_LANES-1:0][DATA_WIDTH-1:0]        wdata_c,
    input  logic [NUM_LANES-1:0]                        wdata_p_c,

    input  logic                                        setvl_en,
    input  logic [31:0]                                 setvl_val,

    output logic                                        divmod_conflict
);

    logic [NUM_LANES-1:0][DATA_WIDTH-1:0] v_reg [0:7];
    logic [DATA_WIDTH-1:0]                s_reg [0:7];
    logic [NUM_LANES-1:0]                 p_reg [0:7];
    logic [31:0]                  vl_reg;


    //defining the constants
    localparam logic [DATA_WIDTH-1:0] CONST_ZERO     = '0;
    localparam logic [DATA_WIDTH-1:0] CONST_ALL_ONES = '1;
    localparam logic [DATA_WIDTH-1:0] CONST_ONE      = {{(DATA_WIDTH-1){1'b0}}, 1'b1};
    localparam logic [DATA_WIDTH-1:0] CONST_NEG_ZERO = {1'b1, {(DATA_WIDTH-1){1'b0}}};

    logic [DATA_WIDTH-1:0] bval_a, bval_b, bval_c;

    always_comb begin
        divmod_conflict = 1'b0;
        if ((wren_v_d && wren_v_c) && (waddr_d[2:0] == waddr_c[2:0]))
            divmod_conflict = 1'b1;
        if ((wren_s_d && wren_s_c) && (waddr_d[2:0] == waddr_c[2:0]))
            divmod_conflict = 1'b1;
        if ((wren_p_d && wren_p_c) && (waddr_d[2:0] == waddr_c[2:0]))
            divmod_conflict = 1'b1;
    end

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

    always_comb begin
        case (raddr_a)
            5'd8,  5'd9,  5'd10, 5'd11,
            5'd12, 5'd13, 5'd14, 5'd15: bval_a = s_reg[raddr_a[2:0]];
            5'd16:                      bval_a = CONST_ZERO;
            5'd17:                      bval_a = CONST_NEG_ZERO;
            5'd18:                      bval_a = CONST_ONE;
            5'd19:                      bval_a = CONST_ALL_ONES;
            default:                    bval_a = '0;
        endcase

        case (raddr_b)
            5'd8,  5'd9,  5'd10, 5'd11,
            5'd12, 5'd13, 5'd14, 5'd15: bval_b = s_reg[raddr_b[2:0]];
            5'd16:                      bval_b = CONST_ZERO;
            5'd17:                      bval_b = CONST_NEG_ZERO;
            5'd18:                      bval_b = CONST_ONE;
            5'd19:                      bval_b = CONST_ALL_ONES;
            default:                    bval_b = '0;
        endcase

        case (raddr_c)
            5'd8,  5'd9,  5'd10, 5'd11,
            5'd12, 5'd13, 5'd14, 5'd15: bval_c = s_reg[raddr_c[2:0]];
            5'd16:                      bval_c = CONST_ZERO;
            5'd17:                      bval_c = CONST_NEG_ZERO;
            5'd18:                      bval_c = CONST_ONE;
            5'd19:                      bval_c = CONST_ALL_ONES;
            default:                    bval_c = '0;
        endcase

        if (raddr_a <= 5'd7)
            rdata_a = v_reg[raddr_a[2:0]];
        else
            for (int i = 0; i < NUM_LANES; i++) rdata_a[i] = bval_a;

        if (raddr_b <= 5'd7)
            rdata_b = v_reg[raddr_b[2:0]];
        else
            for (int i = 0; i < NUM_LANES; i++) rdata_b[i] = bval_b;

        if (raddr_c <= 5'd7)
            rdata_c = v_reg[raddr_c[2:0]];
        else
            for (int i = 0; i < NUM_LANES; i++) rdata_c[i] = bval_c;
    end

    always_comb begin
        rdata_pm = (pm_sel == 3'd0) ? {NUM_LANES{1'b1}} : p_reg[pm_sel];

        if (raddr_a <= 5'd7)
            rdata_p_a = (raddr_a[2:0] == 3'd0) ? {NUM_LANES{1'b1}} : p_reg[raddr_a[2:0]];
        else
            rdata_p_a = {NUM_LANES{1'b0}};

        if (raddr_b <= 5'd7)
            rdata_p_b = (raddr_b[2:0] == 3'd0) ? {NUM_LANES{1'b1}} : p_reg[raddr_b[2:0]];
        else
            rdata_p_b = {NUM_LANES{1'b0}};

        if (raddr_c <= 5'd7)
            rdata_p_c = (raddr_c[2:0] == 3'd0) ? {NUM_LANES{1'b1}} : p_reg[raddr_c[2:0]];
        else
            rdata_p_c = {NUM_LANES{1'b0}};
    end

    assign vl_out = vl_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int r = 0; r < 8; r++) begin
                v_reg[r] <= '0;
                s_reg[r] <= '0;
                p_reg[r] <= '0;
            end
            p_reg[0] <= {NUM_LANES{1'b1}};
            vl_reg   <= '0;
        end else begin
            p_reg[0] <= {NUM_LANES{1'b1}};

            if (wren_v_d && !divmod_conflict) begin
                for (int lane = 0; lane < NUM_LANES; lane++) begin
                    if (lane_mask_d[lane])
                        v_reg[waddr_d[2:0]][lane] <= format_wdata(wdata_d[lane], wsize_d);
                end
            end

            if (wren_s_d && !divmod_conflict)
                s_reg[waddr_d[2:0]] <= format_wdata(wdata_d[0], wsize_d);

            if (wren_p_d && !divmod_conflict && (waddr_d[2:0] != 3'd0)) begin
                for (int lane = 0; lane < NUM_LANES; lane++) begin
                    if (lane_mask_d[lane])
                        p_reg[waddr_d[2:0]][lane] <= wdata_p_d[lane];
                end
            end

            if (wren_v_c && !divmod_conflict) begin
                for (int lane = 0; lane < NUM_LANES; lane++) begin
                    if (lane_mask_d[lane])
                        v_reg[waddr_c[2:0]][lane] <= format_wdata(wdata_c[lane], wsize_c);
                end
            end

            if (wren_s_c && !divmod_conflict)
                s_reg[waddr_c[2:0]] <= format_wdata(wdata_c[0], wsize_c);

            if (wren_p_c && !divmod_conflict && (waddr_c[2:0] != 3'd0)) begin
                for (int lane = 0; lane < NUM_LANES; lane++) begin
                    if (lane_mask_d[lane])
                        p_reg[waddr_c[2:0]][lane] <= wdata_p_c[lane];
                end
            end

            if (setvl_en) begin
                if (setvl_val <= 31'(NUM_LANES))
                    vl_reg <= setvl_val;
                else
                    vl_reg <= 31'(NUM_LANES);
            end
        end
    end


endmodule
