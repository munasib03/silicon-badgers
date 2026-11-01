
`timescale 1ns / 1ps

module scalar_regfile #(
    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    parameter int unsigned NUM_REGS   = 8,   // Number of registers (S0..S7)
    parameter int unsigned DATA_WIDTH = 64,  // Data width of each register in bits
    parameter int unsigned BASE_ADDR  = 8,   // Starting architectural address (S0 = 8)
    parameter int unsigned ADDR_WIDTH = 4,   // Address bus width (covers 8..15)
    parameter bit          FORWARDING = 1    // 1: Enable write-through forwarding, 0: Disable
)(
    // -------------------------------------------------------------------------
    // 1. Clock & Reset
    // -------------------------------------------------------------------------
    input  logic                   clk,       // System clock
    input  logic                   rst_n,     // Synchronous active-low reset

    // -------------------------------------------------------------------------
    // 2. Read Port 1 (r1)
    // -------------------------------------------------------------------------
    input  logic [ADDR_WIDTH-1:0]  r1_addr,   // Source register 1 address (8..15)
    output logic [DATA_WIDTH-1:0]  r1,        // Read data 1 (64-bit)

    // -------------------------------------------------------------------------
    // 3. Read Port 2 (r2)
    // -------------------------------------------------------------------------
    input  logic [ADDR_WIDTH-1:0]  r2_addr,   // Source register 2 address (8..15)
    output logic [DATA_WIDTH-1:0]  r2,        // Read data 2 (64-bit)

    // -------------------------------------------------------------------------
    // 4. Write-back Port (rd)
    // -------------------------------------------------------------------------
    input  logic                   rd_wr_en,  // Destination write enable
    input  logic [ADDR_WIDTH-1:0]  rd_addr,   // Destination register address (8..15)
    input  logic [DATA_WIDTH-1:0]  rd,        // Write data (64-bit)

);

    // =========================================================================
    // Local Constants & Address Validation Functions
    // =========================================================================
    localparam int unsigned MAX_ADDR = BASE_ADDR + NUM_REGS - 1;

    // Check if an address falls within [BASE_ADDR, MAX_ADDR] (8..15)
    function automatic logic is_valid_addr(input logic [ADDR_WIDTH-1:0] addr);
        return (addr >= BASE_ADDR[ADDR_WIDTH-1:0]) && (addr <= MAX_ADDR[ADDR_WIDTH-1:0]);
    endfunction

    // Map architectural address (8..15) to internal index (0..7)
    function automatic logic [$clog2(NUM_REGS)-1:0] get_reg_idx(input logic [ADDR_WIDTH-1:0] addr);
        return (addr - BASE_ADDR[ADDR_WIDTH-1:0]);
    endfunction

    // =========================================================================
    // Internal Storage: 8 x 64-bit registers
    //   regfile[0] = S0 (addr 8)
    //   regfile[1] = S1 (addr 9)
    //   ...
    //   regfile[7] = S7 (addr 15)
    // =========================================================================
    logic [DATA_WIDTH-1:0] regfile [NUM_REGS];

    // =========================================================================
    // Synchronous Write Logic (Active-low reset, rising clock edge)
    // =========================================================================
    always_ff @(posedge clk) begin : proc_write
        if (!rst_n) begin
            for (int i = 0; i < NUM_REGS; i++) begin
                regfile[i] <= '0;
            end
        end else if (rd_wr_en && is_valid_addr(rd_addr)) begin
            regfile[get_reg_idx(rd_addr)] <= rd;
        end
    end 

    // =========================================================================
    // Read Port 1 (r1) — Combinational / Asynchronous Read
    //   Includes optional write-through forwarding to eliminate RAW bubbles
    // =========================================================================
    always_comb begin 
        if (!is_valid_addr(r1_addr)) begin
            r1 = '0;
        end else if (FORWARDING && rd_wr_en && (rd_addr == r1_addr)) begin
            r1 = rd;  // Forward new write-back data
        end else begin
            r1 = regfile[get_reg_idx(r1_addr)];
        end
    end 

    // =========================================================================
    // Read Port 2 (r2) — Combinational / Asynchronous Read
    //   Includes optional write-through forwarding to eliminate RAW bubbles
    // =========================================================================
    always_comb begin 
        if (!is_valid_addr(r2_addr)) begin
            r2 = '0;
        end else if (FORWARDING && rd_wr_en && (rd_addr == r2_addr)) begin
            r2 = rd;  // Forward new write-back data
        end else begin
            r2 = regfile[get_reg_idx(r2_addr)];
        end
    end 

endmodule
