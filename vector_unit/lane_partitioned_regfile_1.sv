
// Parameters
// ──────────
//   NUM_REGS    : architectural vector registers  (default 8, v0–v7)
//   NUM_LANES   : processing lanes                (default 8)
//   ELEM_WIDTH  : bits per lane element           (default 64)
//
// Derived (do not override)
//   ADDR_WIDTH  = $clog2(NUM_REGS)
//   REG_WIDTH   = NUM_LANES * ELEM_WIDTH   (flat bit-width of one register)
//   SRAM_BUS_WIDTH = NUM_REGS * REG_WIDTH  (full file on 512-byte SRAM bus)
//
// Port groups
// ───────────
//   1. Clock / reset
//   2. Read port A  (rs1)  — NUM_LANES ELEM_WIDTH-bit combinational outputs
//   3. Read port B  (rs2)  — same, second source operand
//   4. Write-back port     — one rd_addr, per-lane data + per-lane wr_en
//   5. Bulk SRAM bus       — 512-byte parallel read / write of entire file
//
// =============================================================================

`timescale 1ns / 1ps

module lane_partitioned_regfile #(

    parameter int unsigned NUM_REGS   = 8,   // number of architectural registers (v0–v7)
    parameter int unsigned NUM_LANES  = 8,   // number of parallel processing lanes
    parameter int unsigned ELEM_WIDTH = 64,  // bits per lane element (64-bit / double)

    parameter int unsigned ADDR_WIDTH     = $clog2(NUM_REGS),
    parameter int unsigned REG_WIDTH      = NUM_LANES  * ELEM_WIDTH,   // bits per register
    parameter int unsigned SRAM_BUS_WIDTH = NUM_REGS   * REG_WIDTH,    // total file width
    parameter int unsigned SRAM_BUS_BYTES = SRAM_BUS_WIDTH / 8         // bulk byte-enable count
)(

    input  logic                        clk,    // rising-edge active
    input  logic                        rst_n,  // synchronous, active-low

    input  logic [ADDR_WIDTH-1:0]       rs1_addr,                   // source register A
    output logic [ELEM_WIDTH-1:0]       rs1_lane_data [NUM_LANES],  // element per lane

    input  logic [ADDR_WIDTH-1:0]       rs2_addr,                   // source register B
    output logic [ELEM_WIDTH-1:0]       rs2_lane_data [NUM_LANES],  // element per lane

    input  logic                        rd_wr_en,                   // global write-back gate
    input  logic [ADDR_WIDTH-1:0]       rd_addr,                    // destination register
    input  logic [ELEM_WIDTH-1:0]       rd_lane_data [NUM_LANES],   // one result per lane
    input  logic [NUM_LANES-1:0]        rd_lane_wr_en,              // per-lane write enable / mask

    // =========================================================================
    // 5. Bulk SRAM interface — 512-byte parallel bus
    //
    //    Matches the SRAM's native 4096-bit (512-byte) output word so that the
    //    entire register file can be saved or restored in a single SRAM access.
    //
    //    Bus packing — register r occupies:
    //      bits [ r*REG_WIDTH +: REG_WIDTH ]
    //    Within each register slot, lane l occupies:
    //      bits [ r*REG_WIDTH + l*ELEM_WIDTH +: ELEM_WIDTH ]
    //
    //    Bulk read  — combinational; valid while bulk_rd_en = 1.
    //    Bulk write — synchronous; committed on rising edge when bulk_wr_en = 1.
    //    Do NOT assert bulk_rd_en and bulk_wr_en in the same cycle.
    // =========================================================================
    input  logic                        bulk_rd_en,             // enable bulk read output
    output logic [SRAM_BUS_WIDTH-1:0]   bulk_rd_data,           // 4096-bit flat read bus

    input  logic                        bulk_wr_en,             // enable bulk write
    input  logic [SRAM_BUS_WIDTH-1:0]   bulk_wr_data,           // 4096-bit flat write data
    input  logic [SRAM_BUS_BYTES-1:0]   bulk_wr_byte_en         // per-byte mask (512 bits)
);

    // =========================================================================
    // Internal storage — 2-D array  [register][lane]
    //   regfile[r][l] = the ELEM_WIDTH-bit element in register r for lane l
    // =========================================================================
    logic [ELEM_WIDTH-1:0] regfile [NUM_REGS][NUM_LANES];

    // =========================================================================
    // Synchronous Write Logic
    //   Priority:  reset  >  lane write-back  >  bulk write
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int r = 0; r < NUM_REGS; r++)
                for (int l = 0; l < NUM_LANES; l++)
                    regfile[r][l] <= '0;
        end 
        else begin

            // -----------------------------------------------------------------
            // Lane write-back
            //   rd_wr_en gates the whole operation.
            //   rd_lane_wr_en[l] individually enables each lane.
            // -----------------------------------------------------------------
            if (rd_wr_en) begin
                for (int l = 0; l < NUM_LANES; l++) begin
                    if (rd_lane_wr_en[l])
                        regfile[rd_addr][l] <= rd_lane_data[l];
                end
            end

            // -----------------------------------------------------------------
            // Bulk write — load the entire file from the 512-byte SRAM bus.
            //   Each register's slot in the flat bus is:
            //     bus[ r*REG_WIDTH +: REG_WIDTH ]
            //   Each lane element within that slot:
            //     bus[ r*REG_WIDTH + l*ELEM_WIDTH +: ELEM_WIDTH ]
            //   Per-byte enable covers the full bus; byte b of lane l of
            //   register r maps to bus byte  r*(REG_WIDTH/8) + l*(ELEM_WIDTH/8) + <byte_in_elem>.
            // -----------------------------------------------------------------
            if (bulk_wr_en) begin
                for (int r = 0; r < NUM_REGS; r++) begin
                    for (int l = 0; l < NUM_LANES; l++) begin
                        for (int b = 0; b < ELEM_WIDTH/8; b++) begin
                            // Global byte offset in the 512-byte bus
                            automatic int gb =
                                r * (REG_WIDTH/8) +
                                l * (ELEM_WIDTH/8) + b;
                            if (bulk_wr_byte_en[gb])
                                regfile[r][l][b*8 +: 8] <= bulk_wr_data[gb*8 +: 8];
                        end
                    end
                end
            end

        end
    end

    // =========================================================================
    // Read port A — rs1
    //   Combinational fan-out: all NUM_LANES elements of regfile[rs1_addr]
    //   are presented in parallel.
    //
    //   Write-through forwarding:
    //     If the write-back port is writing to the same register this cycle,
    //     lanes that have rd_lane_wr_en[l]=1 see the NEW value from rd_lane_data;
    //     lanes with rd_lane_wr_en[l]=0 see the stored value.
    //   This prevents a RAW (read-after-write) hazard without inserting a bubble.
    // =========================================================================
    always_comb begin 
        for (int l = 0; l < NUM_LANES; l++) begin
            if (rd_wr_en && (rs1_addr == rd_addr) && rd_lane_wr_en[l])
                rs1_lane_data[l] = rd_lane_data[l];   // forward new result
            else
                rs1_lane_data[l] = regfile[rs1_addr][l];
        end
    end 

    // =========================================================================
    // Read port B — rs2
    //   Identical forwarding logic for the second source operand.
    // =========================================================================
    always_comb begin 
        for (int l = 0; l < NUM_LANES; l++) begin
            if (rd_wr_en && (rs2_addr == rd_addr) && rd_lane_wr_en[l])
                rs2_lane_data[l] = rd_lane_data[l];   // forward new result
            else
                rs2_lane_data[l] = regfile[rs2_addr][l];
        end
    end 

    // =========================================================================
    // Bulk read — 512-byte SRAM parallel output
    //   Packs the entire 2-D array into the flat SRAM bus (combinational).
    //   Bus is driven to 0 when bulk_rd_en is de-asserted.
    // =========================================================================
    always_comb begin 
        if (bulk_rd_en) begin
            for (int r = 0; r < NUM_REGS; r++)
                for (int l = 0; l < NUM_LANES; l++)
                    bulk_rd_data[r*REG_WIDTH + l*ELEM_WIDTH +: ELEM_WIDTH] = regfile[r][l];
        end else begin
            bulk_rd_data = '0;
        end
    end


endmodule 

