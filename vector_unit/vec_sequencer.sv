`timescale 1ns / 1ps

// =============================================================================
// Vector Chunk Sequencer (FSM)
//
// Bridges the physical execution datapath (NUM_LANES) with the multi-chunk
// virtual register file (register_file_chunked).
//
// When NUM_CHUNKS = 1 (or for scalar / SETVL ops):
//   Executes immediately in single-cycle (or standard multi-cycle unit latency)
//   without chunk looping overhead.
//
// When NUM_CHUNKS > 1 and is_vector_op = 1:
//   Iterates chunk-by-chunk from 0 to last_chunk_idx:
//   - Steps rchunk_idx to feed each chunk slice to physical lanes.
//   - Pulses wchunk_valid_d/c to stage each chunk's results into the RF buffer.
//   - For multi-cycle ops (DIV, RSQRT), triggers chunk_start for each chunk
//     and waits for lanes_finished.
//   - On the final chunk, asserts wchunk_commit_d and done to atomically
//     commit all staged chunks into architectural registers.
// =============================================================================

module vec_sequencer #(
  parameter int NUM_LANES  = 8,
  parameter int NUM_CHUNKS = 1,
  localparam int CHUNK_WIDTH   = (NUM_CHUNKS > 1) ? $clog2(NUM_CHUNKS) : 1,
  localparam int VIRTUAL_LANES = NUM_LANES * NUM_CHUNKS
) (
  input  logic                   clk,
  input  logic                   rst_n,

  // Command & status from vec_simd / vec_decoder
  input  logic                   valid,
  input  logic                   faulted,          // sticky sequence fault
  input  logic                   fault,            // current cycle fault / conflict
  input  logic                   is_vector_op,     // true if vector D or predicate D
  input  logic                   multi_cycle,      // DIV, RSQRT, etc.
  input  logic                   lanes_finished,   // &lane_finished from physical lanes
  input  logic [31:0]            vl,

  // Raw write enables from vec_simd
  input  logic                   wren_v_d_raw,
  input  logic                   wren_p_d_raw,
  input  logic                   wren_v_c_raw,

  // Execution triggers back to vec_simd / lanes
  output logic                   chunk_start,      // pulse to start multi-cycle units
  output logic                   busy,
  output logic                   done,
  output logic                   seq_commit,       // high on final commit cycle

  // Handshake signals to register_file_chunked
  output logic [CHUNK_WIDTH-1:0] rchunk_idx,
  output logic [CHUNK_WIDTH-1:0] wchunk_idx_d,
  output logic                   wchunk_valid_d,
  output logic                   wchunk_commit_d,
  output logic [CHUNK_WIDTH-1:0] wchunk_idx_c,
  output logic                   wchunk_valid_c
);

  typedef enum logic [1:0] {
    S_IDLE,
    S_CHUNK_RUN,
    S_WAIT_MULTI,
    S_START_MULTI
  } seq_state_t;

  seq_state_t state, next_state;
  logic [CHUNK_WIDTH-1:0] chunk_cnt, next_chunk_cnt;
  logic [CHUNK_WIDTH-1:0] last_chunk_idx;

  // Active chunks calculation:
  // Non-vector operations, empty VL, or NUM_CHUNKS=1 only execute chunk 0
  always_comb begin
    if (NUM_CHUNKS <= 1 || !is_vector_op || vl == 0) begin
      last_chunk_idx = '0;
    end else if (vl >= VIRTUAL_LANES[31:0]) begin
      last_chunk_idx = CHUNK_WIDTH'(NUM_CHUNKS - 1);
    end else begin
      last_chunk_idx = CHUNK_WIDTH'((vl - 1) / NUM_LANES[31:0]);
    end
  end

  // Sequential state and chunk counter register
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state     <= S_IDLE;
      chunk_cnt <= '0;
    end else if (fault || faulted) begin
      state     <= S_IDLE;
      chunk_cnt <= '0;
    end else begin
      state     <= next_state;
      chunk_cnt <= next_chunk_cnt;
    end
  end

  // Next-state and control combinational logic
  always_comb begin
    next_state     = state;
    next_chunk_cnt = chunk_cnt;
    chunk_start    = 1'b0;
    done           = 1'b0;
    seq_commit     = 1'b0;
    busy           = 1'b0;

    rchunk_idx      = chunk_cnt;
    wchunk_idx_d    = chunk_cnt;
    wchunk_idx_c    = chunk_cnt;
    wchunk_valid_d  = 1'b0;
    wchunk_commit_d = 1'b0;
    wchunk_valid_c  = 1'b0;

    case (state)
      S_IDLE: begin
        rchunk_idx   = '0;
        wchunk_idx_d = '0;
        wchunk_idx_c = '0;

        if (valid && !faulted && !fault) begin
          if (multi_cycle) begin
            // Multi-cycle operation starting on chunk 0
            chunk_start    = 1'b1;
            busy           = 1'b1;
            next_state     = S_WAIT_MULTI;
            next_chunk_cnt = '0;
          end else if (is_vector_op && (NUM_CHUNKS > 1) && (last_chunk_idx > 0)) begin
            // Multi-chunk single-cycle ALU: stage chunk 0 now, advance to next
            wchunk_valid_d = (wren_v_d_raw || wren_p_d_raw);
            wchunk_valid_c = wren_v_c_raw;
            busy           = 1'b1;
            next_state     = S_CHUNK_RUN;
            next_chunk_cnt = CHUNK_WIDTH'(1);
          end else begin
            // Single-cycle single-chunk (or scalar / SETVL): commit immediately
            wchunk_valid_d  = (wren_v_d_raw || wren_p_d_raw);
            wchunk_commit_d = 1'b1;
            wchunk_valid_c  = wren_v_c_raw;
            seq_commit      = 1'b1;
            done            = 1'b1;
            busy            = 1'b0;
            next_state      = S_IDLE;
            next_chunk_cnt  = '0;
          end
        end
      end

      S_CHUNK_RUN: begin
        // Single-cycle ALUs looping across chunks 1..last_chunk_idx
        busy           = 1'b1;
        wchunk_valid_d = (wren_v_d_raw || wren_p_d_raw);
        wchunk_valid_c = wren_v_c_raw;

        if (chunk_cnt == last_chunk_idx) begin
          // Final chunk reached: commit buffer and assert done
          wchunk_commit_d = 1'b1;
          seq_commit      = 1'b1;
          done            = 1'b1;
          next_state      = S_IDLE;
          next_chunk_cnt  = '0;
        end else begin
          wchunk_commit_d = 1'b0;
          next_chunk_cnt  = chunk_cnt + 1'b1;
          next_state      = S_CHUNK_RUN;
        end
      end

      S_WAIT_MULTI: begin
        // Multi-cycle operation executing on chunk chunk_cnt
        busy = 1'b1;

        if (lanes_finished) begin
          wchunk_valid_d = (wren_v_d_raw || wren_p_d_raw);
          wchunk_valid_c = wren_v_c_raw;

          if (chunk_cnt == last_chunk_idx) begin
            // All chunks finished: commit buffer atomically
            wchunk_commit_d = 1'b1;
            seq_commit      = 1'b1;
            done            = 1'b1;
            next_state      = S_IDLE;
            next_chunk_cnt  = '0;
          end else begin
            // Move to start next chunk
            next_chunk_cnt = chunk_cnt + 1'b1;
            next_state     = S_START_MULTI;
          end
        end
      end

      S_START_MULTI: begin
        // Pulse chunk_start for 1 cycle to kick off lanes for next chunk
        busy        = 1'b1;
        chunk_start = 1'b1;
        next_state  = S_WAIT_MULTI;
      end

      default: begin
        next_state     = S_IDLE;
        next_chunk_cnt = '0;
      end
    endcase
  end

endmodule
