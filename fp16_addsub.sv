// Combinational IEEE 754 binary16 add/subtract, round-to-nearest ties-to-even.
// sub=0: D=A+B; sub=1: D=A-B.
// Inputs must be finite. Overflow produces infinity (outside finite-result contract).
// Uses an exact fixed-point intermediate scaled by 2^-24; straightforward but
// area-intensive compared with a conventional exponent-alignment FP adder.
module fp16_addsub (
    input  logic [15:0] a,
    input  logic [15:0] b,
    input  logic        sub,
    output logic [15:0] d
);
    logic sign_a, sign_b, sign_result;
    logic [4:0] exp_a, exp_b;
    logic [9:0] frac_a, frac_b;
    logic [10:0] sig_a, sig_b;
    logic [5:0] shift_a, shift_b;
    logic [41:0] mag_a, mag_b, magnitude;
    logic [10:0] kept;
    logic [11:0] rounded;
    logic guard_bit, sticky_bit, round_up;
    integer msb, right_shift, exponent_result;

    always_comb begin
        sign_a = a[15];
        sign_b = b[15] ^ sub;
        exp_a = a[14:10];
        exp_b = b[14:10];
        frac_a = a[9:0];
        frac_b = b[9:0];

        // The implicit leading bit is 0 for subnormals and zero.
        sig_a = {(exp_a != 0), frac_a};
        sig_b = {(exp_b != 0), frac_b};

        // Exact fixed-point magnitudes in units of 2^-24.
        // Normal: (1024+fraction)*2^(exp-1).
        // Subnormal: fraction.
        shift_a = (exp_a == 0) ? 6'd0 : {1'b0, exp_a} - 6'd1;
        shift_b = (exp_b == 0) ? 6'd0 : {1'b0, exp_b} - 6'd1;
        mag_a = {31'b0, sig_a} << shift_a;
        mag_b = {31'b0, sig_b} << shift_b;

        sign_result = 1'b0;
        magnitude = 42'd0;
        kept = 11'd0;
        rounded = 12'd0;
        guard_bit = 1'b0;
        sticky_bit = 1'b0;
        round_up = 1'b0;
        msb = -1;
        right_shift = 0;
        exponent_result = 0;
        d = 16'h0000;

        if (sign_a == sign_b) begin
            magnitude = mag_a + mag_b;
            sign_result = sign_a;
        end else if (mag_a >= mag_b) begin
            magnitude = mag_a - mag_b;
            sign_result = sign_a;
        end else begin
            magnitude = mag_b - mag_a;
            sign_result = sign_b;
        end

        if (magnitude == 0) begin
            // Exact cancellation and both-zero case: +0 under RNE,
            // except -0 + -0 yields -0.
            d = {((mag_a == 0) && (mag_b == 0) &&
                  sign_a && sign_b), 15'b0};
        end else begin
            // Find highest set bit in the exact magnitude.
            for (int i = 0; i < 42; i++) begin
                if (magnitude[i])
                    msb = i;
            end

            if (msb < 10) begin
                // Subnormal: exact at the binary16 subnormal quantum.
                d = {sign_result, 5'b00000, magnitude[9:0]};
            end else begin
                // Keep 11 significant bits (including hidden bit).
                right_shift = msb - 10;
                kept = 11'(magnitude >> right_shift);

                // Round-to-nearest, ties-to-even using ALL discarded bits.
                if (right_shift > 0) begin
                    guard_bit = magnitude[right_shift-1];
                    for (int i = 0; i < 42; i++) begin
                        if (i < right_shift-1)
                            sticky_bit |= magnitude[i];
                    end
                end
                round_up = guard_bit && (sticky_bit || kept[0]);
                rounded = {1'b0, kept} + {11'b0, round_up};
                exponent_result = msb - 9;

                if (rounded[11]) begin
                    rounded = rounded >> 1;
                    exponent_result = exponent_result + 1;
                end

                if (exponent_result >= 31) begin
                    // Overflow is outside the stated finite-result contract.
                    d = {sign_result, 5'b11111, 10'b0};
                end else begin
                    d = {sign_result, 5'(exponent_result), rounded[9:0]};
                end
            end
        end
    end
endmodule
