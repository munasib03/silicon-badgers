// bpu_vec_int_pkg.sv -- encodings for the SIMD lane integer ALU (ALU SIG = FN*8 + TYPE)
package vec_int_pkg;
  typedef enum logic [3:0] {
    ADD        = 4'd0,
    SUB        = 4'd1,
    EQ         = 4'd2,
    LT         = 4'd3,
    SEL        = 4'd4,
    AND        = 4'd5,
    XOR        = 4'd6,
    SHL        = 4'd7,
    SHR        = 4'd8,
    CLZ        = 4'd9,
    ADD_SAT    = 4'd10,
    SUB_SAT    = 4'd11,
    RSHIFT_RNE = 4'd12
  } int_fn_e;

  typedef enum logic [2:0] {
    F16 = 3'd0,
    F32 = 3'd1,
    I32 = 3'd2,
    U32 = 3'd3,
    I64 = 3'd4,
    U64 = 3'd5
  } alu_type_e;
endpackage