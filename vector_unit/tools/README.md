# Vector unit tools

Generators and reference checkers. Checkers model the RTL arithmetic in C/Python
and compare against exact oracles (MPFR, Python `Fraction`). They check the
algorithms, not the SystemVerilog itself — a simulation testbench is still needed.

Requirements: `brew install gmp mpfr`, `pip install gmpy2`.

| File | Purpose |
|---|---|
| `gen_fp_exp_tables.py` | Regenerates the constant tables in `fp_exp.sv`; with an argument, also writes `exp_tables.h` for the C checker |
| `check_fp_exp.c` | Exhaustive EXP check: all F16 inputs, all 2^32 F32 inputs vs MPFR |
| `check_fp_divsqrt.c` | Cycle-accurate models of `div_iter` and sequential `fp_rsqrt`: integer divide vs C `/` `%`, RSQRT/RCP exhaustive (F16, F32), DIV random + edge cases, vs MPFR |
| `check_int_cast.py` | CAST (24 pairs), WMAC/MULSHIFT/MUL_CHECK, DIVMOD/DIV_RNE, ISQRT vs exact oracles |
| `check_reduce.py` | REDUCE.SUM tree order vs the spec's logical tree, any lane count |

```sh
python3 gen_fp_exp_tables.py exp_tables.h
cc -O2 -I/opt/homebrew/include -L/opt/homebrew/lib -o check_fp_exp check_fp_exp.c -lmpfr -lgmp -lm
./check_fp_exp f16
for i in 0 1 2 3 4 5 6 7; do ./check_fp_exp f32 $i 8 & done; wait     # ~2 min on 8 cores

cc -O2 -I/opt/homebrew/include -L/opt/homebrew/lib -o check_fp_divsqrt check_fp_divsqrt.c -lmpfr -lgmp -lm
./check_fp_divsqrt int 20000000; ./check_fp_divsqrt rsqrt16; ./check_fp_divsqrt rcp16; ./check_fp_divsqrt div32 30000000
for i in 0 1 2 3 4 5 6 7; do ./check_fp_divsqrt rsqrt32 $i 8 & done; wait

python3 check_int_cast.py 1000000
python3 check_reduce.py
```

Note: running `gen_fp_exp_tables.py` rewrites the generated region of
`fp_exp.sv`; close that file in your editor first.
