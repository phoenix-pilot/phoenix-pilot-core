# `matrix.h` library unit tests

## File structure

- `addition_and_subtraction.c` - functions tested it this directory:
    - `matrix_add`
    - `matrix_sub`
- `basic_interactions.c` - contains tests for functions responsible for basic manipulation, creating and deleting matrices. Tested functions:
    - `matrix_rowsGet`
    - `matrix_colsGet`
    - `matrix_at`
    - `matrix_bufAlloc`
    - `matrix_bufFree`
- `inverse.c` - tested function:
    - `matrix_inv`
- `mat_various.c` - contains tests not categorized to other files. Tested functions:
    - `matrix_trp`
    - `matrix_zeroes`
    - `matrix_diag`
    - `matrix_times`
    - `matrix_writeSubmatrix`
    - `matrix_cmp`
- `matrix_multiplication.c` - tested functions:
    - `matrix_prod`
    - `matrix_sparseProd`
- `sandwitch.c` - tested functions:
    - `matrix_sandwitch`
    - `matrix_sparseSandwitch`
- `tools_tests.c` - contains tests for functions used in testing `matrix` library. Tested functions:
    - `algebraTests_realTrp`
    - `algebraTests_submatrixCheck`
