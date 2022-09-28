# `matrix.h` library unit tests

## File structure

- `add_sub.c` - functions tested it this directory:
    - `matrix_add`
    - `matrix_sub`
- `basics.c` - contains tests for functions responsible for basic manipulation, creating and deleting matrices. Tested functions:
    - `matrix_rowsGet`
    - `matrix_colsGet`
    - `matrix_at`
    - `matrix_bufAlloc`
    - `matrix_bufFree`
- `buffs.h` - contains data used in tests
- `inverse.c` - tested function:
    - `matrix_inv`
- `various.c` - contains tests not categorized to other files. Tested functions:
    - `matrix_trp`
    - `matrix_zeroes`
    - `matrix_diag`
    - `matrix_times`
    - `matrix_writeSubmatrix`
    - `matrix_cmp`
- `multiplication.c` - tested functions:
    - `matrix_prod`
    - `matrix_sparseProd`
- `sandwitch.c` - tested functions:
    - `matrix_sandwitch`
    - `matrix_sparseSandwitch`
