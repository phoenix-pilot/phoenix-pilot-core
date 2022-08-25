# `matrix.h` library tests

## File structure

- `basic_interactions.c` - contains tests for functions responsible for basic manipulation, creating and deleting matrices. Tested functions:
    - `matrix_rowsGet`
    - `matrix_colsGet`
    - `matrix_at`
    - `matrix_bufAlloc`
    - `matrix_bufFree`
- `various.c` - contains tests not categorized to other categories. Tested functions:
    - `matrix_trp`
    - `matrix_zeroes`
