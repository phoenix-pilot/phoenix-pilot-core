# Algebra unit tests

This directory contains unit tests of libraries responsible for operations on matrices, vectors and quaternions.

## File structure

- `matrix/` - contains unit tests for matrix library
- `quat/` - contains unit tests for quaternions library
- `vec/` - contains unit tests for vectors library
- `qdiff/` - contains unit tests for quaternions differentiation
- `main.c` - main program responsible for running other unit tests
- `Makefile` - file needed for building tests onto the target
- `tools_tests.c` - contains unit tests for more complicated function from `tools.h` library
- `tools.c` - implements functions used during testing
- `tools.h` - header file with macros and functions used during testing
