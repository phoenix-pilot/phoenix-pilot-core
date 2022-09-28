# `quat.h` library unit tests

## File structure

- `basics.c` - contains tests not categorized to other files. Tested functions:
    - `quat_cmp`
    - `quat_idenWrite`
    - `quat_piWrite`
    - `quat_add`
    - `quat_sum`
    - `quat_sub`
    - `quat_dif`
    - `quat_mlt`
    - `quat_times`
    - `quat_cjg`
    - `quat_dot`
    - `quat_sandwich`
    - `quat_normalize`
- `buffs.h` - contains data used in tests
- `rotations.c` - contains tests for functions used in rotations in 3D space.
    - `quat_quat2euler`
    - `quat_vecRot`
    - `quat_rotQuat`
    - `quat_uvec2uvec`
    - `quat_frameRot`
