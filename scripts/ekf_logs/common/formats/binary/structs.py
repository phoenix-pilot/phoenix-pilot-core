from struct import Struct

# These formats are based on structures from `libsensors.h`

LOG_PREFIX = Struct("<IcQ")

ACCEL_STR_FORMAT = "Iiii"
GYRO_STR_FORMAT = "IiiiIII"
MAG_STR_FORMAT = "Ihhhxx"
IMU = Struct(ACCEL_STR_FORMAT + GYRO_STR_FORMAT + MAG_STR_FORMAT)

POSITION_STR_FORMAT = "iqq"
GPS = Struct("IiqqQQQiIiiiIIIHhHBB")

BARO = Struct("III")
