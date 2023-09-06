from struct import Struct

# These formats are based on structures from `libsensors.h`

LOG_PREFIX = Struct("<IcQ")

ACCEL_STR_FORMAT = "IiiiI"
GYRO_STR_FORMAT = "IiiiIIII"
MAG_STR_FORMAT = "Ihhhxx"
IMU = Struct(ACCEL_STR_FORMAT + GYRO_STR_FORMAT + MAG_STR_FORMAT)

GPS = Struct("IiqqQHHiIiiiIIIHhHBB4x")

BARO = Struct("III")
