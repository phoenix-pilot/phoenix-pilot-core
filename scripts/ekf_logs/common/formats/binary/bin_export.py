from common.models import LogsVisitor, logs_types
from typing import Iterator

import common.formats.binary.structs as structs
import common.formats.binary.specifiers as specifiers


class BinaryLogExporter(LogsVisitor):
    def __init__(self) -> None:
        self.file = None

    def export(self, file_path, logs: Iterator[logs_types.LogEntry]) -> None:
        try:
            self.file = open(file_path, "wb")
            for log in logs:
                log.accept(self)
        finally:
            if self.file is not None:
                self.file.close()
                self.file = None

    def visit_time_log(self, time_log: logs_types.TimeLog):
        self.__write_prefix(time_log, specifiers.TIME_LOG)

    def visit_imu_log(self, imu_log: logs_types.ImuLog):
        self.__write_prefix(imu_log, specifiers.IMU_LOG)
        self.file.write(
            structs.IMU.pack(
                imu_log.data.accelDeviceID,
                imu_log.data.accel.x,
                imu_log.data.accel.y,
                imu_log.data.accel.z,
                imu_log.data.accelTemp,
                imu_log.data.gyroDeviceID,
                imu_log.data.gyro.x,
                imu_log.data.gyro.y,
                imu_log.data.gyro.z,
                imu_log.data.gyroDAngle.x,
                imu_log.data.gyroDAngle.y,
                imu_log.data.gyroDAngle.z,
                imu_log.data.gyroTemp,
                imu_log.data.magDeviceID,
                imu_log.data.mag.x,
                imu_log.data.mag.y,
                imu_log.data.mag.z
            )
        )

    def visit_gps_log(self, gps_log: logs_types.GpsLog):
        self.__write_prefix(gps_log, specifiers.GPS_LOG)
        self.file.write(
            structs.GPS.pack(
                gps_log.data.deviceID,
                gps_log.data.position.altitude,
                gps_log.data.position.latitude,
                gps_log.data.position.longitude,
                gps_log.data.utc,
                gps_log.data.horizontalPrecisionDilution,
                gps_log.data.verticalPrecisionDilution,
                gps_log.data.ellipsoidAlt,
                gps_log.data.groundSpeed,
                gps_log.data.velocity.north,
                gps_log.data.velocity.east,
                gps_log.data.velocity.down,
                gps_log.data.horizontalAccuracy,
                gps_log.data.verticalAccuracy,
                gps_log.data.velocityAccuracy,
                gps_log.data.heading,
                gps_log.data.headingOffset,
                gps_log.data.headingAccuracy,
                gps_log.data.satelliteNumber,
                gps_log.data.fix
            )
        )

    def visit_baro_log(self, baro_log: logs_types.BaroLog):
        self.__write_prefix(baro_log, specifiers.BARO_LOG)
        self.file.write(
            structs.BARO.pack(
                baro_log.data.deviceID,
                baro_log.data.pressure,
                baro_log.data.temperature
            )
        )

    def visit_ekf_state_log(self, state_log: logs_types.EkfStateLog):
        self.__write_prefix(state_log, specifiers.STATE_LOG)

        attitude_quat = state_log.data.attitude.as_quat()

        self.file.write(
            structs.STATE.pack(
                # Scipy uses scalar-last quaternion format (x, y, z, w)
                attitude_quat[3],
                attitude_quat[0],
                attitude_quat[1],
                attitude_quat[2],
                state_log.data.gyroscopeBias.x,
                state_log.data.gyroscopeBias.y,
                state_log.data.gyroscopeBias.z,
                state_log.data.velocity.north,
                state_log.data.velocity.east,
                state_log.data.velocity.down,
                state_log.data.accelerometerBias.x,
                state_log.data.accelerometerBias.y,
                state_log.data.accelerometerBias.z,
                state_log.data.position.north,
                state_log.data.position.east,
                state_log.data.position.down,
                state_log.data.magneticField.x,
                state_log.data.magneticField.y,
                state_log.data.magneticField.z
            )
        )

    def __write_prefix(self, log: logs_types.LogEntry, log_type: str):
        self.file.write(structs.LOG_PREFIX.pack(log.id, bytes(log_type, "ascii"), log.timestamp))
