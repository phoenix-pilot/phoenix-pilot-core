from common.models import LogsVisitor, logs_types

import common.formats.binary.structs as structs
import common.formats.binary.specifiers as specifiers


class BinaryLogExporter(LogsVisitor):
    def __init__(
            self,
    ) -> None:
        self.file = None

    def export(self, file_path, logs: list[logs_types.LogEntry]) -> None:
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
                imu_log.accelDevID,
                imu_log.accel.x,
                imu_log.accel.y,
                imu_log.accel.z,
                imu_log.accelTemp,
                imu_log.gyroDevID,
                imu_log.gyro.x,
                imu_log.gyro.y,
                imu_log.gyro.z,
                imu_log.gyroDAngle.x,
                imu_log.gyroDAngle.y,
                imu_log.gyroDAngle.z,
                imu_log.gyroTemp,
                imu_log.magDevID,
                imu_log.mag.x,
                imu_log.mag.y,
                imu_log.mag.z
            )
        )

    def visit_gps_log(self, gps_log: logs_types.GpsLog):
        self.__write_prefix(gps_log, specifiers.GPS_LOG)
        self.file.write(
            structs.GPS.pack(
                gps_log.devID,
                gps_log.position.altitude,
                gps_log.position.latitude,
                gps_log.position.longitude,
                gps_log.utc,
                gps_log.horizontalPrecisionDilution,
                gps_log.verticalPrecisionDilution,
                gps_log.ellipsoidAlt,
                gps_log.groundSpeed,
                gps_log.velocity.north,
                gps_log.velocity.east,
                gps_log.velocity.down,
                gps_log.horizontalAccuracy,
                gps_log.verticalAccuracy,
                gps_log.velocityAccuracy,
                gps_log.heading,
                gps_log.headingOffset,
                gps_log.headingAccuracy,
                gps_log.satelliteNumber,
                gps_log.fix
            )
        )

    def visit_baro_log(self, baro_log: logs_types.BaroLog):
        self.__write_prefix(baro_log, specifiers.BARO_LOG)
        self.file.write(
            structs.BARO.pack(
                baro_log.devID,
                baro_log.pressure,
                baro_log.temperature
            )
        )

    def __write_prefix(self, log: logs_types.LogEntry, log_type: str):
        self.file.write(structs.LOG_PREFIX.pack(log.id, bytes(log_type, "ascii"), log.timestamp))
