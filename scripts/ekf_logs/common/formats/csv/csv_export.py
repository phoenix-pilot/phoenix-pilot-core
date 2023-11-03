import csv
from common.models import logs_types, LogsVisitor
from typing import Iterator

import common.formats.csv.specifiers as specifiers


class CsvLogExporter(LogsVisitor):
    def __init__(self) -> None:
        self.entry = []

    def export(self, result_file_path: str, logs: Iterator[logs_types.LogEntry]):
        with open(result_file_path, "w") as file:
            csv_writer = csv.writer(file, quoting=csv.QUOTE_MINIMAL)

            for log in logs:
                self.entry.clear()
                log.accept(self)
                csv_writer.writerow(self.entry)

    def __csv_row_prefix_add(self, log: logs_types.LogEntry, log_specifier: str):
        self.entry.append(str(log.id))
        self.entry.append(log_specifier)
        self.entry.append(str(log.timestamp))

    def visit_time_log(self, time_log: logs_types.TimeLog):
        self.__csv_row_prefix_add(time_log, specifiers.TIME_LOG)

    def visit_imu_log(self, imu_log: logs_types.ImuLog):
        self.__csv_row_prefix_add(imu_log, specifiers.IMU_LOG)

        self.entry.append(str(imu_log.data.accelDeviceID))

        self.entry.append(str(imu_log.data.accel.x))
        self.entry.append(str(imu_log.data.accel.y))
        self.entry.append(str(imu_log.data.accel.z))

        self.entry.append(str(imu_log.data.accelTemp))

        self.entry.append(str(imu_log.data.gyroDeviceID))

        self.entry.append(str(imu_log.data.gyro.x))
        self.entry.append(str(imu_log.data.gyro.y))
        self.entry.append(str(imu_log.data.gyro.z))

        self.entry.append(str(imu_log.data.gyroDAngle.x))
        self.entry.append(str(imu_log.data.gyroDAngle.y))
        self.entry.append(str(imu_log.data.gyroDAngle.z))

        self.entry.append(str(imu_log.data.gyroTemp))

        self.entry.append(str(imu_log.data.magDeviceID))

        self.entry.append(str(imu_log.data.mag.x))
        self.entry.append(str(imu_log.data.mag.y))
        self.entry.append(str(imu_log.data.mag.z))

    def visit_gps_log(self, gps_log: logs_types.GpsLog):
        self.__csv_row_prefix_add(gps_log, specifiers.GPS_LOG)

        self.entry.append(str(gps_log.data.deviceID))

        self.entry.append(str(gps_log.data.position.altitude))
        self.entry.append(str(gps_log.data.position.latitude))
        self.entry.append(str(gps_log.data.position.longitude))

        self.entry.append(str(gps_log.data.utc))

        self.entry.append(str(gps_log.data.horizontalPrecisionDilution))
        self.entry.append(str(gps_log.data.verticalPrecisionDilution))

        self.entry.append(str(gps_log.data.ellipsoidAlt))

        self.entry.append(str(gps_log.data.groundSpeed))

        self.entry.append(str(gps_log.data.velocity.north))
        self.entry.append(str(gps_log.data.velocity.east))
        self.entry.append(str(gps_log.data.velocity.down))

        self.entry.append(str(gps_log.data.horizontalAccuracy))
        self.entry.append(str(gps_log.data.verticalAccuracy))
        self.entry.append(str(gps_log.data.velocityAccuracy))

        self.entry.append(str(gps_log.data.heading))
        self.entry.append(str(gps_log.data.headingOffset))
        self.entry.append(str(gps_log.data.headingAccuracy))

        self.entry.append(str(gps_log.data.satelliteNumber))
        self.entry.append(str(gps_log.data.fix))

    def visit_baro_log(self, baro_log: logs_types.BaroLog):
        self.__csv_row_prefix_add(baro_log, specifiers.BARO_LOG)

        self.entry.append(str(baro_log.data.deviceID))

        self.entry.append(str(baro_log.data.pressure))
        self.entry.append(str(baro_log.data.temperature))

    def visit_ekf_state_log(self, state_log: logs_types.EkfStateLog):
        self.__csv_row_prefix_add(state_log, specifiers.STATE_LOG)

        attitude_quat = state_log.data.attitude.as_quat()

        # Scipy uses scalar-last quaternion format (x, y, z, w)
        self.entry.append(attitude_quat[3])
        self.entry.append(attitude_quat[0])
        self.entry.append(attitude_quat[1])
        self.entry.append(attitude_quat[2])

        self.entry.append(state_log.data.gyroscopeBias.x)
        self.entry.append(state_log.data.gyroscopeBias.y)
        self.entry.append(state_log.data.gyroscopeBias.z)

        self.entry.append(state_log.data.velocity.north)
        self.entry.append(state_log.data.velocity.east)
        self.entry.append(state_log.data.velocity.down)

        self.entry.append(state_log.data.accelerometerBias.x)
        self.entry.append(state_log.data.accelerometerBias.y)
        self.entry.append(state_log.data.accelerometerBias.z)

        self.entry.append(state_log.data.position.north)
        self.entry.append(state_log.data.position.east)
        self.entry.append(state_log.data.position.down)
