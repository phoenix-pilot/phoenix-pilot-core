import csv
from common.models import logs_types, LogsVisitor

import common.formats.csv.specifiers as specifiers


class CsvLogExporter(LogsVisitor):
    def __init__(self) -> None:
        self.entry = []

    def export(self, result_file_path: str, logs: list[logs_types.LogEntry]):
        with open(result_file_path, "w") as file:
            csv_writer = csv.writer(file, quoting=csv.QUOTE_MINIMAL)

            for log in logs:
                self.entry = []
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

        self.entry.append(str(imu_log.accelDevID))

        self.entry.append(str(imu_log.accel.x))
        self.entry.append(str(imu_log.accel.y))
        self.entry.append(str(imu_log.accel.z))

        self.entry.append(str(imu_log.gyroDevID))

        self.entry.append(str(imu_log.gyro.x))
        self.entry.append(str(imu_log.gyro.y))
        self.entry.append(str(imu_log.gyro.z))

        self.entry.append(str(imu_log.gyroDAngle.x))
        self.entry.append(str(imu_log.gyroDAngle.y))
        self.entry.append(str(imu_log.gyroDAngle.z))

        self.entry.append(str(imu_log.magDevID))

        self.entry.append(str(imu_log.mag.x))
        self.entry.append(str(imu_log.mag.y))
        self.entry.append(str(imu_log.mag.z))

    def visit_gps_log(self, gps_log: logs_types.GpsLog):
        self.__csv_row_prefix_add(gps_log, specifiers.GPS_LOG)

        self.entry.append(str(gps_log.devID))

        self.entry.append(str(gps_log.position.latitude))
        self.entry.append(str(gps_log.position.longitude))
        self.entry.append(str(gps_log.position.altitude))

        self.entry.append(str(gps_log.utc))

        self.entry.append(str(gps_log.horizontalPrecisionDilution))
        self.entry.append(str(gps_log.verticalPrecisionDilution))

        self.entry.append(str(gps_log.ellipsoidAlt))

        self.entry.append(str(gps_log.groundSpeed))

        self.entry.append(str(gps_log.velocity.north))
        self.entry.append(str(gps_log.velocity.east))
        self.entry.append(str(gps_log.velocity.down))

        self.entry.append(str(gps_log.horizontalAccuracy))
        self.entry.append(str(gps_log.verticalAccuracy))
        self.entry.append(str(gps_log.velocityAccuracy))

        self.entry.append(str(gps_log.heading))
        self.entry.append(str(gps_log.headingOffset))
        self.entry.append(str(gps_log.headingAccuracy))

        self.entry.append(str(gps_log.satelliteNumber))
        self.entry.append(str(gps_log.fix))

    def visit_baro_log(self, baro_log: logs_types.BaroLog):
        self.__csv_row_prefix_add(baro_log, specifiers.BARO_LOG)

        self.entry.append(str(baro_log.devID))

        self.entry.append(str(baro_log.pressure))
        self.entry.append(str(baro_log.temperature))
