import csv
from common.models import logs_types, LogsVisitor


class CsvLogExporter(LogsVisitor):
    def __init__(self) -> None:
        self.act_row = []

    def export(self, result_file_path: str, logs: list[logs_types.LogReading]):
        with open(result_file_path, "w") as file:
            csv_writer = csv.writer(file)

            for log in logs:
                self.act_row = []
                log.accept(self)
                csv_writer.writerow(self.act_row)

    def __csv_row_prefix_add(self, log: logs_types.LogReading, log_specifier: str):
        self.act_row.append(str(log.id))
        self.act_row.append(log_specifier)
        self.act_row.append(str(log.timestamp))

    def visit_time_log(self, time_log: logs_types.TimeLog):
        self.__csv_row_prefix_add(time_log, "Time")

    def visit_imu_log(self, imu_log: logs_types.ImuLog):
        self.__csv_row_prefix_add(imu_log, "Imu")

        self.act_row.append(str(imu_log.accel.x))
        self.act_row.append(str(imu_log.accel.y))
        self.act_row.append(str(imu_log.accel.z))

        self.act_row.append(str(imu_log.gyro.x))
        self.act_row.append(str(imu_log.gyro.y))
        self.act_row.append(str(imu_log.gyro.z))

        self.act_row.append(str(imu_log.gyroDAngle.x))
        self.act_row.append(str(imu_log.gyroDAngle.y))
        self.act_row.append(str(imu_log.gyroDAngle.z))

        self.act_row.append(str(imu_log.mag.x))
        self.act_row.append(str(imu_log.mag.y))
        self.act_row.append(str(imu_log.mag.z))

    def visit_gps_log(self, gps_log: logs_types.GpsLog):
        self.__csv_row_prefix_add(gps_log, "Gps")

        self.act_row.append(str(gps_log.position.latitude))
        self.act_row.append(str(gps_log.position.longitude))
        self.act_row.append(str(gps_log.position.altitude))

        self.act_row.append(str(gps_log.horizontalAccuracy))
        self.act_row.append(str(gps_log.velocityAccuracy))

        self.act_row.append(str(gps_log.fix))
        self.act_row.append(str(gps_log.satelliteNumber))

        self.act_row.append(str(gps_log.velocity.north))
        self.act_row.append(str(gps_log.velocity.east))
        self.act_row.append(str(gps_log.velocity.down))

    def visit_baro_log(self, baro_log: logs_types.BaroLog):
        self.__csv_row_prefix_add(baro_log, "Baro")

        self.act_row.append(str(baro_log.pressure))
        self.act_row.append(str(baro_log.temperature))
