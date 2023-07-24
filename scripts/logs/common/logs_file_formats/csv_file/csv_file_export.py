import csv
import common.models.log_reading as logs_types

from common.models.visitor import LogsVisitor

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


    def visit_time_log(self, timeLog: logs_types.TimeLog):
        self.__csv_row_prefix_add(timeLog, "Time")


    def visit_imu_log(self, imuLog: logs_types.ImuLog):
        self.__csv_row_prefix_add(imuLog, "Imu")

        self.act_row.append(str(imuLog.accel.x))
        self.act_row.append(str(imuLog.accel.y))
        self.act_row.append(str(imuLog.accel.z))

        self.act_row.append(str(imuLog.gyro.x))
        self.act_row.append(str(imuLog.gyro.y))
        self.act_row.append(str(imuLog.gyro.z))

        self.act_row.append(str(imuLog.gyroDAngle.x))
        self.act_row.append(str(imuLog.gyroDAngle.y))
        self.act_row.append(str(imuLog.gyroDAngle.z))

        self.act_row.append(str(imuLog.mag.x))
        self.act_row.append(str(imuLog.mag.y))
        self.act_row.append(str(imuLog.mag.z))


    def visit_gps_log(self, gpsLog: logs_types.GpsLog):
        self.__csv_row_prefix_add(gpsLog, "Gps")

        self.act_row.append(str(gpsLog.position.latitude))
        self.act_row.append(str(gpsLog.position.longitude))
        self.act_row.append(str(gpsLog.position.altitude))

        self.act_row.append(str(gpsLog.horizontalAccuracy))
        self.act_row.append(str(gpsLog.velocityAccuracy))

        self.act_row.append(str(gpsLog.fix))
        self.act_row.append(str(gpsLog.satelliteNumber))

        self.act_row.append(str(gpsLog.velocity.north))
        self.act_row.append(str(gpsLog.velocity.east))
        self.act_row.append(str(gpsLog.velocity.down))


    def visit_baro_log(self, baroLog: logs_types.BaroLog):
        self.__csv_row_prefix_add(baroLog, "Baro")

        self.act_row.append(str(baroLog.pressure))
        self.act_row.append(str(baroLog.temperature))
