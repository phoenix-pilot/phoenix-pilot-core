import csv
from common.models import logs_types, utils


class CsvLogParser:
    def parse(self, file_path: str):
        result = []

        with open(file_path, "r") as file:
            csv_reader = csv.reader(file, delimiter=',')
            for row in csv_reader:
                if len(row) < 2:
                    continue

                if row[1] == "Imu":
                    result.append(self.__parse_imu_log(row))
                elif row[1] == "Gps":
                    result.append(self.__parse_gps_log(row))
                elif row[1] == "Time":
                    result.append(self.__parse_time_log(row))
                elif row[1] == "Baro":
                    result.append(self.__parse_baro_log(row))
                else:
                    print(f"Unknown entry in log file {row}")

        return result

    def __parse_imu_log(self, row: list[str]) -> logs_types.ImuLog:
        id = int(row[0])
        timestamp = int(row[2])
        accel = utils.Vector3(int(row[3]), int(row[4]), int(row[5]))
        gyro = utils.Vector3(int(row[6]), int(row[7]), int(row[8]))
        gyro_d_angle = utils.Vector3(int(row[9]), int(row[10]), int(row[11]))
        mag = utils.Vector3(int(row[12]), int(row[13]), int(row[14]))

        return logs_types.ImuLog(id, timestamp, accel, gyro, gyro_d_angle, mag)

    def __parse_gps_log(self, row: list[str]) -> logs_types.GpsLog:
        id = int(row[0])
        timestamp = int(row[2])
        position = utils.GlobalPosition(int(row[3]), int(row[4]), int(row[5]))
        utc = int(row[6])
        horizontal_prec_dilution = int(row[7])
        vertical_prec_dilution = int(row[8])
        alt_ellipsoid = int(row[9])
        ground_speed = int(row[9])
        velocity = utils.NEDCoordinates(int(row[10]), int(row[11]), int(row[12]))
        horizontal_acc = int(row[13])
        vertical_acc = int(row[14])
        velocity_acc = int(row[15])
        heading = int(row[16])
        heading_offset = int(row[17])
        heading_acc = int(row[18])
        satellite_number = int(row[19])
        fix = int(row[20])

        return logs_types.GpsLog(
            id,
            timestamp,
            position,
            utc,
            horizontal_prec_dilution,
            vertical_prec_dilution,
            alt_ellipsoid,
            ground_speed,
            velocity,
            horizontal_acc,
            vertical_acc,
            velocity_acc,
            heading,
            heading_offset,
            heading_acc,
            satellite_number,
            fix
        )

    def __parse_time_log(self, row: list[str]) -> logs_types.TimeLog:
        id = int(row[0])
        timestamp = int(row[2])

        return logs_types.TimeLog(id, timestamp)

    def __parse_baro_log(self, row: list[str]) -> logs_types.BaroLog:
        id = int(row[0])
        timestamp = int(row[2])
        pressure = int(row[3])
        temperature = int(row[4])

        return logs_types.BaroLog(id, timestamp, pressure, temperature)
