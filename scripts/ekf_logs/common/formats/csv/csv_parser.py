import csv
from common.models import logs_types, utils

from scipy.spatial.transform import Rotation

import common.formats.csv.specifiers as specifiers


class CsvLogParser:
    def parse(self, file_path: str):
        result = []

        with open(file_path, "r") as file:
            csv_reader = csv.reader(file, delimiter=',')
            for row in csv_reader:
                if len(row) < 2:
                    continue

                if row[1] == specifiers.IMU_LOG:
                    result.append(self.__parse_imu_log(row))
                elif row[1] == specifiers.GPS_LOG:
                    result.append(self.__parse_gps_log(row))
                elif row[1] == specifiers.TIME_LOG:
                    result.append(self.__parse_time_log(row))
                elif row[1] == specifiers.BARO_LOG:
                    result.append(self.__parse_baro_log(row))
                elif row[1] == specifiers.STATE_LOG:
                    result.append(self.__parse_state_log(row))
                else:
                    print(f"Unknown entry in log file {row}")

        return result

    def __parse_imu_log(self, row: list[str]) -> logs_types.ImuLog:
        id = int(row[0])
        timestamp = int(row[2])

        accel_device_id = int(row[3])
        accel = utils.Vector3(int(row[4]), int(row[5]), int(row[6]))
        accel_temp = int(row[7])

        gyro_device_id = int(row[8])
        gyro = utils.Vector3(int(row[9]), int(row[10]), int(row[11]))
        gyro_d_angle = utils.Vector3(int(row[12]), int(row[13]), int(row[14]))
        gyro_temp = int(row[15])

        mag_device_id = int(row[16])
        mag = utils.Vector3(int(row[17]), int(row[18]), int(row[19]))

        return logs_types.ImuLog(
            id,
            timestamp,
            accel_device_id,
            accel,
            accel_temp,
            gyro_device_id,
            gyro,
            gyro_d_angle,
            gyro_temp,
            mag_device_id,
            mag
        )

    def __parse_gps_log(self, row: list[str]) -> logs_types.GpsLog:
        id = int(row[0])
        timestamp = int(row[2])
        device_id = int(row[3])
        position = utils.GlobalPosition(int(row[4]), int(row[5]), int(row[6]))
        utc = int(row[7])
        horizontal_prec_dilution = int(row[8])
        vertical_prec_dilution = int(row[9])
        alt_ellipsoid = int(row[10])
        ground_speed = int(row[11])
        velocity = utils.NEDCoordinates(int(row[12]), int(row[13]), int(row[14]))
        horizontal_acc = int(row[15])
        vertical_acc = int(row[16])
        velocity_acc = int(row[17])
        heading = int(row[18])
        heading_offset = int(row[19])
        heading_acc = int(row[20])
        satellite_number = int(row[21])
        fix = int(row[22])

        return logs_types.GpsLog(
            id,
            timestamp,
            device_id,
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
        device_id = int(row[3])
        pressure = int(row[4])
        temperature = int(row[5])

        return logs_types.BaroLog(id, timestamp, device_id, pressure, temperature)

    def __parse_state_log(self, row: list[str]) -> logs_types.EkfState:
        id = int(row[0])
        timestamp = int(row[2])
        attitude = Rotation.from_quat([float(row[3]), float(row[4]), float(row[5]), float(row[6])])
        gyroscope_bias = utils.Vector3(float(row[7]), float(row[8]), float(row[9]))
        velocity = utils.Vector3(float(row[10]), float(row[11]), float(row[12]))
        accelerometer_bias = utils.Vector3(float(row[13]), float(row[14]), float(row[15]))
        position = utils.NEDCoordinates(float(row[16]), float(row[17]), float(row[18]))

        return logs_types.EkfState(
            id,
            timestamp,
            attitude,
            gyroscope_bias,
            velocity,
            accelerometer_bias,
            position
        )
