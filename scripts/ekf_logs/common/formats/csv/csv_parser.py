import csv
from common.models import logs_types, logs_data, utils

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

        imu_data = logs_data.Imu(
            accelDeviceID=int(row[3]),
            accel=utils.Vector3(int(row[4]), int(row[5]), int(row[6])),
            accelTemp=int(row[7]),

            gyroDeviceID=int(row[8]),
            gyro=utils.Vector3(int(row[9]), int(row[10]), int(row[11])),
            gyroDAngle=utils.Vector3(int(row[12]), int(row[13]), int(row[14])),
            gyroTemp=int(row[15]),

            magDeviceID=int(row[16]),
            mag=utils.Vector3(int(row[17]), int(row[18]), int(row[19]))
        )

        return logs_types.ImuLog(id, timestamp, imu_data)

    def __parse_gps_log(self, row: list[str]) -> logs_types.GpsLog:
        id = int(row[0])
        timestamp = int(row[2])

        gps_data = logs_data.Gps(
            deviceID=int(row[3]),
            position=utils.GlobalPosition(int(row[4]), int(row[5]), int(row[6])),
            utc=int(row[7]),
            horizontalPrecisionDilution=int(row[8]),
            verticalPrecisionDilution=int(row[9]),
            ellipsoidAlt=int(row[10]),
            groundSpeed=int(row[11]),
            velocity=utils.NEDCoordinates(int(row[12]), int(row[13]), int(row[14])),
            horizontalAccuracy=int(row[15]),
            verticalAccuracy=int(row[16]),
            velocityAccuracy=int(row[17]),
            heading=int(row[18]),
            headingOffset=int(row[19]),
            headingAccuracy=int(row[20]),
            satelliteNumber=int(row[21]),
            fix=int(row[22])
        )

        return logs_types.GpsLog(id, timestamp, gps_data)

    def __parse_time_log(self, row: list[str]) -> logs_types.TimeLog:
        id = int(row[0])
        timestamp = int(row[2])

        return logs_types.TimeLog(id, timestamp)

    def __parse_baro_log(self, row: list[str]) -> logs_types.BaroLog:
        id = int(row[0])
        timestamp = int(row[2])

        baro_data = logs_data.Baro(
            deviceID=int(row[3]),
            pressure=int(row[4]),
            temperature=int(row[5])
        )

        return logs_types.BaroLog(id, timestamp, baro_data)

    def __parse_state_log(self, row: list[str]) -> logs_types.EkfStateLog:
        id = int(row[0])
        timestamp = int(row[2])

        ekf_state_data = logs_data.EkfState(
            # Scipy uses scalar-last quaternion format (x, y, z, w)
            attitude=Rotation.from_quat([float(row[4]), float(row[5]), float(row[6]), float(row[3])]),
            gyroscopeBias=utils.Vector3(float(row[7]), float(row[8]), float(row[9])),
            velocity=utils.Vector3(float(row[10]), float(row[11]), float(row[12])),
            accelerometerBias=utils.Vector3(float(row[13]), float(row[14]), float(row[15])),
            position=utils.NEDCoordinates(float(row[16]), float(row[17]), float(row[18]))
        )

        return logs_types.EkfStateLog(id, timestamp, ekf_state_data)
