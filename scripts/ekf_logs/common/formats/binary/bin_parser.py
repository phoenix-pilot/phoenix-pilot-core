from io import BufferedReader
from struct import Struct
from common.models import logs_types, utils

from scipy.spatial.transform import Rotation

import common.formats.binary.structs as structs
import common.formats.binary.specifiers as specifiers


class BinaryLogParser:
    def parse(self, file_path: str) -> list[logs_types.LogEntry]:
        result = []

        with open(file_path, "rb") as file:
            while True:
                prefix = self.__parse_struct(file, structs.LOG_PREFIX)
                if prefix is None:
                    break

                log_id, log_type, timestamp = prefix
                log_type = log_type.decode('ascii')

                if log_type == specifiers.TIME_LOG:
                    result.append(self.__parse_time_log(log_id, timestamp))
                elif log_type == specifiers.IMU_LOG:
                    result.append(self.__parse_imu_log(log_id, timestamp, file))
                elif log_type == specifiers.GPS_LOG:
                    result.append(self.__parse_gps_log(log_id, timestamp, file))
                elif log_type == specifiers.BARO_LOG:
                    result.append(self.__parse_baro_log(log_id, timestamp, file))
                elif log_type == specifiers.STATE_LOG:
                    result.append(self.__parse_state_log(log_id, timestamp, file))
                else:
                    print("Unknown entry in log file")
                    print(f"Last valid item: {result[len(result) - 1].id}")
                    raise Exception()

        return result

    def __parse_time_log(self, log_id: int, timestamp: int) -> logs_types.TimeLog:
        return logs_types.TimeLog(log_id, timestamp)

    def __parse_imu_log(self, log_id: int, timestamp: int, file: BufferedReader) -> logs_types.ImuLog:
        fields = self.__parse_struct(file, structs.IMU)

        return logs_types.ImuLog(
            log_id=log_id,
            timestamp=timestamp,
            accel_device_id=fields[0],
            accel=utils.Vector3(fields[1], fields[2], fields[3]),
            accel_temp=fields[4],
            gyro_device_id=fields[5],
            gyro=utils.Vector3(fields[6], fields[7], fields[8]),
            gyro_d_angle=utils.Vector3(fields[9], fields[10], fields[11]),
            gyro_temp=fields[12],
            mag_dev_id=fields[13],
            mag=utils.Vector3(fields[14], fields[15], fields[16])
        )

    def __parse_gps_log(self, log_id: int, timestamp: int, file: BufferedReader) -> logs_types.GpsLog:
        fields = self.__parse_struct(file, structs.GPS)

        return logs_types.GpsLog(
            log_id=log_id,
            timestamp=timestamp,
            device_id=fields[0],
            position=utils.GlobalPosition(fields[1], fields[2], fields[3]),
            utc=fields[4],
            horizontal_dilution=fields[5],
            vertical_dilution=fields[6],
            alt_ellipsoid=fields[7],
            ground_speed=fields[8],
            velocity=utils.NEDCoordinates(fields[9], fields[10], fields[11]),
            horizontal_accuracy=fields[12],
            vertical_accuracy=fields[13],
            velocity_accuracy=fields[14],
            heading=fields[15],
            heading_offset=fields[16],
            heading_accuracy=fields[17],
            satellite_number=fields[18],
            fix=fields[19]
        )

    def __parse_baro_log(self, log_id, timestamp: int, file: BufferedReader) -> logs_types.BaroLog:
        fields = self.__parse_struct(file, structs.BARO)

        return logs_types.BaroLog(
            log_id=log_id,
            timestamp=timestamp,
            device_ID=fields[0],
            pressure=fields[1],
            temperature=fields[2]
        )

    def __parse_state_log(self, log_id, timestamp: int, file: BufferedReader) -> logs_types.EkfState:
        fields = self.__parse_struct(file, structs.STATE)

        return logs_types.EkfState(
            log_id=log_id,
            timestamp=timestamp,
            attitude=Rotation.from_quat([fields[0], fields[1], fields[2], fields[3]]),
            gyroscope_bias=utils.Vector3(fields[4], fields[5], fields[6]),
            velocity=utils.Vector3(fields[7], fields[8], fields[9]),
            accelerometer_Bias=utils.Vector3(fields[10], fields[11], fields[12]),
            position=utils.NEDCoordinates(fields[13], fields[14], fields[15])
        )

    def __parse_struct(self, file: BufferedReader, struct: Struct):
        data = file.read(struct.size)

        if data is None or len(data) == 0:
            return None

        if len(data) != struct.size:
            raise Exception("Invalid file")

        return struct.unpack(data)
