from io import BufferedReader
from struct import Struct
from common.models import logs_types, log_data, utils

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

        imu_data = log_data.Imu(
            accelDeviceID=fields[0],
            accel=utils.Vector3(fields[1], fields[2], fields[3]),
            accelTemp=fields[4],

            gyroDeviceID=fields[5],
            gyro=utils.Vector3(fields[6], fields[7], fields[8]),
            gyroDAngle=utils.Vector3(fields[9], fields[10], fields[11]),
            gyroTemp=fields[12],

            magDeviceID=fields[13],
            mag=utils.Vector3(fields[14], fields[15], fields[16])
        )

        return logs_types.ImuLog(log_id, timestamp, imu_data)

    def __parse_gps_log(self, log_id: int, timestamp: int, file: BufferedReader) -> logs_types.GpsLog:
        fields = self.__parse_struct(file, structs.GPS)

        gps_data = log_data.Gps(
            deviceID=fields[0],
            position=utils.GlobalPosition(fields[1], fields[2], fields[3]),
            utc=fields[4],
            horizontalPrecisionDilution=fields[5],
            verticalPrecisionDilution=fields[6],
            ellipsoidAlt=fields[7],
            groundSpeed=fields[8],
            velocity=utils.NEDCoordinates(fields[9], fields[10], fields[11]),
            horizontalAccuracy=fields[12],
            verticalAccuracy=fields[13],
            velocityAccuracy=fields[14],
            heading=fields[15],
            headingOffset=fields[16],
            headingAccuracy=fields[17],
            satelliteNumber=fields[18],
            fix=fields[19]
        )

        return logs_types.GpsLog(log_id, timestamp, gps_data)

    def __parse_baro_log(self, log_id, timestamp: int, file: BufferedReader) -> logs_types.BaroLog:
        fields = self.__parse_struct(file, structs.BARO)

        baro_data = log_data.Baro(
            deviceID=fields[0],
            pressure=fields[1],
            temperature=fields[2]
        )

        return logs_types.BaroLog(log_id, timestamp, baro_data)

    def __parse_state_log(self, log_id, timestamp: int, file: BufferedReader) -> logs_types.EkfStateLog:
        fields = self.__parse_struct(file, structs.STATE)

        ekf_state = log_data.EkfState(
            # Scipy uses scalar-last quaternion format (x, y, z, w)
            attitude=Rotation.from_quat([fields[1], fields[2], fields[3], fields[0]]),
            gyroscopeBias=utils.Vector3(fields[4], fields[5], fields[6]),
            velocity=utils.NEDCoordinates(fields[7], fields[8], fields[9]),
            accelerometerBias=utils.Vector3(fields[10], fields[11], fields[12]),
            position=utils.NEDCoordinates(fields[13], fields[14], fields[15]),
            magneticField=utils.Vector3(fields[16], fields[17], fields[18])
        )

        return logs_types.EkfStateLog(log_id, timestamp, ekf_state)

    def __parse_struct(self, file: BufferedReader, struct: Struct):
        data = file.read(struct.size)

        if data is None or len(data) == 0:
            return None

        if len(data) != struct.size:
            raise Exception("Invalid file")

        return struct.unpack(data)
