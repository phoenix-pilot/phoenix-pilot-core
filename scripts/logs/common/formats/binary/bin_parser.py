from typing import Literal, Dict
import common.models.log_reading as logs_types
import common.models.utils as utils
from common.formats.binary.utils import FieldSpecifier, FieldType
from io import BufferedReader


class BinaryLogParser:
    def __init__(
            self,
            fields_specifiers: Dict[str, FieldSpecifier],
            byte_order: Literal['little', 'big'] = 'little'
    ) -> None:
        self.byteOrder = byte_order
        self.fields_specifiers = fields_specifiers

    def parse(self, file_path: str) -> list[logs_types.LogReading]:
        result = []

        with open(file_path, "rb") as file:
            while True:
                log_id = self.__parse_field(file, self.fields_specifiers["id"])

                if not log_id:
                    break

                try:
                    log_type = self.__parse_field(file, self.fields_specifiers["type"])
                except Exception as e:
                    print("Unknown entry in log file")
                    print(f"Last item: {result[len(result) - 1].id}")
                    raise e


                if log_type == "T":
                    result.append(self.__parse_time_log(log_id, file))
                elif log_type == "I":
                    result.append(self.__parse_imu_log(log_id, file))
                elif log_type == "P":
                    result.append(self.__parse_gps_log(log_id, file))
                elif log_type == "B":
                    result.append(self.__parse_baro_log(log_id, file))
                else:
                    print("Unknown entry in log file")
                    print(f"Last item: {result[len(result) - 1].id}")
                    raise Exception()

        return result

    def __parse_time_log(self, log_id: int, file: BufferedReader) -> logs_types.TimeLog:
        timestamp = self.__parse_field(file, self.fields_specifiers["timestamp"])
        return logs_types.TimeLog(log_id, timestamp)

    def __parse_imu_log(self, log_id: int, file: BufferedReader) -> logs_types.ImuLog:
        timestamp = self.__parse_field(file, self.fields_specifiers["timestamp"])

        accel_vector = self.__parse_vector3(file, self.fields_specifiers["acceleration"])
        gyro_vector = self.__parse_vector3(file, self.fields_specifiers["gyro"])
        d_angle_vector = self.__parse_vector3(file, self.fields_specifiers["dAngle"])
        mag_vector = self.__parse_vector3(file, self.fields_specifiers["magnetometer"])

        return logs_types.ImuLog(log_id, timestamp, accel_vector, gyro_vector, d_angle_vector, mag_vector)

    def __parse_gps_log(self, log_id: int, file: BufferedReader) -> logs_types.GpsLog:
        timestamp = self.__parse_field(file, self.fields_specifiers["timestamp"])

        position = self.__parse_global_position(file)

        horizontal_acc = self.__parse_field(file, self.fields_specifiers["horizontal_accuracy"])
        velocity_acc = self.__parse_field(file, self.fields_specifiers["velocity_accuracy"])

        fix = self.__parse_field(file, self.fields_specifiers["fix"])

        satellite_nb = self.__parse_field(file, self.fields_specifiers["satellite_number"])

        velocity = self.__parse_ned(file, self.fields_specifiers["velocity"])

        return logs_types.GpsLog(log_id, timestamp, position, horizontal_acc, velocity_acc, satellite_nb, fix, velocity)

    def __parse_baro_log(self, log_id, file: BufferedReader) -> logs_types.BaroLog:
        timestamp = self.__parse_field(file, self.fields_specifiers["timestamp"])

        pressure = self.__parse_field(file, self.fields_specifiers["pressure"])
        temperature = self.__parse_field(file, self.fields_specifiers["temperature"])

        return logs_types.BaroLog(log_id, timestamp, pressure, temperature)

    def __parse_global_position(self, file: BufferedReader):
        return utils.GlobalPosition(
            latitude=self.__parse_field(file, self.fields_specifiers["latitude"]),
            longitude=self.__parse_field(file, self.fields_specifiers["longitude"]),
            altitude=self.__parse_field(file, self.fields_specifiers["altitude"])
        )

    def __parse_ned(self, file: BufferedReader, field: FieldSpecifier) -> utils.NEDCoordinates:
        return utils.NEDCoordinates(
            north=self.__parse_field(file, field),
            east=self.__parse_field(file, field),
            down=self.__parse_field(file, field)
        )

    def __parse_vector3(self, file: BufferedReader, field: FieldSpecifier) -> utils.Vector3:
        return utils.Vector3(
            x=self.__parse_field(file, field),
            y=self.__parse_field(file, field),
            z=self.__parse_field(file, field)
        )

    def __parse_field(self, file: BufferedReader, field: FieldSpecifier):
        if field.type == FieldType.INT:
            bytes = file.read(field.size)
            return int.from_bytes(bytes, byteorder=self.byteOrder, signed=field.signed)
        elif field.type == FieldType.CHAR:
            return file.read(field.size).decode("utf-8")
        else:
            raise Exception("Unknown field type")
