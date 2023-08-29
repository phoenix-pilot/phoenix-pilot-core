from typing import Literal
from io import BufferedReader
from common.models import logs_types, utils
from common.formats.binary.utils import BinaryField, FieldType
from common.formats.binary.fields import FIELDS


class BinaryLogParser:
    def __init__(
            self,
            byte_order: Literal['little', 'big'] = 'little'
    ) -> None:
        self.byteOrder = byte_order

    def parse(self, file_path: str) -> list[logs_types.LogEntry]:
        result = []

        with open(file_path, "rb") as file:
            while True:
                log_id = self.__parse_field(file, FIELDS["id"])

                if not log_id:
                    break

                try:
                    log_type = self.__parse_field(file, FIELDS["type"])
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
        timestamp = self.__parse_field(file, FIELDS["timestamp"])
        return logs_types.TimeLog(log_id, timestamp)

    def __parse_imu_log(self, log_id: int, file: BufferedReader) -> logs_types.ImuLog:
        timestamp = self.__parse_field(file, FIELDS["timestamp"])

        accel_vector = self.__parse_vector3(file, FIELDS["acceleration"])
        gyro_vector = self.__parse_vector3(file, FIELDS["gyro"])
        d_angle_vector = self.__parse_vector3(file, FIELDS["dAngle"])
        mag_vector = self.__parse_vector3(file, FIELDS["magnetometer"])

        return logs_types.ImuLog(log_id, timestamp, accel_vector, gyro_vector, d_angle_vector, mag_vector)

    def __parse_gps_log(self, log_id: int, file: BufferedReader) -> logs_types.GpsLog:
        timestamp = self.__parse_field(file, FIELDS["timestamp"])

        position = self.__parse_global_position(file)

        utc = self.__parse_field(file, FIELDS["utc"])

        horizontal_dilution = self.__parse_field(file, FIELDS["precision_dilution"])
        vertical_dilution = self.__parse_field(file, FIELDS["precision_dilution"])

        ellipsoid_altitude = self.__parse_field(file, FIELDS["ellipsoid_altitude"])

        ground_speed = self.__parse_field(file, FIELDS["ground_speed"])

        velocity = self.__parse_ned(file, FIELDS["velocity"])

        horizontal_accuracy = self.__parse_field(file, FIELDS["position_accuracy"])
        vertical_accuracy = self.__parse_field(file, FIELDS["position_accuracy"])
        velocity_accuracy = self.__parse_field(file, FIELDS["position_accuracy"])

        heading = self.__parse_field(file, FIELDS["heading"])
        heading_offset = self.__parse_field(file, FIELDS["heading_offset"])
        heading_accuracy = self.__parse_field(file, FIELDS["heading_accuracy"])

        satellite_nb = self.__parse_field(file, FIELDS["satellite_number"])

        fix = self.__parse_field(file, FIELDS["fix"])

        return logs_types.GpsLog(
            log_id,
            timestamp,
            position,
            utc,
            horizontal_dilution,
            vertical_dilution,
            ellipsoid_altitude,
            ground_speed,
            velocity,
            horizontal_accuracy,
            vertical_accuracy,
            velocity_accuracy,
            heading,
            heading_offset,
            heading_accuracy,
            satellite_nb,
            fix,
        )

    def __parse_baro_log(self, log_id, file: BufferedReader) -> logs_types.BaroLog:
        timestamp = self.__parse_field(file, FIELDS["timestamp"])

        pressure = self.__parse_field(file, FIELDS["pressure"])
        temperature = self.__parse_field(file, FIELDS["temperature"])

        return logs_types.BaroLog(log_id, timestamp, pressure, temperature)

    def __parse_global_position(self, file: BufferedReader):
        return utils.GlobalPosition(
            latitude=self.__parse_field(file, FIELDS["latitude"]),
            longitude=self.__parse_field(file, FIELDS["longitude"]),
            altitude=self.__parse_field(file, FIELDS["altitude"])
        )

    def __parse_ned(self, file: BufferedReader, field: BinaryField) -> utils.NEDCoordinates:
        return utils.NEDCoordinates(
            north=self.__parse_field(file, field),
            east=self.__parse_field(file, field),
            down=self.__parse_field(file, field)
        )

    def __parse_vector3(self, file: BufferedReader, field: BinaryField) -> utils.Vector3:
        return utils.Vector3(
            x=self.__parse_field(file, field),
            y=self.__parse_field(file, field),
            z=self.__parse_field(file, field)
        )

    def __parse_field(self, file: BufferedReader, field: BinaryField):
        if field.type == FieldType.INT:
            bytes = file.read(field.size)
            return int.from_bytes(bytes, byteorder=self.byteOrder, signed=field.signed)
        elif field.type == FieldType.CHAR:
            return file.read(field.size).decode("utf-8")
        else:
            raise Exception("Unknown field type")
