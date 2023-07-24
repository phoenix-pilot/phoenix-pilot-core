from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import common.logs_file_formats.binary_file.binary_file as bin_file

from typing import Literal, Dict
import common.models.log_reading as logs_types
import common.models.utils as utils
from common.models.visitor import LogsVisitor
from common.logs_file_formats.binary_file.utils import FieldSpecifier, FieldType


class BinaryLogExporter(LogsVisitor):
    def __init__(
            self,
            fields_specifiers: Dict[str, FieldSpecifier],
            byteOrder: Literal['little', 'big'] = 'little'
        ) -> None:
        self.byte_order = byteOrder
        self.fields_specifiers = fields_specifiers
        self.file = None


    def export(self, file_path, logs: list[logs_types.LogReading]) -> None:
        try:
            self.file = open(file_path, "wb")
            for log in logs:
                log.accept(self)
        finally:
            if self.file is not None:
                self.file.close()
                self.file = None


    def visit_time_log(self, timeLog: logs_types.TimeLog):
        self.__write_prefix(timeLog, "T")


    def visit_imu_log(self, imuLog: logs_types.ImuLog):
        self.__write_prefix(imuLog, "I")
        self.__write_vector(imuLog.accel, self.fields_specifiers["acceleration"])
        self.__write_vector(imuLog.gyro, self.fields_specifiers["gyro"])
        self.__write_vector(imuLog.gyroDAngle, self.fields_specifiers["dAngle"])
        self.__write_vector(imuLog.mag, self.fields_specifiers["magnetometer"])


    def visit_gps_log(self, gpsLog: logs_types.GpsLog):
        self.__write_prefix(gpsLog, "P")
        self.__write_global_position(gpsLog.position)
        self.__write_field(gpsLog.horizontalAccuracy, self.fields_specifiers["horizontal_accuracy"])
        self.__write_field(gpsLog.velocityAccuracy, self.fields_specifiers["velocity_accuracy"])
        self.__write_field(gpsLog.fix, self.fields_specifiers["fix"])
        self.__write_field(gpsLog.satelliteNumber, self.fields_specifiers["satellite_number"])
        self.__write_ned(gpsLog.velocity, self.fields_specifiers["velocity"])


    def visit_baro_log(self, baroLog: logs_types.BaroLog):
        self.__write_prefix(baroLog, "B")
        self.__write_field(baroLog.pressure, self.fields_specifiers["pressure"])
        self.__write_field(baroLog.temperature, self.fields_specifiers["temperature"])


    def __write_prefix(self, log: logs_types.LogReading, log_type: str):
        self.__write_field(log.id, self.fields_specifiers["id"])
        self.__write_field(log_type, self.fields_specifiers["type"])
        self.__write_field(log.timestamp, self.fields_specifiers["timestamp"])


    def __write_ned(self, ned: utils.NEDCoordinates, field_specifier: FieldSpecifier):
        self.__write_field(ned.north, field_specifier)
        self.__write_field(ned.east, field_specifier)
        self.__write_field(ned.down, field_specifier)


    def __write_global_position(self, position: utils.GlobalPosition):
        self.__write_field(position.latitude, self.fields_specifiers["latitude"])
        self.__write_field(position.longitude, self.fields_specifiers["longitude"])
        self.__write_field(position.altitude, self.fields_specifiers["altitude"])


    def __write_vector(self, vector: utils.Vector3, field_specifier: FieldSpecifier):
        self.__write_field(vector.x, field_specifier)
        self.__write_field(vector.y, field_specifier)
        self.__write_field(vector.z, field_specifier)


    def __write_field(self, data, field_specifier: FieldSpecifier):
        if field_specifier.type == FieldType.INT:
            bytes = int.to_bytes(
                data,
                field_specifier.size,
                byteorder=self.byte_order,
                signed=field_specifier.signed
            )
        elif field_specifier.type == FieldType.CHAR:
            bytes = str.encode(data)

        self.file.write(bytes)
