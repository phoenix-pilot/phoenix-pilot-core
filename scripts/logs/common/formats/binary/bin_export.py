from typing import Literal
from common.models import LogsVisitor, logs_types, utils
from common.formats.binary.utils import BinaryField, FieldType
from common.formats.binary.fields import FIELDS


class BinaryLogExporter(LogsVisitor):
    def __init__(
            self,
            byte_order: Literal['little', 'big'] = 'little'
    ) -> None:
        self.byte_order = byte_order
        self.file = None

    def export(self, file_path, logs: list[logs_types.LogEntry]) -> None:
        try:
            self.file = open(file_path, "wb")
            for log in logs:
                log.accept(self)
        finally:
            if self.file is not None:
                self.file.close()
                self.file = None

    def visit_time_log(self, time_log: logs_types.TimeLog):
        self.__write_prefix(time_log, "T")

    def visit_imu_log(self, imu_log: logs_types.ImuLog):
        self.__write_prefix(imu_log, "I")
        self.__write_vector(imu_log.accel, FIELDS["acceleration"])
        self.__write_vector(imu_log.gyro, FIELDS["gyro"])
        self.__write_vector(imu_log.gyroDAngle, FIELDS["dAngle"])
        self.__write_vector(imu_log.mag, FIELDS["magnetometer"])

    def visit_gps_log(self, gps_log: logs_types.GpsLog):
        self.__write_prefix(gps_log, "P")
        self.__write_global_position(gps_log.position)
        self.__write_field(gps_log.horizontalAccuracy, FIELDS["horizontal_accuracy"])
        self.__write_field(gps_log.velocityAccuracy, FIELDS["velocity_accuracy"])
        self.__write_field(gps_log.fix, FIELDS["fix"])
        self.__write_field(gps_log.satelliteNumber, FIELDS["satellite_number"])
        self.__write_ned(gps_log.velocity, FIELDS["velocity"])

    def visit_baro_log(self, baro_log: logs_types.BaroLog):
        self.__write_prefix(baro_log, "B")
        self.__write_field(baro_log.pressure, FIELDS["pressure"])
        self.__write_field(baro_log.temperature, FIELDS["temperature"])

    def __write_prefix(self, log: logs_types.LogEntry, log_type: str):
        self.__write_field(log.id, FIELDS["id"])
        self.__write_field(log_type, FIELDS["type"])
        self.__write_field(log.timestamp, FIELDS["timestamp"])

    def __write_ned(self, ned: utils.NEDCoordinates, field_specifier: BinaryField):
        self.__write_field(ned.north, field_specifier)
        self.__write_field(ned.east, field_specifier)
        self.__write_field(ned.down, field_specifier)

    def __write_global_position(self, position: utils.GlobalPosition):
        self.__write_field(position.latitude, FIELDS["latitude"])
        self.__write_field(position.longitude, FIELDS["longitude"])
        self.__write_field(position.altitude, FIELDS["altitude"])

    def __write_vector(self, vector: utils.Vector3, field_specifier: BinaryField):
        self.__write_field(vector.x, field_specifier)
        self.__write_field(vector.y, field_specifier)
        self.__write_field(vector.z, field_specifier)

    def __write_field(self, data, field_specifier: BinaryField):
        if field_specifier.type == FieldType.INT:
            bytes = int.to_bytes(
                data,
                field_specifier.size,
                byteorder=self.byte_order,
                signed=field_specifier.signed
            )
        elif field_specifier.type == FieldType.CHAR:
            bytes = str.encode(data)
        else:
            raise Exception("Unknown field type")

        self.file.write(bytes)
