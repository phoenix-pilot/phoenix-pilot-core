import common.models.log_reading as logs_types
from common.logs_file_formats.binary_file.bin_log_parser import BinaryLogParser
from common.logs_file_formats.binary_file.bin_log_export import BinaryLogExporter
from common.logs_file_formats.binary_file.utils import FieldSpecifier, FieldType
from common.logs_file_formats.file_format import FileFormat
from typing import Literal



class BinaryFile(FileFormat):
    FIELDS = {
        "id": FieldSpecifier(size=4, signed=False),
        "type": FieldSpecifier(size=1, type=FieldType.CHAR),
        "timestamp": FieldSpecifier(size=8, signed=False),

        # IMU fields
        "acceleration": FieldSpecifier(size=2),
        "gyro": FieldSpecifier(size=4),
        "dAngle": FieldSpecifier(size=4, signed=False),
        "magnetometer": FieldSpecifier(size=2),

        # Gps fields
        "latitude": FieldSpecifier(size=8),
        "longitude": FieldSpecifier(size=8),
        "altitude": FieldSpecifier(size=4),
        "horizontal_accuracy": FieldSpecifier(size=4, signed=False),
        "velocity_accuracy": FieldSpecifier(size=4, signed=False),
        "fix": FieldSpecifier(size=1, signed=False),
        "satellite_number": FieldSpecifier(size=1, signed=False),
        "velocity": FieldSpecifier(size=4),

        # Barometer fields
        "pressure": FieldSpecifier(size=4),
        "temperature": FieldSpecifier(size=4, signed=False)
    }


    def __init__(self, byteOrder: Literal['little', 'big'] = 'little') -> None:
        self.byteOrder = byteOrder


    def export_logs(self, file_path: str, logs: list[logs_types.LogReading]) -> None:
        exporter = BinaryLogExporter(BinaryFile.FIELDS, self.byteOrder)
        exporter.export(file_path, logs)


    def import_logs(self, file_path: str) -> list[logs_types.LogReading]:
        parser = BinaryLogParser(BinaryFile.FIELDS, self.byteOrder)
        return parser.parse(file_path)
