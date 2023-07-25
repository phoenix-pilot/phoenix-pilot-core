import os
import abc
from typing import Literal

import common.models.log_reading as logs_types
from common.formats.binary.utils import FieldType, FieldSpecifier

from common.formats.binary.bin_export import BinaryLogExporter
from common.formats.binary.bin_parser import BinaryLogParser

from common.formats.csv.csv_export import CsvLogExporter
from common.formats.csv.csv_parser import CsvLogParser


class FileFormat(abc.ABC):
    @abc.abstractmethod
    def import_logs(self, file_path: str) -> list[logs_types.LogReading]:
        raise NotImplementedError

    @abc.abstractmethod
    def export_logs(self, file_path: str, logs: list[logs_types.LogReading]) -> None:
        raise NotImplementedError


class FormatFactory:
    @staticmethod
    def from_path(path: str):
        _, file_format = os.path.splitext(path)

        if file_format == ".bin":
            return Binary()
        elif file_format == ".csv":
            return Csv()


class Binary(FileFormat):
    FIELDS = {
        "id": FieldSpecifier(size=4, signed=False),
        "type": FieldSpecifier(size=1, type=FieldType.CHAR),
        "timestamp": FieldSpecifier(size=8, signed=False),

        # IMU fields
        "acceleration": FieldSpecifier(size=4),
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

    def __init__(self, byte_order: Literal['little', 'big'] = 'little') -> None:
        self.byteOrder = byte_order

    def export_logs(self, file_path: str, logs: list[logs_types.LogReading]) -> None:
        exporter = BinaryLogExporter(Binary.FIELDS, self.byteOrder)
        exporter.export(file_path, logs)

    def import_logs(self, file_path: str) -> list[logs_types.LogReading]:
        parser = BinaryLogParser(Binary.FIELDS, self.byteOrder)
        return parser.parse(file_path)


class Csv(FileFormat):
    def export_logs(self, file_path: str, logs: list[logs_types.LogReading]) -> None:
        exporter = CsvLogExporter()
        exporter.export(file_path, logs)

    def import_logs(self, file_path: str) -> list[logs_types.LogReading]:
        parser = CsvLogParser()
        return parser.parse(file_path)

