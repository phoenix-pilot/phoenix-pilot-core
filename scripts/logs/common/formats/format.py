import os
import abc
from typing import Literal

from common.models import logs_types

import common.formats.csv as csv
import common.formats.binary as bin


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
        else:
            raise Exception("Unsupported file format")


class Binary(FileFormat):
    def __init__(self, byte_order: Literal['little', 'big'] = 'little') -> None:
        self.byteOrder = byte_order

    def export_logs(self, file_path: str, logs: list[logs_types.LogReading]) -> None:
        exporter = bin.Exporter(self.byteOrder)
        exporter.export(file_path, logs)

    def import_logs(self, file_path: str) -> list[logs_types.LogReading]:
        parser = bin.Parser(self.byteOrder)
        return parser.parse(file_path)


class Csv(FileFormat):
    def export_logs(self, file_path: str, logs: list[logs_types.LogReading]) -> None:
        exporter = csv.Exporter()
        exporter.export(file_path, logs)

    def import_logs(self, file_path: str) -> list[logs_types.LogReading]:
        parser = csv.Parser()
        return parser.parse(file_path)
