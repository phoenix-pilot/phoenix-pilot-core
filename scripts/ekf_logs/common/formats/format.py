import os
import abc

from typing import Iterator

from common.models import logs_types

import common.formats.csv as csv
import common.formats.binary as bin


class FileFormat(abc.ABC):
    @abc.abstractmethod
    def import_logs(self, file_path: str) -> list[logs_types.LogEntry]:
        raise NotImplementedError

    @abc.abstractmethod
    def export_logs(self, file_path: str, logs: Iterator[logs_types.LogEntry]) -> None:
        raise NotImplementedError


class FormatFactory:
    @staticmethod
    def from_path(path: str) -> FileFormat:
        """Returns appropriate file format handler depending on extension of file"""

        _, file_format = os.path.splitext(path)

        if file_format == ".bin":
            return Binary()
        elif file_format == ".csv":
            return Csv()
        else:
            raise ValueError("Unsupported file format")


class Binary(FileFormat):
    def export_logs(self, file_path: str, logs: Iterator[logs_types.LogEntry]) -> None:
        exporter = bin.Exporter()
        exporter.export(file_path, logs)

    def import_logs(self, file_path: str) -> list[logs_types.LogEntry]:
        parser = bin.Parser()
        return parser.parse(file_path)


class Csv(FileFormat):
    def export_logs(self, file_path: str, logs: Iterator[logs_types.LogEntry]) -> None:
        exporter = csv.Exporter()
        exporter.export(file_path, logs)

    def import_logs(self, file_path: str) -> list[logs_types.LogEntry]:
        parser = csv.Parser()
        return parser.parse(file_path)
