import common.models.log_reading as logs_types
from common.logs_file_formats.file_format import FileFormat
from common.logs_file_formats.csv_file.csv_file_export import CsvLogExporter
from common.logs_file_formats.csv_file.csv_file_parser import CsvLogParser


class CsvFile(FileFormat):
    def export_logs(self, file_path: str, logs: list[logs_types.LogReading]) -> None:
        exporter = CsvLogExporter()
        exporter.export(file_path, logs)


    def import_logs(self, file_path: str) -> list[logs_types.LogReading]:
        parser = CsvLogParser()
        return parser.parse(file_path)
