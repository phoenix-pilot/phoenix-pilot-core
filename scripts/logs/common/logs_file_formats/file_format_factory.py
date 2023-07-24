import os
from common.logs_file_formats.binary_file.binary_file import BinaryFile
from common.logs_file_formats.csv_file.csv_file import CsvFile


class FileFormatFactory:
    def from_path(path: str):
        _, file_format = os.path.splitext(path)

        if file_format == ".bin":
            return BinaryFile()
        elif file_format == ".csv":
            return CsvFile()
