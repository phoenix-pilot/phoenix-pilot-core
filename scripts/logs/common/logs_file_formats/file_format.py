import abc
import common.models.log_reading as logs_types

class FileFormat(abc.ABC):
    @abc.abstractmethod
    def import_logs(self, file_path: str) -> list[logs_types.LogReading]:
        raise NotImplementedError

    @abc.abstractmethod
    def export_logs(self, file_path: str, logs: list[logs_types.LogReading]) -> None:
        raise NotImplementedError
