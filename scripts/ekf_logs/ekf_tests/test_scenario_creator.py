from common.models import logs_types, logs_data
from common.formats import FormatFactory

from collections import deque
from typing import Iterator
import itertools


class LogsGenerator:
    def __init__(
        self,
        imu_data_iter: Iterator[logs_data.Imu],
        gps_data_iter: Iterator[logs_data.Gps],
        baro_data_iter: Iterator[logs_data.Baro],

        time_log_dt: int = 2_000,
        imu_log_dt: int = 2_000,
        baro_log_dt: int = 40_000,
        gps_log_dt: int = 200_000,

        init_timestamp: int = 130_000_000
    ):
        self.logsIDsGenerator = itertools.count(1)

        # Timestamps iterators
        self.timeLogsTimestamp = itertools.count(init_timestamp, time_log_dt)
        self.imuLogsTimestamp = itertools.count(init_timestamp, imu_log_dt)
        self.gpsLogsTimestamp = itertools.count(init_timestamp, gps_log_dt)
        self.baroLogsTimestamp = itertools.count(init_timestamp, baro_log_dt)

        self.imuDataGenerator = imu_data_iter
        self.gpsDataGenerator = gps_data_iter
        self.baroDataGenerator = baro_data_iter

    def time_logs(self) -> Iterator[logs_types.TimeLog]:
        while True:
            yield logs_types.TimeLog(
                log_id=next(self.logsIDsGenerator),
                timestamp=next(self.timeLogsTimestamp),
            )

    def imu_logs(self) -> Iterator[logs_types.ImuLog]:
        while True:
            yield logs_types.ImuLog(
                log_id=next(self.logsIDsGenerator),
                timestamp=next(self.imuLogsTimestamp),
                imu_data=next(self.imuDataGenerator)
            )

    def gps_logs(self) -> Iterator[logs_types.GpsLog]:
        while True:
            timestamp = next(self.gpsLogsTimestamp)

            gps_data = next(self.gpsDataGenerator)
            gps_data.utc = timestamp

            yield logs_types.GpsLog(
                log_id=next(self.logsIDsGenerator),
                timestamp=timestamp,
                gps_data=gps_data
            )

    def baro_logs(self) -> Iterator[logs_types.BaroLog]:
        while True:
            yield logs_types.BaroLog(
                log_id=next(self.logsIDsGenerator),
                timestamp=next(self.baroLogsTimestamp),
                baro_data=next(self.baroDataGenerator)
            )


class TestScenarioCreator:
    def create(self, generator: LogsGenerator, result_file: str, scenario_timespan: int):
        logs = deque([])

        self.__add_logs(logs, generator.time_logs(), scenario_timespan)
        self.__add_logs(logs, generator.imu_logs(), scenario_timespan)
        self.__add_logs(logs, generator.baro_logs(), scenario_timespan)
        self.__add_logs(logs, generator.gps_logs(), scenario_timespan)

        format = FormatFactory.from_path(result_file)
        format.export_logs(result_file, logs)

    @staticmethod
    def __add_logs(logs: deque, logs_iterator: Iterator[logs_types.LogEntry], timespan: int):
        first_log = next(logs_iterator)
        logs.append(first_log)
        end_time = first_log.timestamp + timespan

        while True:
            next_log = next(logs_iterator)
            logs.append(next_log)

            if end_time < next_log.timestamp:
                break
