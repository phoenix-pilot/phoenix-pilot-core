from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import common.models.log_reading as log_types

from bisect import bisect_left
from common.models.visitor import LogsVisitor


class StudyContext:
    def __init__(self, all_logs: list[log_types.LogReading]) -> None:
        self.all_logs: list[log_types.LogEntry] = all_logs

        self.time_logs: list[log_types.TimeLog] = []
        self.gps_logs: list[log_types.GpsLog] = []
        self.baro_logs: list[log_types.BaroLog] = []
        self.imu_logs: list[log_types.ImuLog] = []
        self.state_logs: list[log_types.EkfStateLog] = []

        filler = StudyContext.ContextFiller(self)
        for log in all_logs:
            log.accept(filler)

        self.missed_logs_cnt = self.__calculate_missed()

    def get_index(self, log: log_types.LogReading) -> int:
        """Returns index of log from all logs"""

        i = bisect_left(self.all_logs, log, key=lambda entry: entry.id)
        if i != len(self.all_logs) and self.all_logs[i] == log:
            return i
        raise ValueError

    def get_missed(self, index: int) -> int:
        if index == 0:
            return self.all_logs[index].id - 1
        else:
            return self.all_logs[index].id - self.all_logs[index - 1].id - 1

    def __calculate_missed(self) -> int:
        result = 0

        for i in range(len(self.all_logs)):
            missed = self.get_missed(i)
            if missed > 0:
                result = result + missed

        return result

    class ContextFiller(LogsVisitor):
        def __init__(self, context: StudyContext) -> None:
            self.context = context

        def visit_time_log(self, time_log: log_types.TimeLog):
            self.context.time_logs.append(time_log)

        def visit_imu_log(self, imu_log: log_types.ImuLog):
            self.context.imu_logs.append(imu_log)

        def visit_gps_log(self, gps_log: log_types.GpsLog):
            self.context.gps_logs.append(gps_log)

        def visit_baro_log(self, baro_log: log_types.BaroLog):
            self.context.baro_logs.append(baro_log)

        def visit_ekf_state_log(self, state_log: log_types.EkfStateLog):
            self.context.state_logs.append(state_log)
