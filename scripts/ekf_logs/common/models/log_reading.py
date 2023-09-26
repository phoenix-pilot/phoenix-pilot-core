from abc import ABC, abstractmethod

from common.models.log_data import Imu, Gps, Baro, EkfState

from common.models.visitor import LogsVisitor


class LogEntry(ABC):
    def __init__(self, log_id, timestamp):
        self.id = log_id
        self.timestamp = timestamp

    @abstractmethod
    def accept(self, visitor: LogsVisitor):
        raise NotImplementedError


class TimeLog(LogEntry):
    def __init__(self, log_id, timestamp):
        super().__init__(log_id, timestamp)

    def accept(self, visitor: LogsVisitor):
        visitor.visit_time_log(self)


class ImuLog(LogEntry):
    def __init__(self, log_id, timestamp, imu_data: Imu):
        super().__init__(log_id, timestamp)
        self.data = imu_data

    def accept(self, visitor: LogsVisitor):
        visitor.visit_imu_log(self)


class GpsLog(LogEntry):
    def __init__(self, log_id, timestamp, gps_data: Gps):
        super().__init__(log_id, timestamp)
        self.data = gps_data

    def accept(self, visitor: LogsVisitor):
        visitor.visit_gps_log(self)


class BaroLog(LogEntry):
    def __init__(self, log_id, timestamp, baro_data: Baro):
        super().__init__(log_id, timestamp)
        self.data = baro_data

    def accept(self, visitor: LogsVisitor):
        visitor.visit_baro_log(self)


class EkfStateLog(LogEntry):
    def __init__(self, log_id, timestamp, ekf_state: EkfState):
        super().__init__(log_id, timestamp)
        self.data = ekf_state

    def accept(self, visitor: LogsVisitor):
        visitor.visit_ekf_state_log(self)
