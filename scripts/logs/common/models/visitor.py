from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import common.models.log_reading as logs_types

import abc


class LogsVisitor(metaclass=abc.ABCMeta):
    @classmethod
    def __subclasshook__(cls, subclass):
        return (hasattr(subclass, 'visit_time_log') and
                callable(subclass.visit_time_log) and
                hasattr(subclass, 'visit_time_log') and
                callable(subclass.visit_imu_log) and
                hasattr(subclass, 'visit_imu_log') and
                callable(subclass.visit_gps_log) and
                hasattr(subclass, 'visit_gps_log') and
                callable(subclass.visit_time_log) or
                NotImplemented)


    @abc.abstractmethod
    def visit_time_log(self, timeLog: logs_types.TimeLog):
        raise NotImplementedError


    @abc.abstractmethod
    def visit_imu_log(self, imuLog: logs_types.ImuLog):
        raise NotImplementedError


    @abc.abstractmethod
    def visit_gps_log(self, gpsLog: logs_types.GpsLog):
        raise NotImplementedError


    @abc.abstractmethod
    def visit_baro_log(self, baroLog: logs_types.BaroLog):
        raise NotImplementedError
