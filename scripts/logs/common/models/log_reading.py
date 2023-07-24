from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from logs_analysis.context import AnalysisContext

from abc import ABC, abstractmethod

from common.models.utils import Vector3, GlobalPosition, NEDCoordinates
from common.models.visitor import LogsVisitor


class LogReading(ABC):
    def __init__(self, log_id, timestamp):
        self.id = log_id
        self.timestamp = timestamp

    @abstractmethod
    def accept(self, visitor: LogsVisitor):
        raise NotImplementedError


class TimeLog(LogReading):
    def __init__(self, log_id, timestamp):
        super().__init__(log_id, timestamp)

    def accept(self, visitor: LogsVisitor):
        visitor.visit_time_log(self)


class ImuLog(LogReading):
    def __init__(self, log_id, timestamp, accel: Vector3, gyro: Vector3, gyroDAngle: Vector3, mag: Vector3):
        super().__init__(log_id, timestamp)
        self.accel = accel
        self.gyro = gyro
        self.gyroDAngle = gyroDAngle
        self.mag = mag

    def accept(self, visitor: LogsVisitor):
        visitor.visit_imu_log(self)


class GpsLog(LogReading):
    def __init__(self, log_id, timestamp, position: GlobalPosition, horizontalAccuracy, velocityAccuracy, satelliteNumber, fix, velocity: NEDCoordinates):
        super().__init__(log_id, timestamp)
        self.position = position
        self.horizontalAccuracy = horizontalAccuracy
        self.velocityAccuracy = velocityAccuracy
        self.satelliteNumber = satelliteNumber
        self.fix = fix
        self.velocity = velocity

    def accept(self, visitor: LogsVisitor):
        visitor.visit_gps_log(self)


class BaroLog(LogReading):
    def __init__(self, log_id, timestamp, pressure, temperature):
        super().__init__(log_id, timestamp)
        self.pressure = pressure
        self.temperature = temperature

    def accept(self, visitor: LogsVisitor):
        visitor.visit_baro_log(self)
