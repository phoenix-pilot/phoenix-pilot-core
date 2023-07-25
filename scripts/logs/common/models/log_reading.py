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
    def __init__(self, log_id, timestamp, accel: Vector3, gyro: Vector3, gyro_d_angle: Vector3, mag: Vector3):
        super().__init__(log_id, timestamp)
        self.accel = accel
        self.gyro = gyro
        self.gyroDAngle = gyro_d_angle
        self.mag = mag

    def accept(self, visitor: LogsVisitor):
        visitor.visit_imu_log(self)


class GpsLog(LogReading):
    def __init__(
            self,
            log_id,
            timestamp,
            position: GlobalPosition,
            horizontal_accuracy,
            velocity_accuracy,
            satellite_number,
            fix,
            velocity: NEDCoordinates
    ):
        super().__init__(log_id, timestamp)
        self.position = position
        self.horizontalAccuracy = horizontal_accuracy
        self.velocityAccuracy = velocity_accuracy
        self.satelliteNumber = satellite_number
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
