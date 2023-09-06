from abc import ABC, abstractmethod

from common.models.utils import Vector3, GlobalPosition, NEDCoordinates
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
    def __init__(
            self,
            log_id,
            timestamp,
            accel_device_id,
            accel: Vector3,
            accel_temp,
            gyro_device_id,
            gyro: Vector3,
            gyro_d_angle: Vector3,
            gyro_temp,
            mag_dev_id,
            mag: Vector3
    ):
        super().__init__(log_id, timestamp)
        self.accelDevID = accel_device_id
        self.accel = accel
        self.accelTemp = accel_temp
        self.gyroDevID = gyro_device_id
        self.gyro = gyro
        self.gyroDAngle = gyro_d_angle
        self.gyroTemp = gyro_temp
        self.magDevID = mag_dev_id
        self.mag = mag

    def accept(self, visitor: LogsVisitor):
        visitor.visit_imu_log(self)


class GpsLog(LogEntry):
    def __init__(
            self,
            log_id,
            timestamp,
            device_id,
            position: GlobalPosition,
            utc,
            horizontal_dilution,
            vertical_dilution,
            alt_ellipsoid,
            ground_speed,
            velocity: NEDCoordinates,
            horizontal_accuracy,
            vertical_accuracy,
            velocity_accuracy,
            heading,
            heading_offset,
            heading_accuracy,
            satellite_number,
            fix,
    ):
        super().__init__(log_id, timestamp)
        self.devID = device_id

        self.position = position

        self.utc = utc

        self.horizontalPrecisionDilution = horizontal_dilution
        self.verticalPrecisionDilution = vertical_dilution

        self.ellipsoidAlt = alt_ellipsoid

        self.groundSpeed = ground_speed
        self.velocity = velocity

        self.horizontalAccuracy = horizontal_accuracy
        self.verticalAccuracy = vertical_accuracy
        self.velocityAccuracy = velocity_accuracy

        self.heading = heading
        self.headingOffset = heading_offset
        self.headingAccuracy = heading_accuracy

        self.satelliteNumber = satellite_number
        self.fix = fix

    def accept(self, visitor: LogsVisitor):
        visitor.visit_gps_log(self)


class BaroLog(LogEntry):
    def __init__(self, log_id, timestamp, device_ID, pressure, temperature):
        super().__init__(log_id, timestamp)
        self.devID = device_ID
        self.pressure = pressure
        self.temperature = temperature

    def accept(self, visitor: LogsVisitor):
        visitor.visit_baro_log(self)
