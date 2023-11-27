from dataclasses import dataclass

from scipy.spatial.transform import Rotation

from common.models.utils import Vector3, GlobalPosition, NEDCoordinates


@dataclass
class Imu:
    accelDeviceID: int
    accel: Vector3
    accelTemp: int

    gyroDeviceID: int
    gyro: Vector3
    gyroDAngle: Vector3
    gyroTemp: int

    magDeviceID: int
    mag: Vector3


@dataclass
class Gps:
    deviceID: int

    position: GlobalPosition

    utc: int

    horizontalPrecisionDilution: int
    verticalPrecisionDilution: int

    ellipsoidAlt: int

    groundSpeed: int

    velocity: NEDCoordinates

    horizontalAccuracy: int
    verticalAccuracy: int
    velocityAccuracy: int

    heading: int
    headingOffset: int
    headingAccuracy: int

    satelliteNumber: int
    fix: int


@dataclass
class Baro:
    deviceID: int
    pressure: int
    temperature: int


@dataclass
class EkfState:
    attitude: Rotation
    velocity: NEDCoordinates
    gyroscopeBias: Vector3
    accelerometerBias: Vector3
    position: NEDCoordinates
    magneticField: Vector3
