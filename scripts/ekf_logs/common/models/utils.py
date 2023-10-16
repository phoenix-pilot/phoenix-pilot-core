from dataclasses import dataclass


@dataclass
class Vector3:
    x: int | float
    y: int | float
    z: int | float

    def Zero():
        return Vector3(0, 0, 0)

    def as_array(self):
        return [self.x, self.y, self.z]


@dataclass
class GlobalPosition:
    altitude: int | float
    latitude: int | float
    longitude: int | float


@dataclass
class NEDCoordinates:
    north: int | float
    east: int | float
    down: int | float
