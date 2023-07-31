from dataclasses import dataclass


@dataclass
class Vector3:
    x: int
    y: int
    z: int


@dataclass
class GlobalPosition:
    altitude: int
    longitude: int
    latitude: int


@dataclass
class NEDCoordinates:
    north: int
    east: int
    down: int
