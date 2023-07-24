class Vector3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class GlobalPosition:
    def __init__(self, latitude, longitude, altitude):
        self.altitude = altitude
        self.longitude = longitude
        self.latitude = latitude


class NEDCoordinates:
    def __init__(self, north, east, down):
        self.north = north
        self.east = east
        self.down = down
