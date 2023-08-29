from common.formats.binary.utils import FieldType, BinaryField

FIELDS = {
    "id": BinaryField(size=4, signed=False),
    "type": BinaryField(size=1, type=FieldType.CHAR),
    "timestamp": BinaryField(size=8, signed=False),

    # IMU fields
    "acceleration": BinaryField(size=4),
    "gyro": BinaryField(size=4),
    "dAngle": BinaryField(size=4, signed=False),
    "magnetometer": BinaryField(size=2),

    # Gps fields
    "altitude": BinaryField(size=4),
    "latitude": BinaryField(size=8),
    "longitude": BinaryField(size=8),

    "utc": BinaryField(size=8, signed=False),

    "precision_dilution": BinaryField(size=2, signed=False),

    "ellipsoid_altitude": BinaryField(size=4),
    "ground_speed": BinaryField(size=4),

    "velocity": BinaryField(size=4),

    "position_accuracy": BinaryField(size=4, signed=False),

    "heading": BinaryField(size=2, signed=False),
    "heading_offset": BinaryField(size=2),
    "heading_accuracy": BinaryField(size=2, signed=False),

    "satellite_number": BinaryField(size=1, signed=False),
    "fix": BinaryField(size=1, signed=False),


    # Barometer fields
    "pressure": BinaryField(size=4),
    "temperature": BinaryField(size=4, signed=False)
}
