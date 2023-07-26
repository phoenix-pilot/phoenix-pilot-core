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
    "latitude": BinaryField(size=8),
    "longitude": BinaryField(size=8),
    "altitude": BinaryField(size=4),
    "horizontal_accuracy": BinaryField(size=4, signed=False),
    "velocity_accuracy": BinaryField(size=4, signed=False),
    "fix": BinaryField(size=1, signed=False),
    "satellite_number": BinaryField(size=1, signed=False),
    "velocity": BinaryField(size=4),

    # Barometer fields
    "pressure": BinaryField(size=4),
    "temperature": BinaryField(size=4, signed=False)
}
