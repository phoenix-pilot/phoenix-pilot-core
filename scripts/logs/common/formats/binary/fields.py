from common.formats.binary.utils import FieldType, FieldSpecifier

FIELDS = {
    "id": FieldSpecifier(size=4, signed=False),
    "type": FieldSpecifier(size=1, type=FieldType.CHAR),
    "timestamp": FieldSpecifier(size=8, signed=False),

    # IMU fields
    "acceleration": FieldSpecifier(size=4),
    "gyro": FieldSpecifier(size=4),
    "dAngle": FieldSpecifier(size=4, signed=False),
    "magnetometer": FieldSpecifier(size=2),

    # Gps fields
    "latitude": FieldSpecifier(size=8),
    "longitude": FieldSpecifier(size=8),
    "altitude": FieldSpecifier(size=4),
    "horizontal_accuracy": FieldSpecifier(size=4, signed=False),
    "velocity_accuracy": FieldSpecifier(size=4, signed=False),
    "fix": FieldSpecifier(size=1, signed=False),
    "satellite_number": FieldSpecifier(size=1, signed=False),
    "velocity": FieldSpecifier(size=4),

    # Barometer fields
    "pressure": FieldSpecifier(size=4),
    "temperature": FieldSpecifier(size=4, signed=False)
}
