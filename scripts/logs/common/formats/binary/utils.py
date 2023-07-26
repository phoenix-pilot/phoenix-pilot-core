from enum import Enum
from dataclasses import dataclass


class FieldType(Enum):
    INT = 1,
    CHAR = 2


@dataclass
class BinaryField:
    size: int
    signed: bool = True
    type: FieldType = FieldType.INT
