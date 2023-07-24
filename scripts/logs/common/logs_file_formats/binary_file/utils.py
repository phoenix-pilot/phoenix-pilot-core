from enum import Enum
from dataclasses import dataclass


class FieldType(Enum):
    INT = 1,
    CHAR = 2

@dataclass
class FieldSpecifier:
    size: int
    signed: bool = True
    type: FieldType = FieldType.INT
