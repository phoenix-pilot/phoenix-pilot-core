_nano_multiplier = 1_000_000_000.0
_micro_multiplier = 1_000_000.0


def from_nano(value):
    return value / _nano_multiplier


def to_nano(value):
    return value * _nano_multiplier


def from_micro(value):
    return value / _micro_multiplier


def to_micro(value):
    return value * _micro_multiplier
