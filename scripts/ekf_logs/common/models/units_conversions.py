_nano_multiplier = 1_000_000_000.0
_micro_multiplier = 1_000_000.0


def nano_to_SI(value):
    return value / _nano_multiplier


def SI_to_nano(value):
    return value * _nano_multiplier


def micro_to_SI(value):
    return value / _micro_multiplier


def SI_to_micro(value):
    return value * _micro_multiplier
