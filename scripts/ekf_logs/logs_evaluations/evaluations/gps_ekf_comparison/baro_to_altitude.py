import numpy as np


class BaroToAltitudeConverter:
    def __init__(self) -> None:
        self.base_pressure = 101_325,  # normal sea level pressure

    def calibrate(self, pressures_in_pa: np.ndarray):
        self.base_pressure = pressures_in_pa.mean()

    def convert(self, pressures_in_pa: np.ndarray) -> np.ndarray:
        return 8453.669 * np.log(self.base_pressure / pressures_in_pa)
