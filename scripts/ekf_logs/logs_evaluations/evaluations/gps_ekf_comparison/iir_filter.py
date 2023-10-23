import numpy as np


class IIR_filter:
    def __init__(self, cur_input_coef, prev_result_coef) -> None:
        self.input_coef = cur_input_coef
        self.result_coef = prev_result_coef

    def run(self, data):
        result = np.empty(len(data))

        result[0] = data[0]

        for i in range(1, len(result)):
            result[i] = self.input_coef * data[i] + self.result_coef * result[i-1]

        return result
