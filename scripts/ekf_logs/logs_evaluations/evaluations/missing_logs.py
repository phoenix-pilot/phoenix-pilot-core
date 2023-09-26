import matplotlib.pyplot as plt
import numpy as np

from logs_evaluations.log_evaluations import LogEvaluation
from logs_evaluations.context import StudyContext


class MissingLogs(LogEvaluation):

    def run(self, context: StudyContext):
        if context.missed_logs_cnt == 0:
            print("All logs was saved")
            return

        length = len(context.all_logs)
        missed_logs = np.zeros(length, dtype=np.uint)

        for i in range(length):
            missed = context.get_missed(i)
            if missed > 0:
                missed_logs[i] = missed

        print(f"Number of missed logs: {context.missed_logs_cnt}")
        MissingLogs.__draw_plot(missed_logs, context.missed_logs_cnt)

    @staticmethod
    def __draw_plot(missed_logs: list[int], missed_logs_cnt: int):
        x_points = range(len(missed_logs))

        plt.stem(x_points, missed_logs)

        plt.suptitle("Number of missed logs",
                     fontsize=18)
        plt.title(f"Total number of missed logs: {missed_logs_cnt}",
                  fontsize=10)

        plt.xlabel("Log number")
        plt.ylabel("Number of missed logs")

        plt.show()
