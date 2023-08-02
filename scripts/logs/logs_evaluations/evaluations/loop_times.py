import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

from logs_evaluations.log_evaluations import LogEvaluation
from logs_evaluations.context import StudyContext


class EkfLoopTiming(LogEvaluation):
    arrowStyle = dict(facecolor='black', headlength=7, headwidth=10, width=3)

    def run(self, context: StudyContext):
        if context.missed_logs_cnt != 0:
            print("Unable to perform loop times analysis, because file does not contains all logs")

        length = len(context.time_logs)
        loop_times = np.zeros(length - 1, dtype=np.uint)

        for i in range(1, length):
            loop_times[i - 1] = context.time_logs[i].timestamp - context.time_logs[i - 1].timestamp

        self.__draw_plot(loop_times)

    @staticmethod
    def __draw_plot(times):
        x_points = range(len(times))
        mean_val = np.mean(times)
        max_val = np.max(times)
        min_val = np.min(times)

        fig = plt.figure()

        spec = gridspec.GridSpec(ncols=1, nrows=2,
                                 height_ratios=[1, 3])

        ax0 = fig.add_subplot(spec[0])
        ax0.set_title("EKF loop times analysis")
        ax0.boxplot(times, showmeans=True, meanline=True, vert=False)
        ax0.annotate("mean: {:.2f}".format(mean_val), xy=(mean_val, 1.1), xytext=(mean_val, 1.3), ha='center',
                     arrowprops=EkfLoopTiming.arrowStyle)
        ax0.annotate("min: {:.2f}".format(min_val), xy=(min_val, 1.05), xytext=(min_val, 1.2), ha='center',
                     arrowprops=EkfLoopTiming.arrowStyle)
        ax0.annotate("max: {:.2f}".format(max_val), xy=(max_val, 1.05), xytext=(max_val, 1.2), ha='center',
                     arrowprops=EkfLoopTiming.arrowStyle)
        ax0.set_xlabel("Time in microseconds [$\\mu s$]")
        ax0.set_yticks([])

        ax1 = fig.add_subplot(spec[1])
        ax1.set_title("EKF loops times")
        ax1.plot(x_points, times)
        ax1.set_xlabel("Loop number")
        ax1.set_ylabel("Loop time in microseconds [$\\mu s$]")

        plt.show()
