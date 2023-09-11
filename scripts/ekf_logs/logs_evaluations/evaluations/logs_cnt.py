import matplotlib.pyplot as plt

from logs_evaluations.log_evaluations import LogEvaluation
from logs_evaluations.context import StudyContext


class LogsCnt(LogEvaluation):
    def __init__(self) -> None:
        self.labels = []
        self.data = []
        self.remainingLogs = 0
        self.allLogs = 0

    def run(self, context: StudyContext):
        self.remainingLogs = len(context.all_logs)
        self.allLogs = self.remainingLogs

        if context.time_logs:
            self.__add_log_type_to_summary(context.time_logs, "TIME")

        if context.imu_logs:
            self.__add_log_type_to_summary(context.imu_logs, "IMU")

        if context.gps_logs:
            self.__add_log_type_to_summary(context.gps_logs, "GPS")

        if context.baro_logs:
            self.__add_log_type_to_summary(context.baro_logs, "Baro")

        if context.state_logs:
            self.__add_log_type_to_summary(context.state_logs, "EKF status")

        self.__show_summary()

    def __add_log_type_to_summary(self, logs, label):
        logs_cnt = len(logs)
        self.labels.append(label)
        self.data.append(logs_cnt)
        self.remainingLogs = self.remainingLogs - logs_cnt

    def __show_summary(self):
        if self.remainingLogs > 0:
            self.labels.append("Others")
            self.data.append(self.remainingLogs)

        plt.pie(self.data, labels=self.labels, autopct='%1.1f%%')
        plt.suptitle("Number of logs", fontsize=18)
        plt.title(f"All logs: {self.allLogs}", fontsize=10)
        plt.show()
