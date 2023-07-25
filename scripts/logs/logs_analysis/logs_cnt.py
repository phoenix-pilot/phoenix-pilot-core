import matplotlib.pyplot as plt

from logs_analysis.analysis import LogAnalysis
from logs_analysis.context import AnalysisContext


class LogsCnt(LogAnalysis):
    def run(self, context: AnalysisContext):
        all_cnt = len(context.all_logs)
        time_cnt = len(context.time_logs)
        imu_cnt = len(context.imu_logs)
        gps_cnt = len(context.gps_logs)
        baro_cnt = len(context.baro_logs)
        others_cnt = all_cnt - time_cnt - imu_cnt - gps_cnt - baro_cnt

        labels = ["Time", "Imu", "GPS", "Baro"]
        data = [time_cnt, imu_cnt, gps_cnt, baro_cnt]

        if others_cnt != 0:
            labels.append("Others")
            data.append(others_cnt)

        plt.pie(data, labels=labels, autopct='%1.1f%%')
        plt.suptitle("Number of logs", fontsize=18)
        plt.title(f"All logs: {all_cnt}", fontsize=10)
