from logs_analysis.analysis import LogAnalysis
from logs_analysis.missing_logs import MissingLogsAnalysis
from logs_analysis.loop_times import EkfLoopTiming
from logs_analysis.logs_cnt import LogsCnt


class AnalysesFactory:
    @staticmethod
    def get_analyses() -> list[LogAnalysis]:
        return [
            MissingLogsAnalysis(),
            LogsCnt(),
            EkfLoopTiming()
        ]
