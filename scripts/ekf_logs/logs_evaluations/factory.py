import logs_evaluations.evaluations as evals
from logs_evaluations.log_evaluations import LogEvaluation


class EvaluationsFactory:
    @staticmethod
    def get_evaluations() -> list[LogEvaluation]:
        return [
            evals.MissingLogs(),
            evals.LogsCnt(),
            evals.EkfLoopTiming(),
            evals.GpsEkfComparison()
        ]
