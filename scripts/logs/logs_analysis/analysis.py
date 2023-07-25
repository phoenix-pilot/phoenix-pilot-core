from abc import ABC, abstractmethod
from logs_analysis.context import AnalysisContext


class LogAnalysis(ABC):

    @abstractmethod
    def run(self, context: AnalysisContext):
        raise NotImplementedError()
