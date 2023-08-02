from abc import ABC, abstractmethod
from logs_evaluations.context import StudyContext


class LogEvaluation(ABC):

    @abstractmethod
    def run(self, context: StudyContext):
        raise NotImplementedError()
