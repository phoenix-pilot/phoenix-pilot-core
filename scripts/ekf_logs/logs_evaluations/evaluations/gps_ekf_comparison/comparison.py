import sys

import matplotlib.pyplot as plt

from logs_evaluations.log_evaluations import LogEvaluation
from logs_evaluations.context import StudyContext

from logs_evaluations.evaluations.gps_ekf_comparison.position_comparison import PositionComparer
from logs_evaluations.evaluations.gps_ekf_comparison.velocity_comparison import VelocityComparer
from logs_evaluations.evaluations.gps_ekf_comparison.acceleration_chart import AccelerationChart
from logs_evaluations.evaluations.gps_ekf_comparison.altitude_comparison import AltitudeComparison


class GpsEkfComparison(LogEvaluation):
    def run(self, context: StudyContext):
        if len(context.gps_logs) == 0:
            print("Cannot compare gps data with ekf state, no gps logs", file=sys.stderr)
            return

        if len(context.state_logs) == 0:
            print("Cannot compare gps data with ekf state, no gps logs", file=sys.stderr)
            return

        position = PositionComparer(context)
        velocity = VelocityComparer(context)
        acceleration = AccelerationChart(context)
        altitude = AltitudeComparison(context)

        position.draw_figure()
        velocity.draw_figure()
        acceleration.draw_figure()
        altitude.draw_figure()

        plt.show()
