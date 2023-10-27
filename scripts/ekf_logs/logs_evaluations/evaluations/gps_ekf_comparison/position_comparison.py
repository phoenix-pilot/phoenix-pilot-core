import matplotlib.pyplot as plt
import numpy as np

import common.models.units_conversions as uc
from logs_evaluations.context import StudyContext
from logs_evaluations.evaluations.gps_ekf_comparison.geo_distance import GeodeticDistance
from logs_evaluations.evaluations.gps_ekf_comparison.interactive_legend_factory import InteractiveLegendFactory


class PositionComparer:
    def __init__(self, context: StudyContext) -> None:
        self.figure = plt.figure()
        self.ax = self.figure.add_subplot()

        self.context = context

        self.distCalc = GeodeticDistance(
            origin_latitude=uc.nano_to_SI(context.gps_logs[0].data.position.latitude),
            origin_longitude=uc.nano_to_SI(context.gps_logs[0].data.position.longitude)
        )

        self.legendFactory = InteractiveLegendFactory(self.ax)

    def draw_figure(self):
        self._draw_ekf_state_pos()
        self._draw_gps_pos()
        self._draw_gps_extrapolation()

        self.ax.grid()
        self.ax.set_title("Position recording from GPS and EKF state")

        common_msg = "Distance from start position [$m$]"
        self.ax.set_xlabel("$\\longleftarrow$ West, East $\\longrightarrow$\n" + common_msg)
        self.ax.set_ylabel(common_msg + "\n$\\longleftarrow$ South, North $\\longrightarrow$")
        self.ax.set_aspect('equal', adjustable='box')

        self.legendFactory.generate_legend()

    def _draw_ekf_state_pos(self):
        x = np.array([log.data.position.east for log in self.context.state_logs])
        y = np.array([log.data.position.north for log in self.context.state_logs])

        scatter = self.ax.scatter(x, y, s=3, label="EKF state position")
        self.legendFactory.add_hideable_plot(scatter)

    def _draw_gps_pos(self):
        x = np.empty(len(self.context.gps_logs))
        y = np.empty(len(self.context.gps_logs))

        for i, gps_log in enumerate(self.context.gps_logs):
            x[i] = self.distCalc.longitude_diff(uc.nano_to_SI(gps_log.data.position.longitude))
            y[i] = self.distCalc.latitude_diff(uc.nano_to_SI(gps_log.data.position.latitude))

        scatter = self.ax.scatter(x, y, c="#f542e9", marker='s', label="GPS position")

        annotations = []
        for i in range(len(self.context.gps_logs)):
            an = self.ax.annotate(
                self.format_timestamp(self.context.gps_logs[i].timestamp),
                (x[i], y[i]),
                ha="center",
                xytext=(0, -12),
                textcoords='offset points'
            )
            annotations.append(an)

        self.legendFactory.add_hideable_plot(scatter, annotations)

    def _draw_gps_extrapolation(self):
        color = "#1ccc18"  # Green

        extrapolated_x = np.empty(len(self.context.gps_logs) - 2)
        extrapolated_y = np.empty(len(self.context.gps_logs) - 2)

        # Initiating last_dt with small value in case of the same timestamps in first two logs
        last_dt = 1

        # Calculate extrapolation
        for i in range(1, len(self.context.gps_logs) - 1):
            prev_log = self.context.gps_logs[i-1]
            curr_log = self.context.gps_logs[i]
            next_log = self.context.gps_logs[i+1]

            dt = curr_log.timestamp - prev_log.timestamp

            # Prevents from dividing by zero
            if dt == 0:
                dt = last_dt

            last_dt = dt

            extrapolation_dt = next_log.timestamp - curr_log.timestamp

            prev_x = self.distCalc.longitude_diff(uc.nano_to_SI(prev_log.data.position.longitude))
            prev_y = self.distCalc.latitude_diff(uc.nano_to_SI(prev_log.data.position.latitude))

            curr_x = self.distCalc.longitude_diff(uc.nano_to_SI(curr_log.data.position.longitude))
            curr_y = self.distCalc.latitude_diff(uc.nano_to_SI(curr_log.data.position.latitude))

            pos_dx = curr_x - prev_x
            pos_dy = curr_y - prev_y

            extrapolated_x[i-1] = curr_x + pos_dx * extrapolation_dt/dt
            extrapolated_y[i-1] = curr_y + pos_dy * extrapolation_dt/dt

        scatter = self.ax.scatter(
            extrapolated_x,
            extrapolated_y,
            c=color,
            s=5,
            label="Extrapolated GPS position"
        )

        # Add annotations
        annotations = []
        for i in range(len(extrapolated_x)):
            an = self.ax.annotate(
                self.format_timestamp(self.context.gps_logs[i+2].timestamp),
                (extrapolated_x[i], extrapolated_y[i]),
                ha="center",
                xytext=(0, 5),
                textcoords='offset points',
                c=color
            )
            annotations.append(an)

        self.legendFactory.add_hideable_plot(scatter, annotations)

    def format_timestamp(self, timestamp) -> str:
        dt = timestamp - self.context.all_logs[0].timestamp
        seconds = uc.micro_to_SI(dt)

        return "{:.1f}".format(seconds)
