import sys

import matplotlib.pyplot as plt
import numpy as np

import common.models.units_conversions as uc
from logs_evaluations.context import StudyContext
from logs_evaluations.evaluations.gps_ekf_comparison.iir_filter import IIR_filter
from logs_evaluations.evaluations.gps_ekf_comparison.interactive_legend_factory import InteractiveLegendFactory


def nearest_values(known_array, test_array):
    """Returns array of indices, were ith element is an index the nearest value in known_array to test_array[i] value"""

    differences = (test_array.reshape(1, -1) - known_array.reshape(-1, 1))
    indices = np.abs(differences).argmin(axis=0)

    return indices


class AccelerationChart:
    north_plot_color = "#030f82"
    east_plot_color = "#fa1bf2"

    def __init__(self, context: StudyContext) -> None:
        if len(context.imu_logs) == 0:
            print("Cannot draw acceleration graph - no data from IMU", file=sys.stderr)

        self.figure = plt.figure()
        self.ax1 = self.figure.add_subplot(2, 1, 1)
        self.ax2 = self.figure.add_subplot(2, 1, 2)

        self.legendFactory1 = InteractiveLegendFactory(self.ax1)
        self.legendFactory2 = InteractiveLegendFactory(self.ax2)

        self.context = context
        self.start_time = context.all_logs[0].timestamp

    def draw_figure(self):
        north, east = self._accel_to_global_coord(self.context)
        time_from_start = np.array([uc.micro_to_SI(log.timestamp - self.start_time) for log in self.context.imu_logs])

        filter = IIR_filter(cur_input_coef=0.1, prev_result_coef=0.9)

        north_filtered = filter.run(north)
        east_filtered = filter.run(east)

        plot, = self.ax1.plot(
            time_from_start,
            north_filtered,
            label="North acceleration",
            linewidth=0.5,
            c=AccelerationChart.north_plot_color
        )
        self.legendFactory1.add_hideable_plot(plot)

        plot, = self.ax1.plot(
            time_from_start,
            east_filtered,
            label="East acceleration",
            linewidth=0.5,
            c=AccelerationChart.east_plot_color
        )
        self.legendFactory1.add_hideable_plot(plot)

        self.ax1.grid()
        self.ax1.set_title("IMU accelerations (filtered) in earth frame of reference")
        self.ax1.set_ylabel("Acceleration [$mm/s^2$]")
        self.ax1.set_xlabel("Time [$s$]")
        self.legendFactory1.generate_legend()

        scatter = self.ax2.scatter(
            time_from_start,
            north,
            s=2,
            label="North acceleration",
            c=AccelerationChart.north_plot_color
        )
        self.legendFactory2.add_hideable_plot(scatter)

        scatter = self.ax2.scatter(
            time_from_start,
            east,
            s=2,
            label="East acceleration",
            c=AccelerationChart.east_plot_color
        )
        self.legendFactory2.add_hideable_plot(scatter)

        self.ax2.grid()
        self.ax2.set_title("IMU accelerations (raw sensor) in earth frame of reference")
        self.ax2.set_ylabel("Acceleration [$mm/s^2$]")
        self.ax2.set_xlabel("Time [$s$]")
        self.legendFactory2.generate_legend()

    def _accel_to_global_coord(self, context: StudyContext):
        sensor_timestamp = np.array([log.timestamp for log in context.imu_logs])
        state_timestamp = np.array([log.timestamp for log in context.state_logs])

        nearest_state_indices = nearest_values(state_timestamp, sensor_timestamp)
        rotations = np.array([context.state_logs[i].data.attitude for i in nearest_state_indices])

        north = np.empty(len(context.imu_logs))
        east = np.empty(len(context.imu_logs))

        for i in range(len(north)):
            vector = rotations[i].apply(context.imu_logs[i].data.accel.as_array())
            north[i] = vector[0]
            east[i] = vector[1]

        return north, east
