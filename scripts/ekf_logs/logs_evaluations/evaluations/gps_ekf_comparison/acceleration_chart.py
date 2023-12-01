import sys
import math

import matplotlib.pyplot as plt
import numpy as np

import common.models.units_conversions as uc
from logs_evaluations.context import StudyContext
from logs_evaluations.evaluations.gps_ekf_comparison.iir_filter import IIR_filter
from logs_evaluations.evaluations.gps_ekf_comparison.interactive_legend_factory import InteractiveLegendFactory


def nearest_values(known_array, test_array):
    """Returns array of indices, were i-th element is an index the nearest value in known_array to test_array[i] value. Array known_array must be sorted."""

    result = np.empty(len(test_array), dtype=np.int_)

    for i, value in enumerate(test_array):
        idx = np.searchsorted(known_array, value, side="left")
        if idx > 0 and (idx == len(known_array) or math.fabs(value - known_array[idx-1]) < math.fabs(value - known_array[idx])):
            result[i] = idx-1
        else:
            result[i] = idx

    return result


class AccelerationChart:
    def __init__(self, context: StudyContext) -> None:
        if len(context.imu_logs) == 0:
            print("Cannot draw acceleration graph - no data from IMU", file=sys.stderr)

        self.figure = plt.figure()
        self.ax1 = self.figure.add_subplot(3, 1, 1)
        self.ax2 = self.figure.add_subplot(3, 1, 2)
        self.ax3 = self.figure.add_subplot(3, 1, 3)

        self.legendFactory1 = InteractiveLegendFactory(self.ax1)
        self.legendFactory2 = InteractiveLegendFactory(self.ax2)
        self.legendFactory3 = InteractiveLegendFactory(self.ax3)

        self.context = context
        self.start_time = context.all_logs[0].timestamp

        self.filter = IIR_filter(cur_input_coef=0.1, prev_result_coef=0.9)

        global_acc = self._accel_to_global_coord()
        self.north_acc_glob = global_acc[0]
        self.east_acc_glob = global_acc[1]
        self.down_acc_glob = global_acc[2]

        self.imu_times = np.array([uc.from_micro(log.timestamp - self.start_time) for log in self.context.imu_logs])

    def draw_figure(self):
        self._draw_earth_frame_acc()
        self._draw_body_frame_acc()
        self._draw_vertical_acc()

        self.figure.subplots_adjust(
            top=0.94,
            bottom=0.08,
            left=0.12,
            right=0.93,
            hspace=0.385,
            wspace=0.2
        )

    def _draw_earth_frame_acc(self):
        north_filtered = self.filter.run(self.north_acc_glob)
        east_filtered = self.filter.run(self.east_acc_glob)

        plot, = self.ax1.plot(
            self.imu_times,
            north_filtered,
            label="North acceleration",
            c="#64FA92"
        )
        self.legendFactory1.add_hideable_plot(plot)

        plot, = self.ax1.plot(
            self.imu_times,
            east_filtered,
            label="East acceleration",
            c="#FADB64"
        )
        self.legendFactory1.add_hideable_plot(plot)

        scatter = self.ax1.scatter(
            self.imu_times,
            self.north_acc_glob,
            s=2,
            label="North acceleration - raw data",
            c="#646EFA"
        )
        self.legendFactory1.add_hideable_plot(scatter)

        scatter = self.ax1.scatter(
            self.imu_times,
            self.east_acc_glob,
            s=2,
            label="East acceleration - raw data",
            c="#FA6664"
        )
        self.legendFactory1.add_hideable_plot(scatter)

        self.ax1.grid()
        self.ax1.set_title("Accelerations in earth frame of reference")
        self.ax1.set_ylabel("Acceleration [$mm/s^2$]")
        self.ax1.set_xlabel("Time [$s$]")
        self.legendFactory1.generate_legend()

    def _draw_body_frame_acc(self):
        imu_logs = self.context.imu_logs

        x = np.array([log.data.accel.x for log in imu_logs])
        y = np.array([log.data.accel.y for log in imu_logs])

        x_filtered = self.filter.run(x)
        y_filtered = self.filter.run(y)

        plot, = self.ax2.plot(
            self.imu_times,
            x_filtered,
            label="Acceleration along $x$ axis - filtered",
            c="#646EFA"
        )
        self.legendFactory2.add_hideable_plot(plot)

        plot, = self.ax2.plot(
            self.imu_times,
            y_filtered,
            label="Acceleration along $y$ axis - filtered",
            c="#FA6664"
        )
        self.legendFactory2.add_hideable_plot(plot)

        scatter = self.ax2.scatter(
            self.imu_times,
            x,
            s=2,
            label="Acceleration along $x$ axis - raw data",
            c="#64FA92"
        )
        self.legendFactory2.add_hideable_plot(scatter)

        scatter = self.ax2.scatter(
            self.imu_times,
            y,
            s=2,
            label="Acceleration along $y$ axis - raw data",
            c="#FADB64"
        )
        self.legendFactory2.add_hideable_plot(scatter)

        self.ax2.grid()
        self.ax2.set_title("Accelerations in body frame of reference")
        self.ax2.set_ylabel("Acceleration [$mm/s^2$]")
        self.ax2.set_xlabel("Time [$s$]")
        self.legendFactory2.generate_legend()

    def _draw_vertical_acc(self):
        alt_raw = -self.down_acc_glob

        alt_filtered = self.filter.run(alt_raw)

        scatter = self.ax3.scatter(
            self.imu_times,
            alt_raw,
            s=2,
            label="Vertical acceleration - raw data",
            c="#FA8D07"
        )
        self.legendFactory3.add_hideable_plot(scatter)

        plot, = self.ax3.plot(
            self.imu_times,
            alt_filtered,
            label="Vertical acceleration - filtered",
            c="#072BFA"
        )
        self.legendFactory3.add_hideable_plot(plot)

        self.ax3.grid()
        self.ax3.set_title("Vertical acceleration")
        self.ax3.set_ylabel("Acceleration [$mm/s^2$]")
        self.ax3.set_xlabel("Time [$s$]")
        self.legendFactory3.generate_legend()

    def _accel_to_global_coord(self):
        sensor_timestamp = np.array([log.timestamp for log in self.context.imu_logs])
        state_timestamp = np.array([log.timestamp for log in self.context.state_logs])

        nearest_state_indices = nearest_values(state_timestamp, sensor_timestamp)
        rotations = np.array([self.context.state_logs[i].data.attitude for i in nearest_state_indices])

        north = np.empty(len(self.context.imu_logs))
        east = np.empty(len(self.context.imu_logs))
        down = np.empty(len(self.context.imu_logs))

        for i in range(len(north)):
            vector = rotations[i].apply(self.context.imu_logs[i].data.accel.as_array())
            north[i] = vector[0]
            east[i] = vector[1]
            down[i] = vector[2]

        return north, east, down
