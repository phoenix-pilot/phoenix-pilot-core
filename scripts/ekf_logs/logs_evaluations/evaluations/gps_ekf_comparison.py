import sys

import matplotlib.pyplot as plt
from geopy import distance
import numpy as np

from logs_evaluations.log_evaluations import LogEvaluation
from logs_evaluations.context import StudyContext


def nanodegrees_to_degrees(nanodegrees):
    return nanodegrees / 1_000_000_000.0


def geo_distance(latitude1, longitude1, latitude2, longitude2):
    return distance.geodesic(
        (latitude1, longitude1),
        (latitude2, longitude2),
        ellipsoid='WGS-84'
    ).m


class GpsEkfComparison(LogEvaluation):
    def __init__(self) -> None:
        self.start_lat = None
        self.start_lon = None

        self.start_time = None

    def init_evaluation(self, context: StudyContext):
        self.start_time = context.all_logs[0].timestamp

        first_gps = context.gps_logs[0].data

        self.start_lat = nanodegrees_to_degrees(first_gps.position.latitude)
        self.start_lon = nanodegrees_to_degrees(first_gps.position.longitude)

    def run(self, context: StudyContext):
        if len(context.gps_logs) == 0:
            print("Cannot compare gps data with ekf state, no gps logs", file=sys.stderr)
            return

        if len(context.state_logs) == 0:
            print("Cannot compare gps data with ekf state, no gps logs", file=sys.stderr)
            return

        self.init_evaluation(context)

        fig = plt.figure()
        ax = fig.add_subplot()

        self._draw_ekf_state_pos(ax, context)
        self._draw_gps_pos(ax, context)
        self._draw_gps_extrapolation(ax, context)

        ax.grid()
        ax.set_title("Position recording from GPS and EKF state")

        common_msg = "Distance from start position [$m$]"
        ax.set_xlabel("$\\longleftarrow$ West, East $\\longrightarrow$\n" + common_msg)
        ax.set_ylabel(common_msg + "\n$\\longleftarrow$ South, North $\\longrightarrow$")
        ax.set_aspect('equal', adjustable='box')

        plt.legend()
        plt.show()

    def _draw_gps_pos(self, ax, context: StudyContext):
        x = np.array([self.calculate_gps_x(log) for log in context.gps_logs])
        y = np.array([self.calculate_gps_y(log) for log in context.gps_logs])

        ax.scatter(x, y, c="#f542e9", marker='s', label="GPS position")

        for i in range(len(context.gps_logs)):
            ax.annotate(
                self.format_timestamp(context.gps_logs[i].timestamp),
                (x[i], y[i]),
                ha="center",
                xytext=(0, -12),
                textcoords='offset points'
            )

    def _draw_ekf_state_pos(self, ax, context: StudyContext):
        x = np.array([log.data.position.east for log in context.state_logs])
        y = np.array([log.data.position.north for log in context.state_logs])

        ax.scatter(x, y, s=3, label="EKF state position")

    def _draw_gps_extrapolation(self, ax, context: StudyContext):
        color = "#1ccc18"  # Green

        extrapolated_x = np.empty(len(context.gps_logs) - 2)
        extrapolated_y = np.empty(len(context.gps_logs) - 2)

        # Calculate extrapolation
        for i in range(1, len(context.gps_logs) - 1):
            prev_log = context.gps_logs[i-1]
            curr_log = context.gps_logs[i]
            next_log = context.gps_logs[i+1]

            if (curr_log.timestamp == prev_log.timestamp):
                # Truncating the last element
                extrapolated_x = extrapolated_x[:len(extrapolated_x) - 2]
                extrapolated_y = extrapolated_y[:len(extrapolated_y) - 2]
                continue

            dt = curr_log.timestamp - prev_log.timestamp
            extrapolation_dt = next_log.timestamp - curr_log.timestamp

            prev_pos_x = self.calculate_gps_x(context.gps_logs[i-1])
            prev_pos_y = self.calculate_gps_y(context.gps_logs[i-1])

            curr_pos_x = self.calculate_gps_x(context.gps_logs[i])
            curr_pos_y = self.calculate_gps_y(context.gps_logs[i])

            prev_to_curr_vec_x = curr_pos_x - prev_pos_x
            prev_to_curr_vec_y = curr_pos_y - prev_pos_y

            extrapolated_x[i-1] = curr_pos_x + prev_to_curr_vec_x * extrapolation_dt/dt
            extrapolated_y[i-1] = curr_pos_y + prev_to_curr_vec_y * extrapolation_dt/dt

        ax.scatter(extrapolated_x, extrapolated_y, c=color, s=5, label="Extrapolated GPS position")

        # Add annotations
        for i in range(len(extrapolated_x)):
            ax.annotate(
                self.format_timestamp(context.gps_logs[i+2].timestamp),
                (extrapolated_x[i], extrapolated_y[i]),
                ha="center",
                xytext=(0, 5),
                textcoords='offset points',
                c=color
            )

    def format_timestamp(self, timestamp) -> str:
        dt = timestamp - self.start_time
        seconds = dt / 1_000_000

        return "{:.1f}".format(seconds)

    def calculate_gps_x(self, gps_log):
        log_lon = nanodegrees_to_degrees(gps_log.data.position.longitude)

        res = geo_distance(
            self.start_lat, self.start_lon,
            self.start_lat, log_lon
        )

        if self.start_lon < log_lon:
            res = -res

        return -res

    def calculate_gps_y(self, gps_log):
        log_lat = nanodegrees_to_degrees(gps_log.data.position.latitude)

        res = geo_distance(
            self.start_lat, self.start_lon,
            log_lat, self.start_lon,
        )

        if self.start_lat > log_lat:
            res = -res

        return res
