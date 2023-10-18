import sys

import matplotlib.pyplot as plt
from geopy import distance
import numpy as np

from logs_evaluations.log_evaluations import LogEvaluation
from logs_evaluations.context import StudyContext


def nanodegrees_to_degrees(nanodegrees):
    return nanodegrees / 1_000_000_000.0


def microseconds_to_seconds(microseconds):
    return microseconds / 1_000_000.0


def geo_distance(latitude1, longitude1, latitude2, longitude2):
    return distance.geodesic(
        (latitude1, longitude1),
        (latitude2, longitude2),
        ellipsoid='WGS-84'
    ).m


def nearest_values(known_array, test_array):
    """Returns array of indices, were ith element is an index the nearest value in known_array to test_array[i] value"""

    differences = (test_array.reshape(1, -1) - known_array.reshape(-1, 1))
    indices = np.abs(differences).argmin(axis=0)

    return indices


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

        self._position_comparison(context)
        self._speed_comparison(context)
        self._acceleration_chart(context)

        plt.show()

    def _position_comparison(self, context: StudyContext):
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
        ax.legend()

    def _speed_comparison(self, context: StudyContext):
        fig = plt.figure()
        ax = fig.add_subplot()

        self._draw_ekf_velocities(ax, context)
        self._draw_gps_velocities(ax, context)

        ax.grid()
        ax.set_title("GPS and EKF speed comparison")
        ax.set_ylabel("Speed [$m/s$]")
        ax.set_xlabel("Time [$s$]")
        ax.legend()

    def _acceleration_chart(self, context: StudyContext):
        if len(context.imu_logs) == 0:
            print("Cannot draw acceleration graph - no data from IMU", file=sys.stderr)

        north_direction_color = "#030f82"
        east_direction_color = "#fa1bf2"

        fig = plt.figure()
        ax1 = fig.add_subplot(2, 1, 1)

        north, east = self._accel_to_global_coord(context)

        time_from_start = np.array([microseconds_to_seconds(log.timestamp - self.start_time) for log in context.imu_logs])

        north_filtered = self._iir_filter(north)
        east_filtered = self._iir_filter(east)

        ax1.plot(time_from_start, north_filtered, label="North acceleration", linewidth=0.5, c=north_direction_color)
        ax1.plot(time_from_start, east_filtered, label="East acceleration", linewidth=0.5, c=east_direction_color)

        ax1.grid()
        ax1.set_title("IMU accelerations (filtered) in earth frame of reference")
        ax1.set_ylabel("Acceleration [$mm/s^2$]")
        ax1.set_xlabel("Time [$s$]")
        ax1.legend()

        ax2 = fig.add_subplot(2, 1, 2)

        ax2.scatter(time_from_start, north, s=2, label="North acceleration", c=north_direction_color)
        ax2.scatter(time_from_start, east, s=2, label="East acceleration", c=east_direction_color)

        ax2.grid()
        ax2.set_title("IMU accelerations (raw sensor) in earth frame of reference")
        ax2.set_ylabel("Acceleration [$mm/s^2$]")
        ax2.set_xlabel("Time [$s$]")
        ax2.legend()

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

    def _iir_filter(self, accels):
        result = np.empty(len(accels))

        result[0] = accels[0]

        for i in range(1, len(result)):
            result[i] = 0.1 * accels[i] + 0.9 * result[i-1]

        return result

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

        # Initiating last_dt with small value in case of the same timestamps in first two logs
        last_dt = 1

        # Calculate extrapolation
        for i in range(1, len(context.gps_logs) - 1):
            prev_log = context.gps_logs[i-1]
            curr_log = context.gps_logs[i]
            next_log = context.gps_logs[i+1]

            dt = curr_log.timestamp - prev_log.timestamp

            # Prevents from dividing by zero
            if dt == 0:
                dt = last_dt

            last_dt = dt

            extrapolation_dt = next_log.timestamp - curr_log.timestamp

            prev_x = self.calculate_gps_x(context.gps_logs[i-1])
            prev_y = self.calculate_gps_y(context.gps_logs[i-1])

            curr_x = self.calculate_gps_x(context.gps_logs[i])
            curr_y = self.calculate_gps_y(context.gps_logs[i])

            pos_dx = curr_x - prev_x
            pos_dy = curr_y - prev_y

            extrapolated_x[i-1] = curr_x + pos_dx * extrapolation_dt/dt
            extrapolated_y[i-1] = curr_y + pos_dy * extrapolation_dt/dt

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

    def _draw_ekf_velocities(self, ax, context: StudyContext):
        ekf_states = context.state_logs

        time = np.array([microseconds_to_seconds(state.timestamp - self.start_time) for state in ekf_states])
        ekf_velocity_NS = np.array([state.data.velocity.x for state in ekf_states])
        ekf_velocity_EW = np.array([state.data.velocity.y for state in ekf_states])

        ax.plot(time, ekf_velocity_NS, c="#0398fc", label="EKF N-S speed")
        ax.plot(time, ekf_velocity_EW, c="#f403fc", label="EKF E-W speed")

    def _draw_gps_velocities(self, ax, context: StudyContext):
        gps_logs = context.gps_logs

        gps_x = np.array([microseconds_to_seconds(log.timestamp - self.start_time) for log in gps_logs])
        gps_velocity_NS = np.empty(len(gps_logs))
        gps_velocity_EW = np.empty(len(gps_logs))

        gps_velocity_NS[0] = 0
        gps_velocity_EW[0] = 0

        # Initiating last_dt with small value in case of the same timestamps in first two logs
        last_dt = 1

        for i in range(1, len(gps_velocity_NS)):
            prev_log = gps_logs[i-1]
            curr_log = gps_logs[i]

            dist_n = self.calculate_gps_y(curr_log) - self.calculate_gps_y(prev_log)
            dist_e = self.calculate_gps_x(curr_log) - self.calculate_gps_x(prev_log)

            dt = microseconds_to_seconds(curr_log.timestamp - prev_log.timestamp)

            # Prevents from dividing by zero
            if dt == 0:
                dt = last_dt

            last_dt = dt

            gps_velocity_NS[i] = dist_n/dt
            gps_velocity_EW[i] = dist_e/dt

        ax.plot(gps_x, gps_velocity_NS, label="GPS N-S speed", c="#2cfc03", linestyle="dashed")
        ax.plot(gps_x, gps_velocity_EW, label="GPS E-W speed", c="#fc9d03", linestyle="dashed")

    def format_timestamp(self, timestamp) -> str:
        dt = timestamp - self.start_time
        seconds = microseconds_to_seconds(dt)

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
