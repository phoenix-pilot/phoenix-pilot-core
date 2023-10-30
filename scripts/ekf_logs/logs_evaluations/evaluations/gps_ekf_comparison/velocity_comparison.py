import matplotlib.pyplot as plt
import numpy as np

import common.models.units_conversions as uc
from logs_evaluations.context import StudyContext
from logs_evaluations.evaluations.gps_ekf_comparison.geo_distance import GeodeticDistance
from logs_evaluations.evaluations.gps_ekf_comparison.interactive_legend_factory import InteractiveLegendFactory


class VelocityComparer:
    def __init__(self, context: StudyContext) -> None:
        self.figure = plt.figure()
        self.ax = self.figure.add_subplot()

        self.context = context
        self.start_time = context.all_logs[0].timestamp

        self.legendFactory = InteractiveLegendFactory(self.ax)

    def draw_figure(self):
        self._draw_ekf_velocities()
        self._draw_gps_velocities()

        self.ax.grid()
        self.ax.set_title("GPS and EKF speed comparison")
        self.ax.set_ylabel("Speed [$m/s$]")
        self.ax.set_xlabel("Time [$s$]")
        self.legendFactory.generate_legend()

    def _draw_ekf_velocities(self):
        ekf_states = self.context.state_logs

        time = np.array([uc.from_micro(state.timestamp - self.start_time) for state in ekf_states])
        ekf_velocity_NS = np.array([state.data.velocity.x for state in ekf_states])
        ekf_velocity_EW = np.array([state.data.velocity.y for state in ekf_states])

        plot1, = self.ax.plot(time, ekf_velocity_NS, c="#0398fc", label="EKF N-S speed")
        plot2, = self.ax.plot(time, ekf_velocity_EW, c="#f403fc", label="EKF E-W speed")

        self.legendFactory.add_hideable_plot(plot1)
        self.legendFactory.add_hideable_plot(plot2)

    def _draw_gps_velocities(self):
        gps_logs = self.context.gps_logs

        gps_x = np.array([uc.from_micro(log.timestamp - self.start_time) for log in gps_logs])
        gps_velocity_NS = np.empty(len(gps_logs))
        gps_velocity_EW = np.empty(len(gps_logs))

        gps_velocity_NS[0] = 0
        gps_velocity_EW[0] = 0

        # Initiating last_dt with small value in case of the same timestamps in first two logs
        last_dt = 1

        for i in range(1, len(gps_velocity_NS)):
            prev_log = gps_logs[i-1]
            curr_log = gps_logs[i]

            prev_lat = uc.from_nano(prev_log.data.position.latitude)
            prev_lon = uc.from_nano(prev_log.data.position.longitude)

            curr_lat = uc.from_nano(curr_log.data.position.latitude)
            curr_lon = uc.from_nano(curr_log.data.position.longitude)

            distCalc = GeodeticDistance(
                origin_latitude=prev_lat,
                origin_longitude=prev_lon
            )

            dist_n = distCalc.latitude_diff(curr_lat)
            dist_e = distCalc.longitude_diff(curr_lon)

            dt = uc.from_micro(curr_log.timestamp - prev_log.timestamp)

            # Prevents from dividing by zero
            if dt == 0:
                dt = last_dt

            last_dt = dt

            gps_velocity_NS[i] = dist_n/dt
            gps_velocity_EW[i] = dist_e/dt

        plot1, = self.ax.plot(gps_x, gps_velocity_NS, label="GPS N-S speed", c="#2cfc03", linestyle="dashed")
        plot2, = self.ax.plot(gps_x, gps_velocity_EW, label="GPS E-W speed", c="#fc9d03", linestyle="dashed")

        self.legendFactory.add_hideable_plot(plot1)
        self.legendFactory.add_hideable_plot(plot2)
