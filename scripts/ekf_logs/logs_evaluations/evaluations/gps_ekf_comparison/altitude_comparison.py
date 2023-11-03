import matplotlib.pyplot as plt
import numpy as np

import common.models.units_conversions as uc
from logs_evaluations.context import StudyContext
from logs_evaluations.evaluations.gps_ekf_comparison.interactive_legend_factory import InteractiveLegendFactory
from logs_evaluations.evaluations.gps_ekf_comparison.baro_to_altitude import BaroToAltitudeConverter


class AltitudeComparison:
    baro_calib_logs_cnt = 100

    def __init__(self, context: StudyContext) -> None:
        self.figure = plt.figure()
        self.ax1 = self.figure.add_subplot(2, 1, 1)
        self.ax2 = self.figure.add_subplot(2, 1, 2)

        self.context = context
        self.start_time = context.all_logs[0].timestamp

        self.legendFactory1 = InteractiveLegendFactory(self.ax1)
        self.legendFactory2 = InteractiveLegendFactory(self.ax2)

    def draw_figure(self):
        self._draw_vertical_velocity()
        self._draw_altitudes()

    def _draw_vertical_velocity(self):
        ekf_states = self.context.state_logs

        times = np.array([uc.from_micro(log.timestamp - self.start_time) for log in ekf_states])

        ekf_state_based_vel = np.array([-log.data.velocity.down for log in ekf_states])
        position_based_vel = np.empty(len(ekf_states))

        # Initiating last_dt with small value in case of the same timestamps in first two logs
        last_dt = 1

        for i in range(1, len(position_based_vel)):
            prev_log = ekf_states[i-1]
            curr_log = ekf_states[i]

            prev_alt = -prev_log.data.position.down
            curr_alt = -curr_log.data.position.down

            dist = curr_alt - prev_alt
            dt = uc.from_micro(curr_log.timestamp - prev_log.timestamp)

            # Prevents from dividing by zero
            if dt == 0:
                dt = last_dt

            last_dt = dt

            position_based_vel[i] = dist/dt

        plot, = self.ax1.plot(times, position_based_vel, label="Vertical velocity from EKF position")
        self.legendFactory1.add_hideable_plot(plot)

        plot, = self.ax1.plot(times, ekf_state_based_vel, label="Vertical velocity from EKF velocity")
        self.legendFactory1.add_hideable_plot(plot)

        self.legendFactory1.generate_legend()

        self.ax1.grid()
        self.ax1.set_title("Vertical velocity based on EKF states")
        self.ax1.set_xlabel("Time [$s$]")
        self.ax1.set_ylabel("Vertical velocity [$m/s$]")

    def _draw_altitudes(self):
        ekf_states = self.context.state_logs
        baro_logs = self.context.baro_logs[self.baro_calib_logs_cnt:]

        state_times = np.array([uc.from_micro(state.timestamp - self.start_time) for state in ekf_states])
        state_altitude = np.array([-state.data.position.down for state in ekf_states])

        baro_times = np.array([uc.from_micro(state.timestamp - self.start_time) for state in baro_logs])
        baro_altitude = self._baro_to_altitude()

        plot1, = self.ax2.plot(state_times, state_altitude, label="EKF state altitude")
        plot2, = self.ax2.plot(baro_times, baro_altitude, label="Baro based altitude")

        self.legendFactory2.add_hideable_plot(plot1)
        self.legendFactory2.add_hideable_plot(plot2)

        self.legendFactory2.generate_legend()

        self.ax2.grid()
        self.ax2.set_title("EKF state and barometer based altitude comparison")
        self.ax2.set_xlabel("Time [$s$]")
        self.ax2.set_ylabel("Altitude with $0$ as start position [$m$]")

    def _baro_to_altitude(self):
        converter = BaroToAltitudeConverter()

        baro_calib_logs = self.context.baro_logs[:self.baro_calib_logs_cnt]
        baro_values_logs = self.context.baro_logs[self.baro_calib_logs_cnt:]

        baro_calib_press = np.array([log.data.pressure for log in baro_calib_logs])
        baro_values_press = np.array([log.data.pressure for log in baro_values_logs])

        converter.calibrate(baro_calib_press)

        return converter.convert(baro_values_press)
