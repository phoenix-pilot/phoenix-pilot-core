# Phoenix-Pilot
#
# quad-control live logging utility. 
# 
# Reads and plots live logs from serial connection specified as parameter (or /dev/ttyUSB0 by default)
# All other serial connections should be closed. 
#
# WARNING: pythonqtplot is known for memory leaks, thus intensive using may leed to computer running slow
# ADVICE: cumbersome closing procedure - click (X) once on the window, wait for window to close, then ctrl+c in commandline
#
# Copyright 2022 Phoenix Systems
# Author: Mateusz Niewiadomski
#
# This file is part of Phoenix-Pilot.
#
# 
import argparse
import signal
import sys

import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore


def processLine(l):
    l = l.replace(":","")
    l = l.replace(",","")
    l = l.replace("\\r\\n", "")
    return l.split()

class lpgui:
    """Holistic wrapper class for quadLive gui creation/running and data acquisition/processing"""

    def __init__(self, ser: serial, args):
        """Gui init, and refresh timer handler"""

        self.plotcut = args.cut
        self.pwmLow = args.pwm
        self.ser = ser

        #app = pg.mkQApp("My multiple plot")
        self.win = pg.GraphicsLayoutWidget(show=True, title="Phoenix-pilot logging tool")
        self.win.resize(1500, 800)
        self.win.setWindowTitle("Phoenix-pilot logging tool")
        pg.setConfigOptions(antialias=True)

        self.win.nextRow()

        self.y = [[] for _ in range(3)]
        self.py = self.win.addPlot(title="Yaw", colspan=1)
        self.py.addLegend()

        # Prepare roll/pitch plot
        self.e = [[] for _ in range(3)]
        self.p2 = self.win.addPlot(title="Roll & Pitch", colspan=2)
        self.p2.addLegend()
        self.elines = [
            self.py.plot(pen="red", name="yaw"), 
            self.p2.plot(pen="green", name="pitch"), 
            self.p2.plot(pen="blue", name="roll")]

        self.win.nextRow()

        # Prepare PID plots
        self.pids = [[[] for _ in range(4)] for _ in range(3)]
        self.winpids = [self.win.addPlot(title="Roll pid"), self.win.addPlot(title="Pitch pid"), self.win.addPlot(title="Yaw pid")]
        self.linepids = [[] for _ in range(3)]
        for w in range(len(self.winpids)):
            self.winpids[w].showGrid(x=False, y=True, alpha=0.5)
            self.winpids[w].addLegend()
            self.linepids[w].append(self.winpids[w].plot(pen="red", name="P"))
            self.linepids[w].append(self.winpids[w].plot(pen="green", name="I"))
            self.linepids[w].append(self.winpids[w].plot(pen="blue", name="D"))
            self.linepids[w].append(self.winpids[w].plot(pen="orange", name="Σ"))

        self.win.nextRow()

        # Prepare PWM plots
        self.pwms = [[] for _ in range(4)]
        self.winpwm = self.win.addPlot(title="Engine PWMs", colspan=3)
        self.winpwm.addLegend()
        self.pwmlines = [
            self.winpwm.plot(pen="white", name="M4"),
            self.winpwm.plot(pen="red", name="M1"),
            self.winpwm.plot(pen="green", name="M3"),
            self.winpwm.plot(pen="blue", name="M2")]

        # Quadrocontrol has maximum frequency of 100Hz (with calculation time excluded) so with 10ms wait we should catch all data
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(10)


    def digestIntoData(self, ls):
        """Interpret chopped line of data from quadrocontrol, and decompose it into plot arrays"""

        if len(ls) <= 1:
            return

        # EKFE - ekf log of Euler angles
        if ls[0] == "EKFE":
            for i in range(len(self.e)):
                if len(self.e[i]) > self.plotcut:
                    self.e[i].pop(0)
                self.e[i].append(float(ls[i+2]))
                self.elines[i].setData(self.e[i])
        # PID - logs of pid values from pid controllers
        if ls[0] == "PID":
            # iterate over roll/pitch/yaw plots
            for p in range(len(self.pids)):
                # iterate over pid values in one plot
                for l in range(len(self.pids[p])):
                    if len(self.pids[p][l]) > self.plotcut:
                        self.pids[p][l].pop(0)
                    v = float(ls[2 + (p + 1) * 4 + l])
                    # There happens to be some huge values at the start so crop them
                    if -100 < v < 100:
                        self.pids[p][l].append(v)
                    self.linepids[p][l].setData(self.pids[p][l])
        # PWM - percent of throttle on each engine
        if ls[0] == "PWM":
            for i in range(0, 4):
                if len(self.pwms[i]) > self.plotcut:
                    self.pwms[i].pop(0)
                v = float(ls[i+1])
                # autoscaling causes problems at start end the very end
                if v > self.pwmLow:
                    self.pwms[i].append(v)
                self.pwmlines[i].setData(self.pwms[i])


    def update(self):
        """Data acquisition routine"""

        # flush and dummy read
        self.ser.flushInput()
        processLine(self.ser.readline().decode("utf-8"))
        for i in range(4):
            l = processLine(self.ser.readline().decode("utf-8"))
            try:
                self.digestIntoData(l)
            except:
                pass


    def guiStop(self, signalNumber, frame):
        """Plot stop routine"""

        self.timer.stop()
        print("Closing quadLive.py...")
        sys.exit(0)


def main():
    cmdParse = argparse.ArgumentParser()
    cmdParse.add_argument("port", default="/dev/ttyUSB0")
    cmdParse.add_argument("-c", "--cut", required=False, default=200)
    cmdParse.add_argument("-p", "--pwm", required=False, default=0.15)
    args = cmdParse.parse_args()

    try:
        ser = serial.Serial(args.port, baudrate=115200, timeout=1)
    except serial.SerialException:
        print(f"Error while opening f{args.port}")
        sys.exit(1)

    gui = lpgui(ser, args)

    signal.signal(signal.SIGINT, gui.guiStop)


# Run quadLive
main()


if __name__ == "__main__":
    pg.exec()
