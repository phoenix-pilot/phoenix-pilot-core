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

        # Prepare roll/pitch plot
        self.eulerData = [[] for _ in range(3)]
        self.rollPitchPlot = self.win.addPlot(title="Roll & Pitch", colspan=1)
        self.rollPitchPlot.addLegend()
        # Prepare yaw plot
        self.yawPlot = self.win.addPlot(title="Yaw", colspan=1)
        self.yawPlot.addLegend()
        self.eulerDatalines = [
            self.yawPlot.plot(pen="red", name="yaw"),
            self.rollPitchPlot.plot(pen="green", name="pitch"),
            self.rollPitchPlot.plot(pen="blue", name="roll")]

        # Prepare position plot (only height currently)
        self.enuzData = []
        self.enuzPlot = self.win.addPlot(title="Height", colspan=1)
        self.enuzPlot.addLegend()
        self.enuzDatalines = self.enuzPlot.plot(pen="yellow", name="enuz")

        self.win.nextRow()

        # Prepare PID plots
        self.pidData = [[[] for _ in range(4)] for _ in range(3)]
        self.pidPlots = [self.win.addPlot(title="Roll pid"), self.win.addPlot(title="Pitch pid"), self.win.addPlot(title="Yaw pid")]
        self.pidLines = [[] for _ in range(3)]
        for w in range(len(self.pidPlots)):
            self.pidPlots[w].showGrid(x=False, y=True, alpha=0.5)
            self.pidPlots[w].addLegend()
            self.pidLines[w].append(self.pidPlots[w].plot(pen="red", name="P"))
            self.pidLines[w].append(self.pidPlots[w].plot(pen="green", name="I"))
            self.pidLines[w].append(self.pidPlots[w].plot(pen="blue", name="D"))
            self.pidLines[w].append(self.pidPlots[w].plot(pen="orange", name="Σ"))

        self.win.nextRow()

        # Prepare PWM plots
        self.pwmData = [[] for _ in range(4)]
        self.pwmPlots = self.win.addPlot(title="Engine PWMs", colspan=3)
        self.pwmPlots.addLegend()
        self.pwmlines = [
            self.pwmPlots.plot(pen="white", name="M1"),
            self.pwmPlots.plot(pen="red", name="M2"),
            self.pwmPlots.plot(pen="green", name="M3"),
            self.pwmPlots.plot(pen="blue", name="M4")]

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
            for i in range(len(self.eulerData)):
                if len(self.eulerData[i]) > self.plotcut:
                    self.eulerData[i].pop(0)
                self.eulerData[i].append(float(ls[i+2]))
                self.eulerDatalines[i].setData(self.eulerData[i])
        # EKFX - position related logging
        if ls[0] == "EKFX":
            if len(self.enuzData) > self.plotcut:
                self.enuzData.pop(0)
            self.enuzData.append(float(ls[2]))
            self.enuzDatalines.setData(self.enuzData)
        # PID - logs of pid values from pid controllers
        if ls[0] == "PID":
            # iterate over roll/pitch/yaw plots
            for p in range(len(self.pidData)):
                # iterate over pid values in one plot
                for l in range(len(self.pidData[p])):
                    if len(self.pidData[p][l]) > self.plotcut:
                        self.pidData[p][l].pop(0)
                    v = float(ls[2 + (p + 1) * 4 + l])
                    # There happens to be some huge values at the start so crop them
                    if -100 < v < 100:
                        self.pidData[p][l].append(v)
                    self.pidLines[p][l].setData(self.pidData[p][l])
        # PWM - percent of throttle on each engine
        if ls[0] == "PWM":
            for i in range(0, 4):
                if len(self.pwmData[i]) > self.plotcut:
                    self.pwmData[i].pop(0)
                v = float(ls[i+1])
                # autoscaling causes problems at start end the very end
                if v > self.pwmLow:
                    self.pwmData[i].append(v)
                self.pwmlines[i].setData(self.pwmData[i])


    def update(self):
        """Data acquisition routine"""

        # flush and dummy read
        self.ser.flushInput()
        processLine(self.ser.readline().decode("utf-8"))
        for i in range(5):
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
    cmdParse.add_argument("-c", "--cut", required=False, default=200, type=int)
    cmdParse.add_argument("-p", "--pwm", required=False, default=0.15, type=float)
    args = cmdParse.parse_args()

    try:
        ser = serial.Serial(args.port, baudrate=115200, timeout=1)
    except serial.SerialException:
        print(f"Error while opening f{args.port}")
        sys.exit(1)

    gui = lpgui(ser, args)

    signal.signal(signal.SIGINT, gui.guiStop)
    signal.signal(signal.SIGHUP, gui.guiStop)


# Run quadLive
main()


if __name__ == "__main__":
    pg.exec()
