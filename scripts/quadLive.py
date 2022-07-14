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

from pdb import line_prefix
import numpy as np

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from sqlalchemy import except_all
import serial
import time
import sys



#### CONFIGS ####

PORTPATH = "/dev/ttyUSB0" # default serial port to access logs
PLOTCUT = 200              # amount of samples to be shown at once on screen
PWM_LOW = 0.15

#################

# catch port name
if (len(sys.argv) > 1 and sys.argv[1] is not None):
    PORTPATH = str(sys.argv[1])
    print(f"reading from {PORTPATH}")


def processLine(l):
    l = l.replace(":","")
    l = l.replace(",","")
    l = l.replace("\\r\\n'", "")
    return l.split()

app = pg.mkQApp("My multiple plot")

win = pg.GraphicsLayoutWidget(show=True, title="Phoenix-pilot logging tool")
win.resize(1500,800)
win.setWindowTitle('Phoenix-pilot logging tool')
pg.setConfigOptions(antialias=True)

win.nextRow()

y = [[], [], []]
py = win.addPlot(title="Yaw", colspan=1)
py.addLegend()

# Prepare roll/pitch plot
e = [[], [], []]
p2 = win.addPlot(title="Roll & Pitch", colspan=2)
p2.addLegend()
elines = [py.plot(pen="red", name="yaw"), p2.plot(pen="green", name="pitch"), p2.plot(pen="blue", name="roll")]

win.nextRow()

# Prepare PID plots
pids = [[[],[],[],[]], [[],[],[],[]], [[],[],[],[]]]
winpids = [win.addPlot(title="Roll pid"), win.addPlot(title="Pitch pid"), win.addPlot(title="Yaw pid")]
linepids = [[],[],[]]
for w in range(0, 3):
    winpids[w].showGrid(x = False, y = True, alpha = 0.5)
    winpids[w].addLegend()
    linepids[w].append(winpids[w].plot(pen="red", name="P"))
    linepids[w].append(winpids[w].plot(pen="green", name="I"))
    linepids[w].append(winpids[w].plot(pen="blue", name="D"))
    linepids[w].append(winpids[w].plot(pen="orange", name="Σ"))

win.nextRow()

# Prepare PWM plots
pwms = [[], [], [], []]
winpwm = win.addPlot(title="Engine PWMs", colspan=3)
winpwm.addLegend()
pwmlines = [winpwm.plot(pen="white", name="M4"), winpwm.plot(pen="red", name="M1"), winpwm.plot(pen="green", name="M3"), winpwm.plot(pen="blue", name="M2")]

def digestIntoData(ls):
    """Interpret chopped line of data from quadrocontrol, and decompose it into plot arrays"""

    global q, p1, qlines
    global e, p2, elines
    global PLOTCUT, PWM_LOW
    if len(ls) > 1:
            # EKFE - ekf log of Euler angles
            if ls[0] == "b'EKFE":
                try:
                    for i in range(0,3):
                        if len(e[i]) > PLOTCUT:
                            e[i].pop(0)
                        e[i].append(float(ls[i+2]))
                        elines[i].setData(e[i])
                except:
                    None
            # PID - logs of pid values from pid controllers
            if ls[0] == "b'PID":
                try:
                    # iterate over roll/pitch/yaw plots
                    for p in range(0, 3):
                        # iterate over pid values in one plot
                        for l in range(0, 4):
                            if len(pids[p][l]) > PLOTCUT:
                                pids[p][l].pop(0)
                            v = float(ls[2 + (p + 1) * 4 + l])
                            # There happens to be some huge values at the start so crop them
                            if v < 100 and v > -100:
                                pids[p][l].append(v)
                            linepids[p][l].setData(pids[p][l])
                except:
                    None
            # PWM - percent of throttle on each engine
            if ls[0] == "b'PWM":
                try:
                    for i in range(0, 4):
                        if len(pwms[i]) > PLOTCUT:
                            pwms[i].pop(0)
                        v = float(ls[i+1])
                        # autoscaling causes problems at start end the very end
                        if v > PWM_LOW:
                            pwms[i].append(v)
                        pwmlines[i].setData(pwms[i])
                except:
                    None

ser = serial.Serial(PORTPATH, baudrate=115200, timeout=1)
def update():
    """Data acquisition routine"""

    global ser
    global q, p1, qlines
    global e, p2, elines
    ser.flushInput()
    ls1 = processLine(str(ser.readline()))
    ls1 = processLine(str(ser.readline()))
    ls2 = processLine(str(ser.readline()))
    ls3 = processLine(str(ser.readline()))
    ls4 = processLine(str(ser.readline()))
    digestIntoData(ls1)
    digestIntoData(ls2)
    digestIntoData(ls3)
    digestIntoData(ls4)

# Quadrocontrol has maximum frequency of 100Hz (with calculation time excluded) so with 10ms wait we should catch all data
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)


if __name__ == '__main__':
    pg.exec()
