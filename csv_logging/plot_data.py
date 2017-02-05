#!/usr/bin/env python
#
# You can install numpy and matplotlib via pip
#

import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook

data = np.genfromtxt('SHOOTER-LOGS-JRAD.csv', delimiter=',', names=True)

fig = plt.figure()

outputPlot = fig.add_subplot(3, 1, 1)
outputPlot.plot(data['time'], data['newOutput'], label='output')
outputPlot.set_ylim([7, 9])

errorPlot = fig.add_subplot(3, 1, 2)
errorPlot.plot(data['time'], data['error'], label='error')
errorPlot.set_ylim([-200, 200])

meanSquareErrorPlot = fig.add_subplot(3, 1, 3)
meanSquareErrorPlot.plot(data['time'], data['meanSquareError'], label='mean square error')
meanSquareErrorPlot.set_ylim([0, 600])

plt.show()
