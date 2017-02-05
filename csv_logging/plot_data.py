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

ax1 = fig.add_subplot(2, 1, 2)

ax1.plot(data['time'], data['error'], label='bounded_error')

ax2 = fig.add_subplot(2, 1, 1)

ax2.plot(data['time'], data['newOutput'], label='output')
ax2.set_ylim([0, 12])

plt.show()
