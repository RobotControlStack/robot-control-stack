from collections import deque
from itertools import product
from socket import AF_INET, SOCK_DGRAM, socket
from struct import unpack

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

host = "127.0.0.1"
port = 54322
FMT = "!" + 7 * "d" + "i"

buffsize = 30
# buffsize = 300
# Order of buffers: x, y, z, rx, ry, rz

titles = [["x", "rx"], ["y", "ry"], ["z", "rz"]]
colors = [["r", "r"], ["b", "b"], ["g", "g"]]
buffers = [[deque(maxlen=buffsize) for j in range(2)] for i in range(3)]
plots = [[None for j in range(2)] for i in range(3)]

fig, axs = plt.subplots(3, 2)
for i, j in product(range(3), range(2)):
    buffer = buffers[i][j]
    (plots[i][j],) = axs[i][j].plot(np.arange(len(buffer)), buffer, c=colors[i][j])
    axs[i][j].set_title(titles[i][j])
    axs[i][j].set_xticklabels([])

fig.tight_layout()
plt.show(block=False)


with socket(AF_INET, SOCK_DGRAM) as sock:
    sock.settimeout(2)
    sock.bind((host, port))
    while True:
        try:
            unpacked = unpack(FMT, sock.recv(7 * 8 + 4))
        except TimeoutError:
            continue

        last_controller_pose_raw = np.ctypeslib.as_array(unpacked[:7])
        rotation = last_controller_pose_raw[:4]
        translation = last_controller_pose_raw[4:]

        rot_q = R.from_quat(rotation)
        rot_euler = rot_q.as_euler("xyz", degrees=True)

        buffers[0][0].append(translation[0])
        buffers[0][1].append(rot_euler[0])
        buffers[1][0].append(translation[1])
        buffers[1][1].append(rot_euler[1])
        buffers[2][0].append(translation[2])
        buffers[2][1].append(rot_euler[2])

        for i, j in product(range(3), range(2)):
            buffer = buffers[i][j]
            plots[i][j].set_xdata(np.arange(len(buffer)))
            plots[i][j].set_ydata(buffer)
            axs[i][j].relim()
            axs[i][j].autoscale_view()
            axs[i][j].set_title(titles[i][j] + ": {:.2f}".format(buffer[-1]))

        plt.draw()
        plt.pause(0.001)
        plt.pause(0.5)
