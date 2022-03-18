#!/usr/bin/env python3

import itertools
import math
import sys
import json
import numpy as np
import matplotlib.pyplot as plt

num_plots_down = 8
fig, axs = plt.subplots(num_plots_down, 2, constrained_layout=True,
        gridspec_kw={'height_ratios': [1.5, 0.6, 0.6, 0.3, 0.3, 0.3, 0.3, 0.3], 'width_ratios': [3, 1]})

axs[0, 0].set_xlabel("Elapsed time(s)")

data = None
with open("output.json") as fs:
    data = json.load(fs)

time = data["elapsed_time"]

def to_rpm(rads):
    return rads * 60.0 / (math.pi * 2.0)

axs[0, 0].plot(time, [to_rpm(vel) for vel in data["engine"]["angular_velocity"]], label="Engine velocity(rpm)")
axs[0, 0].plot(time, [to_rpm(vel) for vel in data["fl_wheel"]["angular_velocity"]], label="Fl Velocity(rpm)")
axs[0, 0].plot(time, [to_rpm(vel) for vel in data["fr_wheel"]["angular_velocity"]], label="Fr Velocity(rpm)")
axs[0, 0].plot(time, [to_rpm(vel) for vel in data["rl_wheel"]["angular_velocity"]], label="Rl Velocity(rpm)")
axs[0, 0].plot(time, [to_rpm(vel) for vel in data["rr_wheel"]["angular_velocity"]], label="Rr Velocity(rpm)")

axs[0, 0].plot(time, [s * 100 for s in data["fl_wheel"]["slip_ratio"]], label="Fl Slip Ratio(x100)")
axs[0, 0].plot(time, [s * 100 for s in data["fr_wheel"]["slip_ratio"]], label="Fr Slip Ratio(x100)")
axs[0, 0].plot(time, [s * 100 for s in data["rl_wheel"]["slip_ratio"]], label="Rl Slip Ratio(x100)")
axs[0, 0].plot(time, [s * 100 for s in data["rr_wheel"]["slip_ratio"]], label="Rr Slip Ratio(x100)")

axs[1, 0].plot(time, [math.sqrt(x * x + y * y) * 3.6 for (x, y) in itertools.zip_longest(data["velocity_x"], data["velocity_y"])], label="Velocity (km/h)")
axs[1, 0].plot(time, [x * 3.6 for x in data["velocity_x"]], label="Velocity x (km/h)")
axs[1, 0].plot(time, [y * 3.6 for y in data["velocity_y"]], label="Velocity y (km/h)")
axs[2, 0].plot(time, data["yaw_velocity"], label="Yaw rate")

axs[3, 0].plot(time, data["gear"], color="yellow", label="Gear")
axs[4, 0].plot(time, data["steering"], label="Steering")
axs[5, 0].plot(time, data["throttle"], 'tab:red', label="Throttle")
axs[6, 0].plot(time, data["brake"], 'tab:green', label="Brake")
axs[7, 0].plot(time, data["clutch"], 'tab:orange', label="Clutch")

axs[0, 1].plot(data["position_x"], data["position_y"], label="Position")

for i in range(num_plots_down):
    axs[i, 0].label_outer()
    axs[i, 0].legend()

for i in range(1, num_plots_down):
    axs[i, 1].set_visible(False)

plt.show()
