import matplotlib.pyplot as plt
import numpy as np
import re
import sys

if len(sys.argv) != 3:
    print("Usage: python plot_throttle_steering.py <log_file.txt> <output_plot.png>")
    sys.exit(1)

log_file = sys.argv[1]
output_file = sys.argv[2]

# Data storage
timestamps = []
dV = []
V = []
th_cmd = []
dDelta = []
Delta = []
st_cmd = []

# Parse variables
kp_throttle = None
kd_throttle = None
ki_throttle = None
kp_steering = None
kd_steering = None

out_post = ""

with open(log_file, 'r') as f:
    for line in f:
        # Get timestamp
        time_match = re.match(r"\[(\d+\.\d+)\]", line)
        if not time_match:
            continue
        timestamp = float(time_match.group(1))

        # Parse controller gains
        if "Kp_th_" in line:
            kp_match = re.search(r"Kp_th_=\s*([\d\.\-e]+)", line)
            if kp_match:
                kp_throttle = float(kp_match.group(1))
        elif "Kd_th_" in line:
            kd_match = re.search(r"Kd_th_=\s*([\d\.\-e]+)", line)
            if kd_match:
                kd_throttle = float(kd_match.group(1))
        elif "Ki_th_" in line:
            ki_match = re.search(r"Ki_th_=\s*([\d\.\-e]+)", line)
            if ki_match:
                ki_throttle = float(ki_match.group(1))
        elif "Kp_st_" in line:
            kp_match = re.search(r"Kp_st_=\s*([\d\.\-e]+)", line)
            if kp_match:
                kp_steering = float(kp_match.group(1))
        elif "Kd_st_" in line:
            kd_match = re.search(r"Kd_st_=\s*([\d\.\-e]+)", line)
            if kd_match:
                kd_steering = float(kd_match.group(1))

        # Parse log data
        if "dV=" in line:
            dv_match = re.search(r"dV=([\d\.\-e]+) ,V=([\d\.\-e]+)", line)
            if dv_match:
                timestamps.append(timestamp)
                dV.append(float(dv_match.group(1)))
                V.append(float(dv_match.group(2)))
                th_cmd.append(None)
                dDelta.append(None)
                Delta.append(None)
                st_cmd.append(None)
        elif "dDelta=" in line:
            delta_match = re.search(r"dDelta=([\d\.\-e]+) ,Delta=([\d\.\-e]+)", line)
            if delta_match:
                timestamps.append(timestamp)
                dV.append(None)
                V.append(None)
                th_cmd.append(None)
                dDelta.append(float(delta_match.group(1)))
                Delta.append(float(delta_match.group(2)))
                st_cmd.append(None)
        elif "th_cmd=" in line:
            cmd_match = re.search(r"th_cmd=([\d\.\-e]+) ,st_cmd=([\d\.\-e]+)", line)
            if cmd_match:
                timestamps.append(timestamp)
                dV.append(None)
                V.append(None)
                th_cmd.append(float(cmd_match.group(1)))
                dDelta.append(None)
                Delta.append(None)
                st_cmd.append(float(cmd_match.group(2)))

# Helper to filter series
def filter_series(timestamps, series):
    return [t for t, v in zip(timestamps, series) if v is not None], [v for v in series if v is not None]

timestamps = np.array(timestamps)

# Plot
fig, axs = plt.subplots(4, 1, sharex=True, figsize=(12, 10))
title = "Low-Level Controller Debug\n"
if kp_throttle is not None:
    title += f"Kp_th = {kp_throttle:.3f} "
    out_post += f"_Kp_th_{kp_throttle:.3f}"
if kd_throttle is not None:
    title += f"Kd_th = {kd_throttle:.3f} "
    out_post += f"_Kd_th_{kd_throttle:.3f}"
if ki_throttle is not None:
    title += f"Ki_th = {ki_throttle:.3f}\n"
    out_post += f"_Ki_th_{ki_throttle:.3f}"
if kp_steering is not None:
    title += f"Kp_st = {kp_steering:.3f} "
    out_post += f"_Kp_st_{kp_steering:.3f}"
if kd_steering is not None:
    title += f"Kd_st = {kd_steering:.3f}"
    out_post += f"_Kd_st_{kd_steering:.3f}"
fig.suptitle(title.strip(), fontsize=14)

# 1. Desired vs current velocity
t1, dV_plot = filter_series(timestamps, dV)
_, V_plot = filter_series(timestamps, V)
axs[0].plot(t1, dV_plot, label="Desired Velocity")
axs[0].plot(t1, V_plot, label="Current Velocity")
axs[0].set_ylabel("Velocity (m/s)")
axs[0].legend()
axs[0].grid()

# 2. Throttle command
t2, th_plot = filter_series(timestamps, th_cmd)
axs[1].plot(t2, th_plot, label="Throttle Command", color='orange')
axs[1].set_ylabel("Throttle")
axs[1].legend()
axs[1].grid()

# 3. Desired vs current steering
t3, dDelta_plot = filter_series(timestamps, dDelta)
_, Delta_plot = filter_series(timestamps, Delta)
axs[2].plot(t3, dDelta_plot, label="Desired Steering")
axs[2].plot(t3, Delta_plot, label="Current Steering")
axs[2].set_ylabel("Steering (rad)")
axs[2].legend()
axs[2].grid()

# 4. Steering command
t4, st_plot = filter_series(timestamps, st_cmd)
axs[3].plot(t4, st_plot, label="Steering Command", color='green')
axs[3].set_ylabel("Steering Cmd")
axs[3].set_xlabel("Time (s)")
axs[3].legend()
axs[3].grid()

plt.tight_layout(rect=[0, 0, 1, 0.95])
#plt.show()

out_post += f".png"

output_file += out_post
plt.savefig(output_file)
print(f"Plot saved to {output_file}")
