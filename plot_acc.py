# Plot vehicle acceleration

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import glob

filter_order = 4
filter_cutoff = 2

trips = input("Trips (separated by commas): ").split(', ')

y_lo = None
y_hi = None

# Plot individual trips
for n in trips:
    plt.figure(figsize=(10, 8))
    plt.grid(True)
    
    files = glob.glob(f'csv/imu_raw_trip{n}_*_trim.csv')
    for file in files:
        data = np.loadtxt(file, delimiter=',', skiprows=1)
        data = data[1:, :]
        time = data[:, 0]
        uniform_time = np.linspace(min(time), max(time), num=len(time))
        data_columns = data[:, 1:4]
        uniform_data = np.interp(uniform_time, time, data_columns[:, 0])
        dt = np.mean(np.diff(uniform_time))
        b, a = signal.butter(filter_order, filter_cutoff, fs=1 / dt)
        filteredData = signal.filtfilt(b, a, uniform_data)
        plt.plot(time, filteredData)
    
    ax = plt.gca()
    y_limits = ax.get_ylim()
    y_lo = y_limits[0] if y_lo is None else min(y_limits[0], y_lo)
    y_hi = y_limits[1] if y_hi is None else max(y_limits[1], y_hi)

    plt.xlabel('Time')
    plt.ylabel('Acceleration X')
    plt.title(f'Trip {n}')
    plt.savefig(f'plots/acc_trip{n}.png', bbox_inches='tight', dpi=300)
    plt.close()

# Plot trip comparison
fig, axes = plt.subplots(1, len(trips), figsize=(27, 9))
for i, ax in enumerate(axes):
    n = trips[i]
    ax.grid(True)

    files = glob.glob(f'csv/imu_raw_trip{n}_*_trim.csv')
    for file in files:
        data = np.loadtxt(file, delimiter=',', skiprows=1)
        data = data[1:, :]
        time = data[:, 0]
        uniform_time = np.linspace(min(time), max(time), num=len(time))
        data_columns = data[:, 1:4]
        uniform_data = np.interp(uniform_time, time, data_columns[:, 0])
        dt = np.mean(np.diff(uniform_time))
        b, a = signal.butter(filter_order, filter_cutoff, fs=1 / dt)
        filteredData = signal.filtfilt(b, a, uniform_data)
        ax.plot(time, filteredData)

    ax.set_ylim([y_lo, y_hi])
    ax.set_xlabel('Time')
    ax.set_ylabel('Acceleration X')
    ax.set_title(f'Trip {n}')

plt.savefig(f"plots/acc_trip{''.join(trips)}.png", bbox_inches='tight', dpi=300)
plt.close()