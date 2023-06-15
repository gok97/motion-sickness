import csv

import numpy as np
import matplotlib.pyplot as plt

import os
import matplotlib.pyplot as plt
import numpy as np

from scipy.signal import butter, sosfilt, sosfreqz

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


def plot_acceleration(filename):
    csv_file_path = f"./csv/{filename}.csv"
    with open(csv_file_path, 'r') as csv_file:
        reader = csv.reader(csv_file)
        next(reader)

        time = []
        acc_x, acc_y, acc_z = [], [], []

        for row in reader:
            t = float(row[0])
            acc_x.append(float(row[1]))
            acc_y.append(float(row[2]))
            acc_z.append(float(row[3]))

            time.append(t)

    plt.plot(time, acc_x)
    plt.plot(time, acc_y)
    plt.plot(time, acc_z)

    plt.legend(["acc_x", "acc_y", "acc_z"])

    plt.xlabel('Time')
    plt.ylabel('Acceleration')
    plt.title('Acceleration over Time')
    plt.grid(True)

    if not os.path.exists("./plots"):
        os.mkdir("./plots")

    plot_file_path = f"./plots/{filename}.png"
    plt.savefig(plot_file_path)
    # plt.show()
    plt.close()

    return acc_x, acc_y, acc_z


def plot_gps_vel(filename):
    csv_file_path = f"./csv/{filename}.csv"
    with open(csv_file_path, 'r') as csv_file:
        reader = csv.reader(csv_file)
        next(reader)  

        time = []
        vel_x, vel_y, vel_z = [], [], []

        for row in reader:
            t = float(row[0])
            vel_x.append(float(row[1]))
            vel_y.append(float(row[2]))
            vel_z.append(float(row[3]))

            time.append(t)

    plt.plot(time, vel_x)
    plt.plot(time, vel_y)
    plt.plot(time, vel_z)

    plt.legend(["vel_x",  "vel_y", "vel_z"])

    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.title('Velocity over Time')
    plt.grid(True)

    if not os.path.exists("./plots"):
        os.mkdir("./plots")

    plot_file_path = f"./plots/{filename}.png"
    plt.savefig(plot_file_path)
    # plt.show()
    plt.close()

    return vel_x, vel_y, vel_z


def deserialize_ros_messages_from_bag(bag_file_path, query_topic, filename):
    csv_file_path = f"./csv/{filename}.csv"
    # print(f"Deserializing {query_topic} ros messages from {bag_file_path} to {csv_file_path}...")
    with open(csv_file_path, 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        if query_topic == IMU_TOPIC:
            writer.writerow(['t', 'linear_acc_x', 'linear_acc_y', 'linear_acc_z', 'angular_vel_x', 'angular_vel_y', 'angular_vel_z'])
        elif query_topic == GPS_VEL_TOPIC:
            writer.writerow(['t', 'linear_vel_x', 'linear_vel_y', 'linear_vel_z', 'angular_vel_x', 'angular_vel_y', 'angular_vel_z'])

        # create reader instance and open for reading
        with Reader(f'{bag_file_path}') as reader:
            # iterate over messages
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == IMU_TOPIC and connection.topic == query_topic:
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    linear_acc_x = msg.linear_acceleration.x
                    linear_acc_y = msg.linear_acceleration.y
                    linear_acc_z = msg.linear_acceleration.z
                    angular_vel_x = msg.angular_velocity.x
                    angular_vel_y = msg.angular_velocity.y
                    angular_vel_z = msg.angular_velocity.z

                    writer.writerow([timestamp, linear_acc_x, linear_acc_y,
                                     linear_acc_z, angular_vel_x, angular_vel_y,
                                     angular_vel_z])
                    
                if connection.topic == GPS_VEL_TOPIC and connection.topic == query_topic:
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    linear_vel_x = msg.twist.linear.x
                    linear_vel_y = msg.twist.linear.y
                    linear_vel_z = msg.twist.linear.z
                    angular_vel_x = msg.twist.angular.x
                    angular_vel_y = msg.twist.angular.y
                    angular_vel_z = msg.twist.angular.z
                    
                    writer.writerow([timestamp, linear_vel_x, linear_vel_y,
                                     linear_vel_z, angular_vel_x, angular_vel_y,
                                     angular_vel_z])
                    
    csv_file.close()
                

def find_lowpass_cutoff(signal, sampling_rate = 100, cutoff_percentage = 0.9):
    fft = np.fft.fft(signal)
    freq_axis = np.fft.fftfreq(len(signal), 1 / sampling_rate)
    magnitude_spectrum = np.abs(fft)
    normalized_spectrum = magnitude_spectrum / np.max(magnitude_spectrum)
    plt.figure()
    plt.plot(freq_axis, normalized_spectrum)
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Normalized Magnitude Spectrum')
    plt.title('Magnitude Spectrum')
    plt.grid(True)
    plt.show()
    return  freq_axis, normalized_spectrum


def phaseless_lowpass_filter(data, cutoff_freq, sampling_freq, order, plot_freq_response=False):
    # Calculate filter coefficients
    nyquist_freq = .5 * sampling_freq
    normal_cutoff = cutoff_freq / nyquist_freq
    sos = butter(order, normal_cutoff, btype='low', output='sos')

    # Apply filter using sosfilt to get phaseless output
    filtered_data = sosfilt(sos, data)

    if plot_freq_response:
        # Frequency response of the filter
        w, h = sosfreqz(sos, worN=2000, fs=sampling_freq)

        # Convert magnitude to dB
        h_db = 20 * np.log10(abs(h))

        # Plot the frequency response
        fig, ax = plt.subplots()
        ax.semilogx(w, h_db)
        ax.set_title('Frequency Response')
        ax.set_xlabel('Frequency [Hz]')
        ax.set_ylabel('Magnitude [dB]')
        plt.grid(True, which="both", ls="-", color='0.65')
        plt.axvline(cutoff_freq, color='r')  # add vertical line at cutoff frequency
        plt.subplots_adjust(hspace=0.5)  # adjust spacing between subplots
        plt.show()

    return filtered_data


def read_bags():
    if not os.path.exists("./csv"):
        os.mkdir("./csv")

    bag_files = os.listdir("bags")
    for bag_file in bag_files:
        if "trip_straight" in bag_file:
            bag_file_path = f"./bags/{bag_file}"
            imu_raw_filename = f"imu_raw_{bag_file}"
            gps_vel_filename = f"gps_vel_{bag_file}"
            
            deserialize_ros_messages_from_bag(bag_file_path, IMU_TOPIC, imu_raw_filename)
            deserialize_ros_messages_from_bag(bag_file_path, GPS_VEL_TOPIC, gps_vel_filename)

            plot_acceleration(imu_raw_filename)
            plot_gps_vel(gps_vel_filename)


if __name__ == "__main__":
    IMU_TOPIC = "/vehicle/imu/data_raw"
    GPS_VEL_TOPIC = "/vehicle/gps/vel"
    read_bags()

# find_lowpass_cutoff(acc)
# filtered_acc = phaseless_lowpass_filter(acc, 1, 100, 2, plot_freq_response=True)