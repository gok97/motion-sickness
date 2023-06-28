import csv

import numpy as np
import matplotlib.pyplot as plt

import os
import matplotlib.pyplot as plt
import numpy as np

from scipy.signal import butter, sosfilt, sosfreqz, savgol_filter

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

import folium

def plot_gnss(gnss_data, filename):
    initial_location = [42.300710, -83.698257]
    map_center = folium.Map(location=initial_location, zoom_start=18)
    for loc in gnss_data:
        _, latitude, longitude, _ = loc
        folium.CircleMarker(location=[latitude, longitude], radius=1, fill=True).add_to(map_center)
    map_center.save(f'plots/{filename}.html')


def plot_imu_acceleration(imu_data, filename):
    imu_data = np.array(imu_data)
    # time = (imu_data[:, 0] - imu_data[0, 0]) / (1e9)
    # print(time)
    time = imu_data[:, 0]
    # print(time.shape)
    acc_x = imu_data[:, 1]
    acc_y = imu_data[:, 2]
    acc_z = imu_data[:, 3]

    plt.plot(time, acc_x)
    plt.plot(time, acc_y)
    plt.plot(time, acc_z)

    plt.legend(["acc_x", "acc_y", "acc_z"])

    plt.xlabel('Time')
    plt.ylabel('Acceleration')
    plt.title('Acceleration over Time')
    plt.grid(True, linewidth=2)
    plt.minorticks_on()

    if not os.path.exists("./plots"):
        os.mkdir("./plots")

    plot_file_path = f"./plots/{filename}.png"
    plt.savefig(plot_file_path)
    # plt.show()
    plt.close()

    return acc_x, acc_y, acc_z


def plot_gps_vel(gps_vel_data, filename):
    gps_vel_data = np.array(gps_vel_data)
    # time = (gps_vel_data[:, 0] - gps_vel_data[0, 0]) / (1e9)
    time = gps_vel_data[:, 0]
    vel_x = gps_vel_data[:, 1]
    vel_y = gps_vel_data[:, 2]
    vel_z = gps_vel_data[:, 3]

    plt.plot(time, vel_x)
    plt.plot(time, vel_y)
    plt.plot(time, vel_z)

    plt.legend(["vel_x",  "vel_y", "vel_z"])

    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.title('Velocity over Time')
    plt.grid(True, linewidth=2)
    plt.minorticks_on()

    if not os.path.exists("./plots"):
        os.mkdir("./plots")

    plot_file_path = f"./plots/{filename}.png"
    plt.savefig(plot_file_path)
    # plt.show()
    plt.close()

    return vel_x, vel_y, vel_z


def plot_odom_vel(odom_data, filename):
    odom_data = np.array(odom_data)
    time = odom_data[:, 0]
    # print(time.shape)
    time = odom_data[:, 0]
    vel_x = odom_data[:, 1]
    vel_y = odom_data[:, 2]
    vel_z = odom_data[:, 3]

    plt.plot(time, vel_x)
    plt.plot(time, vel_y)
    plt.plot(time, vel_z)

    plt.legend(["vel_x",  "vel_y", "vel_z"])

    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.title('Odom Velocity over Time')
    plt.grid(True, linewidth=2)
    plt.minorticks_on()

    if not os.path.exists("./plots"):
        os.mkdir("./plots")

    plot_file_path = f"./plots/{filename}.png"
    plt.savefig(plot_file_path)
    # plt.show()
    plt.close()

    return vel_x, vel_y, vel_z


def plot_imu_accleration_combined(all_imu_data, all_trim_time, file_names):
    num_of_data = len(all_imu_data)
    
    for acc_idx, acc_type in enumerate(["accleration_x", "accleration_y", "accleration_z"]):
        # plot each acc x, y and z in different graphs
        fig, ax = plt.subplots(nrows=1, ncols=1)
        for idx in range(num_of_data):
            imu_data = np.array(all_imu_data[idx])
            time = imu_data[:, 0]
            start_data_index = np.where(time<=all_trim_time[idx]-2)[0][-1]
            # start_data_point = np.where(time >= TIME_TRIM[file_names[idx]]-2)[0][0]
            acc = imu_data[start_data_index:, acc_idx+1]

            ax.plot(acc, label=file_names[idx], linewidth=0.5)
            ax.set_title(acc_type)

        lines = []
        labels = []
        for ax in fig.axes:
            line, label = ax.get_legend_handles_labels()
            lines.append(line)
            labels.append(label)

        fig.legend(loc='outside upper right')
        fig.suptitle('IMU Accleration over Time')
        plt.grid(True, linewidth=2)
        # plt.xlim([1500, 2500])
        plt.minorticks_on()
        plot_file_path = f"./plots/imu_{acc_type}_combined.png"
        fig.savefig(plot_file_path)
        # plt.show()
        plt.close()


def deserialize_ros_messages_from_bag(bag_file_path, query_topic, filename):
    csv_file_path = f"./csv/{filename}.csv"
    # print(f"Deserializing {query_topic} ros messages from {bag_file_path} to {csv_file_path}...")
    with open(csv_file_path, 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        if query_topic == IMU_TOPIC:
            writer.writerow(['t', 'linear_acc_x', 'linear_acc_y', 'linear_acc_z', 'angular_vel_x', 'angular_vel_y', 'angular_vel_z'])
        elif query_topic == GPS_VEL_TOPIC:
            writer.writerow(['t', 'linear_vel_x', 'linear_vel_y', 'linear_vel_z', 'angular_vel_x', 'angular_vel_y', 'angular_vel_z'])
        elif query_topic == ODOM_TOPIC:
            writer.writerow(['t', 'linear_vel_x', 'linear_vel_y', 'linear_vel_z', 'angular_vel_x', 'angular_vel_y', 'angular_vel_z'])
        elif query_topic == GNSS_TOPIC:
            writer.writerow(['t', 'latitude', 'longitude', 'altitude'])

        dbw_enabled = True # set this to False when you want start data entry to csv after DBW is enabled
        vehicle_moving = False
        vehicle_stop = False
        start_data_entry = True
        data = []

        initial_timestamp = None

        # create reader instance and open for reading
        with Reader(f'{bag_file_path}') as reader:
            # iterate over messages
            for connection, timestamp, rawdata in reader.messages():
                if initial_timestamp is None:
                    initial_timestamp = timestamp
                if connection.topic == IMU_TOPIC and connection.topic == query_topic\
                      and start_data_entry:
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    linear_acc_x = msg.linear_acceleration.x
                    linear_acc_y = msg.linear_acceleration.y
                    linear_acc_z = msg.linear_acceleration.z
                    angular_vel_x = msg.angular_velocity.x
                    angular_vel_y = msg.angular_velocity.y
                    angular_vel_z = msg.angular_velocity.z
                    t = (timestamp - initial_timestamp) / 1e9

                    writer.writerow([t, linear_acc_x, linear_acc_y,
                                     linear_acc_z, angular_vel_x, angular_vel_y,
                                     angular_vel_z])
                    data.append([t, linear_acc_x, linear_acc_y,
                                 linear_acc_z, angular_vel_x, angular_vel_y,
                                 angular_vel_z])
                    
                if connection.topic == GPS_VEL_TOPIC and connection.topic == query_topic\
                    and start_data_entry:
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    linear_vel_x = msg.twist.linear.x
                    linear_vel_y = msg.twist.linear.y
                    linear_vel_z = msg.twist.linear.z
                    angular_vel_x = msg.twist.angular.x
                    angular_vel_y = msg.twist.angular.y
                    angular_vel_z = msg.twist.angular.z
                    t = (timestamp - initial_timestamp) / 1e9

                    writer.writerow([t, linear_vel_x, linear_vel_y,
                                     linear_vel_z, angular_vel_x, angular_vel_y,
                                     angular_vel_z])
                    data.append([t, linear_vel_x, linear_vel_y,
                                 linear_vel_z, angular_vel_x, angular_vel_y,
                                 angular_vel_z])
                    
                if connection.topic == DBW_TOPIC and not dbw_enabled:
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    dbw_enabled = msg.data

                if connection.topic == ODOM_TOPIC and connection.topic == query_topic\
                    and start_data_entry:
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    linear_vel_x = msg.twist.twist.linear.x
                    linear_vel_y = msg.twist.twist.linear.y
                    linear_vel_z = msg.twist.twist.linear.z
                    angular_vel_x = msg.twist.twist.angular.x
                    angular_vel_y = msg.twist.twist.angular.y
                    angular_vel_z = msg.twist.twist.angular.z
                    t = (timestamp - initial_timestamp) / 1e9

                    if not start_data_entry:
                        if linear_vel_x <= 0.02:
                            vehicle_stop = True
                        else:
                            vehicle_moving = True
                        start_data_entry = vehicle_stop and vehicle_moving # the point when vehicle moves from rest

                    if start_data_entry:
                        writer.writerow([t, linear_vel_x, linear_vel_y,
                                        linear_vel_z, angular_vel_x, angular_vel_y,
                                        angular_vel_z])
                        data.append([t, linear_vel_x, linear_vel_y,
                                     linear_vel_z, angular_vel_x, angular_vel_y,
                                     angular_vel_z])
                        
                if connection.topic == GNSS_TOPIC and connection.topic == query_topic:
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    lat = msg.latitude
                    lon = msg.longitude
                    alt = msg.altitude
                    t = (timestamp - initial_timestamp) / 1e9
                    writer.writerow([t, lat, lon, alt])
                    data.append([t, lat, lon, alt])

    csv_file.close()
    
    return data


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
    filtered_data = sosfilt(sos, data[:, 1])

    dx = np.diff(data[:, 1]) / np.diff(data[:, 0])
    dt = (data[:, 0][:-1] + data[:, 0][1:]) / 2
    
    if not plot_freq_response:
        plt.plot(data[:, 0], data[:, 1])
        plt.plot(data[:, 0], filtered_data)
        plt.plot(dt, dx)
        plt.show()

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


def plot_savgol_filter(data, window_length, polyorder, deriv, delta):
    filtered_data = savgol_filter(x=data[:, 1],
                                  window_length=window_length,
                                  polyorder=polyorder,
                                  deriv=deriv,
                                  delta=delta)
    filtered_data_first_order = savgol_filter(x=data[:, 1],
                                             window_length=window_length,
                                             polyorder=polyorder,
                                             deriv=deriv+1,
                                             delta=delta)
    filtered_data_second_order = savgol_filter(x=data[:, 1],
                                               window_length=window_length,
                                               polyorder=polyorder,
                                               deriv=deriv+2,
                                               delta=delta)

    plt.plot(data[:, 0], data[:, 1])
    plt.plot(data[:, 0], filtered_data)
    plt.plot(data[:, 0], filtered_data_first_order)
    plt.plot(data[:, 0], filtered_data_second_order)
    plt.show()


def get_trim_time(odom_data):
    # set very low x velocities to 0 (noise)
    vel_x = odom_data[:, 1]
    vel_x[vel_x < 0.01] = 0

    # find peak x velocity index
    max_idx = np.argmax(vel_x)
    
    # find 0 velocity indices
    zero_indices = np.where(vel_x==0)[0]
    # find the largest 0 velocity index
    # that is smaller than peak x velocity index
    trim_index = zero_indices[zero_indices<max_idx][-1]
    trim_time = odom_data[:, 0][trim_index]

    return trim_time


def trim_csv(all_trim_time, bags_list):
    for idx, file in enumerate(bags_list):
        imu_raw_filename = f"./csv/imu_raw_{file}.csv"
        gps_vel_filename = f"./csv/gps_vel_{file}.csv"
        odom_vel_filename = f"./csv/odom_vel_{file}.csv"
        files = [imu_raw_filename, gps_vel_filename, odom_vel_filename]
        for f in files:
            lines = []
            with open(f, 'r') as csv_file:
                reader = csv.reader(csv_file)
                header = None
                for row in reader:
                    if header is None:
                        header = row
                        lines.append(header)
                        continue
                    t = float(row[0])
                    if t >= all_trim_time[idx]:
                        lines.append(row)
            
            with open(f.replace(".csv", "_trim.csv"), 'w') as write_file:
                writer = csv.writer(write_file)
                writer.writerows(lines)

            write_file.close()


def read_and_plot_bags():
    if not os.path.exists("./csv"):
        os.mkdir("./csv")

    bag_files = os.listdir("./bags/umtri_recorded_0623")
    for bag_file in bag_files:
        if bag_file in ["straight_trip11_3", "straight_trip11_4"]:
            continue
        bag_file_path = f"./bags/umtri_recorded_0623/{bag_file}"
        imu_raw_filename = f"imu_raw_{bag_file}"
        gps_vel_filename = f"gps_vel_{bag_file}"
        odom_vel_filename = f"odom_vel_{bag_file}"
        gnss_filename = f"gnss_{bag_file}"
        
        print(f"Parsing {bag_file_path}\n")
        imu_data = deserialize_ros_messages_from_bag(bag_file_path, IMU_TOPIC, imu_raw_filename)
        gps_vel_data = deserialize_ros_messages_from_bag(bag_file_path, GPS_VEL_TOPIC, gps_vel_filename)
        odom_data = deserialize_ros_messages_from_bag(bag_file_path, ODOM_TOPIC, odom_vel_filename)
        gnss_data = deserialize_ros_messages_from_bag(bag_file_path, GNSS_TOPIC, gnss_filename)

        # filtered_acc = phaseless_lowpass_filter(np.array(imu_data)[:, :2], 1, 100, 2, plot_freq_response=False)
        # plot_savgol_filter(np.array(odom_data)[:, :2], 75, 2, 0, 1 / 100)
        plot_gps_vel(np.array(gps_vel_data), gps_vel_filename)
        plot_odom_vel(np.array(odom_data), odom_vel_filename)
        plot_imu_acceleration(np.array(imu_data), imu_raw_filename)
        plot_gnss(np.array(gnss_data), gnss_filename)


def read_and_plot_combined_bags(bags):
    bag_files = os.listdir("./bags/umtri_recorded_0623")
    all_trim_time = []
    all_imu_data = []
    bags_list = []
    for bag_file in bag_files:
        if bag_file in bags:
            bag_file_path = f"./bags/umtri_recorded_0623/{bag_file}"
            imu_raw_filename = f"imu_raw_{bag_file}"
            gps_vel_filename = f"gps_vel_{bag_file}"
            odom_vel_filename = f"odom_vel_{bag_file}"
            
            print(f"Parsing {bag_file_path}\n")
            imu_data = deserialize_ros_messages_from_bag(bag_file_path, IMU_TOPIC, imu_raw_filename)
            gps_vel_data = deserialize_ros_messages_from_bag(bag_file_path, GPS_VEL_TOPIC, gps_vel_filename)
            odom_data = deserialize_ros_messages_from_bag(bag_file_path, ODOM_TOPIC, odom_vel_filename)
            # filtered_imu_data = phaseless_lowpass_filter(np.array(imu_data)[:, :2], 1, 100, 2, plot_freq_response=False)
            
            all_trim_time.append(get_trim_time(odom_data=np.array(odom_data)))
            all_imu_data.append(imu_data)
            bags_list.append(bag_file)
    
    plot_imu_accleration_combined(all_imu_data, all_trim_time, bags_list)
    trim_csv(all_trim_time, bags_list)
    # plot_imu_accleration_combined(all_imu_data, ["trip9_1", "trip10_1", "trip11_1", "trip12_1"])


if __name__ == "__main__":
    IMU_TOPIC = "/vehicle/imu/data_raw"
    GPS_VEL_TOPIC = "/vehicle/gps/vel"
    DBW_TOPIC = "/vehicle/dbw_enabled"
    ODOM_TOPIC = "/vehicle/odom"
    GNSS_TOPIC = "/ins_fix"

    TIME_TRIM = {
                "new_straightaway_1": 9.7,
                "new_straightaway_2": 7.0,
                "new_straightaway_3": 6.5,
                "new_straightaway_4": 5.0,
                }
    # deserialize_ros_messages_from_bag("./bags/trip_straight_5", DBW_TOPIC, "dbw_trip_straight_5")

    read_and_plot_combined_bags(["straight_trip10_1", "straight_trip11_1", "straight_trip12_1"])
    # read_and_plot_bags()

# find_lowpass_cutoff(acc)
# filtered_acc = phaseless_lowpass_filter(acc, 1, 100, 2, plot_freq_response=True)
