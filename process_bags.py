# Extract vehicle data into csv

import csv

import numpy as np
import matplotlib.pyplot as plt

import os
import matplotlib.pyplot as plt
import numpy as np

from scipy.signal import butter, sosfilt, sosfreqz, savgol_filter

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


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
                    if t >= all_trim_time[idx] - 2:
                        row[0] = str(t - all_trim_time[idx])
                        lines.append(row)
            
            with open(f.replace(".csv", "_trim.csv"), 'w') as write_file:
                writer = csv.writer(write_file)
                writer.writerows(lines)

            write_file.close()

def read_bags():
    bag_files = os.listdir("./bags")
    all_trim_time = []
    bags_list = []
    for bag_file in bag_files:
        if bag_file[0] != '.':
            bag_file_path = f"./bags/{bag_file}"
            imu_raw_filename = f"imu_raw_{bag_file}"
            gps_vel_filename = f"gps_vel_{bag_file}"
            odom_vel_filename = f"odom_vel_{bag_file}"
            gnss_filename = f"gnss_{bag_file}"
            
            print(f"Parsing {bag_file_path}\n")
            imu_data = deserialize_ros_messages_from_bag(bag_file_path, IMU_TOPIC, imu_raw_filename)
            gps_vel_data = deserialize_ros_messages_from_bag(bag_file_path, GPS_VEL_TOPIC, gps_vel_filename)
            odom_data = deserialize_ros_messages_from_bag(bag_file_path, ODOM_TOPIC, odom_vel_filename)     
            gnss_data = deserialize_ros_messages_from_bag(bag_file_path, GNSS_TOPIC, gnss_filename)       
            all_trim_time.append(get_trim_time(odom_data=np.array(odom_data)))
            bags_list.append(bag_file)
    
    trim_csv(all_trim_time, bags_list)


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

    read_bags()
