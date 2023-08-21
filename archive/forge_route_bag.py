from rosbags.rosbag2 import Writer, Reader
from rosbags.serde import serialize_cdr, deserialize_cdr
import csv

def create_ros2_bag(csv_file_path):
    topic_name = "/ins_fix"

    with Reader('bags/path00') as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic_name:
                msg_type = connection.msgtype
                msg = deserialize_cdr(rawdata, msg_type)
                break

    with Writer('./forged_loop') as bag_writer:
        connection = bag_writer.add_connection(topic_name, msg_type, 'cdr', '')
        with open(csv_file_path, "r") as csvfile:
            csvreader = csv.reader(csvfile)
            header = next(csvreader)  # Skip the header row
            for row in csvreader:
                t, lat, lon, alt = map(float, row)
                msg.latitude = lat
                msg.longitude = lon
                msg.altitude = alt
                cdr_data = serialize_cdr(msg, 'sensor_msgs/msg/NavSatFix')
                bag_writer.write(connection, t, cdr_data)

if __name__ == "__main__":
    csv_file_path = "forged_loop.csv"
    create_ros2_bag(csv_file_path)
