import os
import gmplot
import csv
import pandas as pd

api_key = ""

def get_stop(filename):
    df = pd.read_csv(f"csv/imu_raw_{filename}.csv", header=0)
    min_linear_acc_x = df['linear_acc_x'].min()
    row_with_min_linear_acc_x = df[df['linear_acc_x'] == min_linear_acc_x]
    return row_with_min_linear_acc_x.iloc[0]['t']

def read_csv(filename):
    t = []
    lat_data = []
    lon_data = []
    with open(f'csv/gnss_{filename}.csv', 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            t.append(float(row['t']))
            lat_data.append(float(row['latitude']))
            lon_data.append(float(row['longitude']))
    
    return t, lat_data, lon_data

def plot_gnss_single(latitude_list, longitude_list, filename):
    initial_location = (42.300710, -83.698257)
    # gmap = gmplot.GoogleMapPlotter(initial_location[0], initial_location[1], 18)
    gmap = gmplot.GoogleMapPlotter(initial_location[0], initial_location[1], 18, apikey=api_key)
    gmap.map_type = "satellite"
    
    gmap.scatter(latitude_list, longitude_list, size=0.1, marker=False, color='blue')
    gmap.draw(f'deliverables/{filename}.html')


def plot_gnss(latitude_list, longitude_list, ref_lat_list, ref_lon_list, filename, t, stop_time):
    initial_location = (42.300710, -83.698257)
    # gmap = gmplot.GoogleMapPlotter(initial_location[0], initial_location[1], 18)
    gmap = gmplot.GoogleMapPlotter(initial_location[0], initial_location[1], 18, apikey=api_key)
    gmap.map_type = "satellite"
    
    gmap.scatter(ref_lat_list, ref_lon_list, size=0.1, marker=False, color='blue')

    stop_idx = -1
    for i in range(len(t)):
        if t[i] >= stop_time:
            stop_idx = i
            break
    
    gmap.scatter(latitude_list[:stop_idx], longitude_list[:stop_idx], size=0.1, marker=False, color='green')
    gmap.scatter(latitude_list[stop_idx:], longitude_list[stop_idx:], size=0.1, marker=False, color='red')
    gmap.draw(f'deliverables/{filename}.html')

if __name__ == "__main__":
    bag_files = os.listdir("bags")

    for bag_file in bag_files:
        if bag_file == "path19":
            _, lat_data, lon_data = read_csv(bag_file)
            plot_gnss_single(lat_data, lon_data, bag_file)

    # _, ref_lat_data, ref_lon_data = read_csv('path01_loop2')
    # for bag_file in bag_files:
    #     if bag_file[0:5] == "route":
    #         stop_time = get_stop(bag_file)
    #         t, lat_data, lon_data = read_csv(bag_file)
    #         plot_gnss(lat_data, lon_data, ref_lat_data, ref_lon_data, bag_file, t, stop_time)