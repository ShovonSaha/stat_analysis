#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import pcl
import matplotlib.pyplot as plt

first_col_avg = []
last_col_avg = []

first_col_std = []
last_col_std = []

mid_sec_avg = []
mid_sec_std = []

first_row_avg = []
last_row_avg = []

first_row_std = []
last_row_std = []

def fill_crop_array(array, desired_length):
    if len(array) < desired_length:
        last_element = array[-1]
        while len(array) < desired_length:
            array.append(last_element)
    elif len(array) > desired_length:
        array = array[:desired_length]
    return array

def pointcloud_callback(data):
    # Create a generator to iterate over the points in the point cloud
    gen = pc2.read_points(data, skip_nans=True)
        
    points_list_x = []
   
    for data in gen:
        # points_list.append([data[0], data[1], data[2], data[3]])
        # data[0] gives the points in X axis; the depth that we are interested in
        
        # Only inputting the non-zero numbers into the list
        if data[0]!= 0: 
            # print("Non-Zero")
            points_list_x.append(data[0])
    
    # print(points_list_x)
    len_points = len(points_list_x)
    # print(len_points)
    
    desired_rows = 60 # Keeping this fixed
    desired_columns = len_points // desired_rows
    # print("Desired Col: ", desired_columns)

    desired_length = desired_columns*desired_rows
    # print("Desired Length: ", desired_length)
    
    
    # div = 6 # Division for both vertical and horizontal

    # Checking the size of the array to fill / crop
    modified_list = fill_crop_array(points_list_x, desired_length)

    # print(modified_list)
    
    # Converting the list to an array
    points_array = np.array(modified_list)
        
    # print(points_array[:10])

    # Converting from metre (m) to centimetres (cm)
    points_array = points_array * 100
    
    # Before reshaping the matrix:
    # print("Before reshaping: ", points_array)
    # Print the length before reshaping the matrix
    # print("Before reshaping: ", len(points_array))
       
    # Reshape the matrix to desired_rows by desired_columns
    reshaped_array = np.reshape(points_array, (desired_rows, desired_columns))
    # print("After reshaping: ", len(reshaped_array))
    # After reshaping the matrix:
    # print("After reshaping: ", reshaped_array)

    # Number of elements in points_array
    num_elements_points_array = points_array.size

    # Number of elements in reshaped_array
    num_elements_reshaped_array = reshaped_array.size

    # Print the results
    print("Number of elements in points_array:", num_elements_points_array)
    print("Number of elements in reshaped_array:", num_elements_reshaped_array)

    # Revising the grid system
    # Initializing boundaries as percentages
    a = 0.1 # percentage
    b = 0.1
    first_div = int(desired_columns * a)
    first_col = reshaped_array[:, :first_div]
    last_col = reshaped_array[:, -first_div:]
    
    mid_sec = reshaped_array[first_div:-first_div, first_div:-first_div]
    print("Total number of points in mid-section: ", mid_sec.size)

    first_row = reshaped_array[:first_div, :]
    last_row = reshaped_array[-first_div:, :]
    
    # Middle Section
    mid_sec_avg.append(np.mean(mid_sec))
    mid_sec_std.append(np.std(mid_sec))

    # First and Last Columns
    first_col_avg.append(np.mean(first_col))
    last_col_avg.append(np.mean(last_col))

    first_col_std.append(np.std(first_col))
    last_col_std.append(np.std(last_col))

    # First and Last Rows
    first_row_avg.append(np.mean(first_row))
    last_row_avg.append(np.mean(last_row)) 

    first_row_std.append(np.std(first_row))
    last_row_std.append(np.std(last_row))
    

def listener():
    # Initialize the node
    rospy.init_node('pointcloud_subscriber', anonymous=True)

    # Subscribe to the 'pointcloud' topic
    rospy.Subscriber("/scan_3D", PointCloud2, pointcloud_callback)

    # Spin until Ctrl+C is pressed
    rospy.spin()

if __name__ == '__main__':
    listener()
    