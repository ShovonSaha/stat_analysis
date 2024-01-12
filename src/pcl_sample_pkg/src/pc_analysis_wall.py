#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import pcl
import matplotlib.pyplot as plt

# from pcl.filters import CropBox

# Initializations
# max_mean_first_col = 0
# max_std_first_col = 0

# max_mean_last_col = 0
# max_std_last_col = 0

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



# def find_nonzero_zero_indices(my_tuple):
#     nonzero_index = None
#     zero_index = None

#     for index, element in enumerate(my_tuple):
#         if element != 0 and nonzero_index is None:
#             nonzero_index = index
#         elif element == 0 and nonzero_index is not None and zero_index is None:
#             zero_index = index

#     return nonzero_index, zero_index

# def modify_array(array, desired_rows):

def fill_crop_array(array, desired_length):
    if len(array) < desired_length:
        last_element = array[-1]
        while len(array) < desired_length:
            array.append(last_element)
    elif len(array) > desired_length:
        array = array[:desired_length]
    return array

# # Example usage
# my_array = [1, 2, 3, 4, 5, 6, 7]
# desired_length = 10

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
    
    # d(cm)	Points	Col.  Vert.Divs
    # 25	7200	120		20
    # 50	7200	120		20
    # 75	6120	102		17
    # 100	5400	90		15
    # 125	4320	72		12
    # 150	3600	60		10
    # 175	2880	48		8
    # 200	2520	42		7
    # desired_length = 2520 #Keeping 60 rows fixed, just adjusting the columns
    # desired_columns = 42
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
    
    # print("First 10% of columns: ")
    # print(first_col)
    # print("Last 10% of columns: ")
    # print(last_col)
    
    # print("First 10% of rows: ")
    # print(first_row)
    # print("Last 10% of rows: ")
    # print(last_row)

    # Calculate the mean values
    # first_col_avg = np.mean(first_col, axis=0)   
    # axis = 0, 
    # This calculates the mean along the columns of the last_columns array. 
    # The resulting last_columns_mean will have a shape corresponding to the number of columns in last_columns. 
    # This is useful when you want to calculate the mean for each column individually.
    # global first_col_avg
    # global last_col_avg

    # global first_row_avg
    # global last_row_avg

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
    
    # print("mid section ", np.mean(mid_sec_avg))
    # print("mid_sec_std ", np.mean(mid_sec_std))

    # print("mean_first_col ", np.mean(first_col_avg))
    # print("std_first_col ", np.mean(first_col_std))

    # print("mean_last_col ", np.mean(last_col_avg))
    # print("std_last_col ", np.mean(last_col_std))

    # print("mean_first_row ", np.mean(first_row_avg))
    # print("std_first_row ", np.mean(first_col_std))
    
    # print("mean_last_row ", np.mean(last_row_avg))
    # print("std_last_row ", np.mean(last_col_std))

    # print(np.mean(mid_sec_avg))
    # print(np.mean(mid_sec_std))

    # print(np.mean(first_col_avg))
    # print(np.mean(first_col_std))

    # print(np.mean(last_col_avg))
    # print(np.mean(last_col_std))

    # print(np.mean(first_row_avg))
    # print(np.mean(first_col_std))
    
    # print(np.mean(last_row_avg))
    # print(np.mean(last_col_std))

    # print("////////////////////////////////////////////////")
    
    # print("First Col: ", first_col_avg)
    # print("Last Col: ", last_col_avg)

    # first_row_avg = np.mean(first_row)
    # last_row_avg = np.mean(last_row)

    # Calculate the standard deviations
    # global first_col_std
    # global last_col_std

    # first_std = np.std(first_col)
    # last_std = np.std(last_col)

    # first_col_std = np.append(first_col_std, first_std)
    # last_col_std = np.append(last_col_std, last_std)

    # # Initializing global variables
    # global max_mean_first_col
    # global max_std_first_col

    # global max_mean_last_col
    # global max_std_last_col

    # # For first columns
    # if first_col_avg > max_mean_first_col:
    #     max_mean_first_col = first_col_avg

    # if first_col_std > max_std_first_col:
    #     max_std_first_col = first_col_std

    # # For last columns
    # if last_col_avg > max_mean_last_col:
    #     max_mean_last_col = last_col_avg

    # if last_col_std > max_std_last_col:
    #     max_std_last_col = last_col_std
 
    # print("max_mean_first_col ", max_mean_first_col)
    # print("max_std_first_col ", max_std_first_col)
    # print("max_mean_last_col ", max_mean_last_col)
    # print("max_std_last_col ", max_std_last_col)

    # # Create x-axis values
    # x = np.arange(1, first_div + 1)

    # Plot the mean values with error bars representing the standard deviations
    # plt.errorbar(x, first_col_avg[:len(x)], yerr=first_col_std[:len(x)], label='First Columns')
    # plt.errorbar(x, last_col_avg[:len(x)], yerr=last_col_std[:len(x)], label='Last Columns')
    
    # plt.errorbar(range(len(averages)), averages, yerr=std_dev, fmt='o')
    # plt.errorbar(first_col_avg, yerr=first_col_std, fmt='o')
    # plt.errorbar(last_col_avg, yerr=last_col_std, fmt='o')
    
    # # Plot the mean values with error bars representing the standard deviations
    # plt.errorbar(x, [first_col_avg] * len(x), yerr=first_col_std, label='First Columns', fmt='o', capsize=3)
    # plt.errorbar(x, [last_col_avg] * len(x), yerr=last_col_std, label='Last Columns', fmt='o', capsize=3)

    # # Add labels and legend
    # plt.xlabel('Column')
    # plt.ylabel('Value')
    # plt.legend()

    # # Show the plot
    # plt.show()

    # # Plot the average values
    # plt.plot(averages)

    # # Add labels and title to the plot
    # plt.xlabel('Small Matrix Index')
    # plt.ylabel('Average Value')
    # plt.title('Average Values of Small Matrices')

    # # Display the plot
    # plt.show()

    # # Calculate the standard deviation of the mean values
    # std_dev = np.std(averages)

    # # Plot the mean values and standard deviation
    # plt.errorbar(range(len(averages)), averages, yerr=std_dev, fmt='o')

    # # Add labels and title to the plot
    # plt.xlabel('Grid Index')
    # plt.ylabel('Mean Value (cm)')
    # plt.title('Mean Values with Standard Deviation')

    # # Display the plot
    # plt.show()


    # grids = []

    # # for i in range(desired_columns):
    # #     for j in range(rows):

    # # Define the size of each small matrix
    # small_size = (6, 6)

    # # Split the large matrix into small matrices
    # small_matrices = []
    # for i in range(0, reshaped_array.shape[0], small_size[0]):
    #     for j in range(0, reshaped_array.shape[1], small_size[1]):
    #         small_matrix = reshaped_array[i:i+small_size[0], j:j+small_size[1]]
    #         small_matrices.append(small_matrix)

    # # Print the number of small matrices
    # print("Number of small matrices:", len(small_matrices))
    # print(small_matrices[0])

    # # Calculate the average value for each small matrix
    # averages = [np.mean(small_matrix) for small_matrix in small_matrices]

    # # Plot the average values
    # plt.plot(averages)

    # # # Add labels and title to the plot
    # # plt.xlabel('Small Matrix Index')
    # # plt.ylabel('Average Value')
    # # plt.title('Average Values of Small Matrices')

    # # # Display the plot
    # # plt.show()

    # # Calculate the standard deviation of the mean values
    # std_dev = np.std(averages)

    # # Plot the mean values and standard deviation
    # plt.errorbar(range(len(averages)), averages, yerr=std_dev, fmt='o')

    # # Add labels and title to the plot
    # plt.xlabel('Grid Index')
    # plt.ylabel('Mean Value (cm)')
    # plt.title('Mean Values with Standard Deviation')

    # # Display the plot
    # plt.show()


    # global minimum
    # if len_points < minimum:
    #     minimum = len_points
    #     print("New Minimum: ", minimum)

    # nonzero_index, zero_index = find_nonzero_zero_indices(points_list_x)
    
    # points_list = []


    # for index, data in enumerate(gen):
    #     if index == nonzero_index:  # Use the nonzero_index obtained from find_nonzero_zero_indices()
    #         points_list.append(data[1])
    
    # print(points_list)
    # print("Filtered: %d points", len(points_list))
    
       
    # first_row = reshaped_matrix[1, :]
    # # cropped_points_array = points_array[:159,1]
    # # # Generate x-axis values (indices of the tuple)
    # # x = range(len(cropped_points_array))

    # # # Plot the tuple values
    # plt.plot(first_row)

    # # # Add labels and title
    # plt.xlabel('Index')
    # plt.ylabel('X values')
    # plt.title('Variation of C')

    # # Display the plot
    # plt.show()


    # pcl_data = pcl.PointCloud_PointXYZRGBA()
    # pcl_data.from_list(points_list)
    # print(points_list)

    # points_array = np.array(points_list)

    # first_grid = points_list[:10]
    # print(first_grid)
    

    # nonzero_index, zero_index = find_nonzero_zero_indices(points_list)

    # print(f"First non-zero element found at index: {nonzero_index}")
    # print(f"Next zero element found at index: {zero_index}")


def listener():
    # Initialize the node
    rospy.init_node('pointcloud_subscriber', anonymous=True)

    # Subscribe to the 'pointcloud' topic
    rospy.Subscriber("/scan_3D", PointCloud2, pointcloud_callback)

    # Spin until Ctrl+C is pressed
    rospy.spin()

if __name__ == '__main__':
    listener()