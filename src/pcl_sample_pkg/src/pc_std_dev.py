import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import time

def pointcloud_callback(data):
    

    # Create a generator to iterate over the points in the point cloud
    gen = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z", "rgba"))

    points_list = []

    for data in gen:
        points_list.append([data[0], data[1], data[2], data[3]])
    
    # Assuming points_list is a list of points in the format [x, y, z, rgba]
    total_points = len(points_list)
    mean_positions = []  # Initialize an empty list to store mean positions
    std_deviations = []  # Initialize an empty list to store standard deviations

    # Calculate the sum of X, Y, and Z coordinates
    for point in points_list:
        mean_position = [0.0, 0.0, 0.0]  # Initialize mean position for the current point

        # Calculate the sum of X, Y, and Z coordinates for the current point
        for i in range(3):
            mean_position[i] += point[i]

        # Divide the sum by the total number of points to get the mean position
        for i in range(3):
            mean_position[i] /= total_points

        mean_positions.append(mean_position)  # Store the mean position in the array

        # Calculate the standard deviation for the current point
        std_deviation = [0.0, 0.0, 0.0]  # Initialize standard deviation for the current point

        # Calculate the sum of squared differences for X, Y, and Z coordinates
        for i in range(3):
            squared_diff_sum = 0.0

            for point in points_list:
                squared_diff = (point[i] - mean_position[i]) ** 2
                squared_diff_sum += squared_diff

            std_deviation[i] = np.sqrt(squared_diff_sum / total_points)

        std_deviations.append(std_deviation)  # Store the standard deviation in the array

    
    # Extract the X, Y, and Z coordinates from the mean_positions list
    x = [mean_position[0] for mean_position in mean_positions]
    y = [mean_position[1] for mean_position in mean_positions]
    z = [mean_position[2] for mean_position in mean_positions]

    # Create a 3D scatter plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, c='b', marker='o')

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Mean Positions of Points')

    # Show the plot
    plt.show()

def listener():
    # Initialize the node
    rospy.init_node('pointcloud_subscriber', anonymous=True)
    
    # Subscribe to the 'pointcloud' topic
    rospy.Subscriber("/scan_3D", PointCloud2, pointcloud_callback)

    # Spin until Ctrl+C is pressed
    rospy.spin()

if __name__ == '__main__':
    listener()
