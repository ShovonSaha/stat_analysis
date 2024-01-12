#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import pcl
import time

def pointcloud_callback(data):
    # Create a generator to iterate over the points in the point cloud
    gen = pc2.read_points(data, skip_nans=True)
    
    points_list = []

    for data in gen:
        points_list.append([data[0], data[1], data[2], data[3]])
    
    pcl_data = pcl.PointCloud_PointXYZRGBA()
    pcl_data.from_list(points_list)

    # Getting the start time
    # start_time = time.time()

    # PCL Plane Segmentation Algorithm
    seg = pcl_data.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.2)
    seg.set_normal_distance_weight(0.01)
    seg.set_max_iterations(50)
    indices, coefficients = seg.segment()
    min_inliers = 500  # Set a threshold for the minimum number of inliers

    if len(indices) == 0:
        print('Could not estimate a planar model for the given dataset.')
        return
    
    if len(indices) < min_inliers:
        rospy.loginfo("The segmented plane has less than the minimum required inliers. Skipping.")
        return
    
    # Getting the end time
    # end_time = time.time()

    # Calculating the time taken for the custom plane segmentation algorithm
    # time_taken = end_time - start_time
    # time_array = np.append(time_array, time_taken)

    # print("Time Taken: ", time_taken)
    
    # print(indices)
    # print("Length = ",len(indices))

    # print('Model coefficients: ' + str(coefficients[0]) + ' ' + str(
    #     coefficients[1]) + ' ' + str(coefficients[2]) + ' ' + str(coefficients[3]))
    # print('Model inliers: ' + str(len(indices)))

    # Add the plane points to the new message
    plane_points = []
    
    # iterate over the indices found by the RANSAC segmentation and append the corresponding points to the plane_points list:
    for i in indices:
        point = pcl_data[i]
        plane_points.append([point[0], point[1], point[2]])

    # Create a new point cloud message for the filtered point cloud
    filtered_msg = PointCloud2()
    filtered_msg.header = std_msgs.msg.Header()
    filtered_msg.header.stamp = rospy.Time.now()
    filtered_msg.header.frame_id = 'laser_link'
    filtered_msg.height = 1
    filtered_msg.width = len(indices)

    # Only considering the X, Y and Z fields
    filtered_msg.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                       PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                       PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]
    filtered_msg.point_step = 12  # Change from 32 to 12, as we only have x, y, z fields now

    filtered_msg.is_bigendian = False
    # filtered_msg.point_step = 32
    filtered_msg.row_step = filtered_msg.point_step * filtered_msg.width
    filtered_msg.is_dense = True

    # Add the filtered points to the message
    filtered_msg.data = np.asarray(plane_points, dtype=np.float32).tostring()

    # Create a publisher for the '/filtered_point_cloud' topic
    filtered_pub = rospy.Publisher('/pcl_plane_seg_cloud', PointCloud2, queue_size=10)

    # Publish the filtered point cloud message to the '/filtered_point_cloud' topic
    filtered_pub.publish(filtered_msg)
    rospy.loginfo('Published filtered point cloud with %d points to topic /pcl_plane_seg_cloud', len(indices))



def listener():
    # Initialize the node
    rospy.init_node('pointcloud_subscriber', anonymous=True)
    
    # Subscribe to the 'pointcloud' topic
    rospy.Subscriber("/scan_3D", PointCloud2, pointcloud_callback)

    # Spin until Ctrl+C is pressed
    rospy.spin()

if __name__ == '__main__':
    listener()
