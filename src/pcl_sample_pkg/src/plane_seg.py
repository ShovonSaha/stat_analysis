#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import pcl
import time

time_array = []

# Custom RANSAC plane segmentation functions
def fit_plane(points):
    centroid = np.mean(points, axis=0)
    _, _, V = np.linalg.svd(points - centroid)
    normal = V[-1]
    return centroid, normal

def distance_to_plane(point, centroid, normal):
    return np.abs(np.dot(normal, point - centroid))

def ransac_plane_segmentation(points, iterations=50, threshold=0.2, min_inliers=500):
    best_inliers = []
    best_centroid = None
    best_normal = None

    for _ in range(iterations):
        sample_points = points[np.random.choice(points.shape[0], 3, replace=False)]
        centroid, normal = fit_plane(sample_points)
        distances = np.array([distance_to_plane(point, centroid, normal) for point in points])

        inliers = points[distances < threshold]

        if len(inliers) > len(best_inliers) and len(inliers) > min_inliers:
            best_inliers = inliers
            best_centroid = centroid
            best_normal = normal

    return best_inliers, best_centroid, best_normal

# Main callback function
def pointcloud_callback(data):
    gen = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z", "rgba"))

    points_list = []

    for data in gen:
        points_list.append([data[0], data[1], data[2], data[3]])
    
    pcl_data = pcl.PointCloud_PointXYZRGBA()
    pcl_data.from_list(points_list)

    # Getting the start time
    start_time = time.time()

    # Calling Custom Plane Segmentation 
    points_array = np.array(points_list)
    inliers, centroid, normal = ransac_plane_segmentation(points_array[:, :3])

    # Getting the end time
    end_time = time.time()

    # Calculating the time taken for the custom plane segmentation algorithm
    time_taken = end_time - start_time
    # time_array = np.append(time_array, time_taken)

    print("Time Taken: ", time_taken)

    # print("Inliers:", inliers)
    # print("Centroid:", centroid)
    # print("Normal:", normal)

    # Create a new point cloud message for the 
    # Custom Plane Segmentation Algorithm point cloud
    filtered_msg = PointCloud2()
    filtered_msg.header = std_msgs.msg.Header()
    filtered_msg.header.stamp = rospy.Time.now()
    filtered_msg.header.frame_id = 'laser_link'
    filtered_msg.height = 1
    filtered_msg.width = len(inliers)

    filtered_msg.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                       PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                       PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]
    filtered_msg.point_step = 12
    filtered_msg.is_bigendian = False
    filtered_msg.row_step = filtered_msg.point_step * filtered_msg.width
    filtered_msg.is_dense = True

    filtered_msg.data = np.asarray(inliers, dtype=np.float32).tostring()

    filtered_pub = rospy.Publisher('/custom_plane_seg_cloud', PointCloud2, queue_size=10)
    filtered_pub.publish(filtered_msg)
    rospy.loginfo('Published Custom Plane point cloud with %d points to topic /custom_plane_seg', len(inliers))
    

def listener():
    rospy.init_node('pointcloud_subscriber', anonymous=True)
    rospy.Subscriber("/scan_3D", PointCloud2, pointcloud_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()