#!/usr/bin/env python3

import numpy as np
import struct
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import pcl

def pointcloud_callback(data):
    #print(data.height)
    #return
    # Create a generator to iterate over the points in the point cloud
    gen = pc2.read_points(data, skip_nans=True)
    #cloud = pcl.PointCloud_PointXYZRGBA.from_array(gen)
    
    # print(gen.header)

    points_list = []

    for data in gen:
        points_list.append([data[0], data[1], data[2], data[3]])
        
    pcl_data = pcl.PointCloud_PointXYZRGBA()
    pcl_data.from_list(points_list)
    print(pcl_data)
    seg = pcl_data.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.01)
    seg.set_normal_distance_weight(0.01)
    seg.set_max_iterations(100)
    indices, coefficients = seg.segment()
    if len(indices) == 0:
        print('Could not estimate a planar model for the given dataset.')
        return
    print('Model coefficients: ' + str(coefficients[0]) + ' ' + str(
        coefficients[1]) + ' ' + str(coefficients[2]) + ' ' + str(coefficients[3]))
    print('Model inliers: ' + str(len(indices)))

    # For printing the points in the indices where a plane was found

    # for i in range(0, len(indices)):
    #     print(str(indices[i]) + ', x: ' + str(pcl_data[indices[i]][0]) + ', y : ' +
    #           str(pcl_data[indices[i]][1]) + ', z : ' + str(pcl_data[indices[i]][2]))
    # return

    # Create a new point cloud message for the plane
    plane_msg = PointCloud2()
    plane_msg.header = std_msgs.msg.Header
    plane_msg.header.stamp = rospy.Time.now()
    plane_msg.header.frame_id = 'laser_link'
    plane_msg.height = 1
    plane_msg.width = len(indices)
    plane_msg.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1)]
    plane_msg.is_bigendian = False
    plane_msg.point_step = 32
    plane_msg.row_step = plane_msg.point_step * plane_msg.width
    plane_msg.is_dense = True
    
    # Add the plane points to the message
    plane_points = []
    for i in indices:
        point = pcl_data[i]
        # rgb = struct.unpack('I', struct.pack('BBBB', point[3], point[2], point[1], point[0]))[0]
        # rgb = struct.unpack('I', struct.pack('BBBB', int(point[3]), int(point[2]), int(point[1]), int(point[0])))[0]
        rgb = struct.unpack('I', struct.pack('BBBB',
                                     int(max(0, min(255, point[3]))),
                                     int(max(0, min(255, point[2]))),
                                     int(max(0, min(255, point[1]))),
                                     int(max(0, min(255, point[0])))))[0]

        plane_points.append([point[0], point[1], point[2], rgb])

    plane_msg.data = np.asarray(plane_points, dtype=np.float32).tostring()
    
    # Create a publisher for the 'plane_1' topic
    plane_pub = rospy.Publisher('/plane_1', PointCloud2, queue_size=10)
        
    # Publish the plane point cloud message to the 'plane_1' topic
    plane_pub.publish(plane_msg)
    rospy.loginfo('Published plane point cloud with %d points to topic /plane_1', len(indices))



def listener():
    # Initialize the node
    rospy.init_node('pointcloud_subscriber', anonymous=True)
    
    # Subscribe to the 'pointcloud' topic
    rospy.Subscriber("/scan_3D", PointCloud2, pointcloud_callback)

    # Spin until Ctrl+C is pressed
    rospy.spin()

if __name__ == '__main__':
    listener()
