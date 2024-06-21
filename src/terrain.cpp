#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/distances.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/bilateral.h>

#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/surface/mls.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h> // Use OpenMP version for faster computation
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <cmath>
#include <boost/make_shared.hpp> // For creating shared_ptr instances
#include <algorithm>
#include <limits>
#include <vector>
#include <sstream>

// ROS Publishers
ros::Publisher pub_after_passthrough_x;
ros::Publisher pub_after_passthrough_y;
ros::Publisher pub_after_passthrough_z;
ros::Publisher pub_after_downsampling;

ros::Publisher marker_pub;

std::vector<pcl::ModelCoefficients> plane_coefficients;









// ----------------------------------------------------------------------------------
// PREPROCESSING STEPS
// ----------------------------------------------------------------------------------

// Function to publish a point cloud
void publishProcessedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const ros::Publisher& publisher, const sensor_msgs::PointCloud2ConstPtr& original_msg)
{
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header = original_msg->header;
    publisher.publish(output_msg);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr passthroughFilterX(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.0, 1.5);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
  pass.filter(*cloud_filtered_x);

  return cloud_filtered_x;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr passthroughFilterY(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.3, 0.3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*cloud_filtered_y);

    return cloud_filtered_y;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr passthroughFilterZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.7, 0.7);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*cloud_filtered_z);

    return cloud_filtered_z;
}


// Voxel Grid Downsampling
pcl::PointCloud<pcl::PointXYZ>::Ptr voxelGridDownsampling(
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    float leaf_size_x,  // Leaf size for x dimension
                                    float leaf_size_y,   // Leaf size for y dimension
                                    float leaf_size_z)   
{
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid.filter(*cloud_downsampled);

  return cloud_downsampled;
}




// ----------------------------------------------------------------------------------
// PLANE SEGMENTATION
// ----------------------------------------------------------------------------------

// Helper function to determine if two planes are similar
bool is_similar_plane(const pcl::ModelCoefficients& plane1, const pcl::ModelCoefficients& plane2, double angle_threshold, double distance_threshold) {
    Eigen::Vector3f normal1(plane1.values[0], plane1.values[1], plane1.values[2]);
    Eigen::Vector3f normal2(plane2.values[0], plane2.values[1], plane2.values[2]);
    double angle = acos(normal1.dot(normal2) / (normal1.norm() * normal2.norm()));

    double distance1 = plane1.values[3] / normal1.norm();
    double distance2 = plane2.values[3] / normal2.norm();
    double distance_difference = fabs(distance1 - distance2);

    return (angle < angle_threshold && distance_difference < distance_threshold);
}

void extract_planes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float remaining_percentage, int max_planes, int max_iterations, double distance_threshold, double angle_threshold = 0.1) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_filtered);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations);
    seg.setDistanceThreshold(distance_threshold);

    while (cloud_filtered->points.size() > remaining_percentage * cloud->points.size() && plane_coefficients.size() < max_planes) {
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            break;
        }

        bool plane_updated = false;
        for (auto& existing_coeff : plane_coefficients) {
            if (is_similar_plane(existing_coeff, *coefficients, angle_threshold, (distance_threshold*5))) {
                existing_coeff = *coefficients;  // Update the existing plane coefficients
                plane_updated = true;
                break;
            }
        }

        if (!plane_updated) {
            plane_coefficients.push_back(*coefficients);
        }

        // Extract inliers (points belonging to the current plane) and update the cloud for the next iteration
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);  // Extract the points that are not inliers
        extract.filter(*cloud_filtered);
    }

    std::sort(plane_coefficients.begin(), plane_coefficients.end(), [](const pcl::ModelCoefficients& a, const pcl::ModelCoefficients& b) {
        return a.values[3] < b.values[3];
    });

    // Print the number of planes and their equations
    std::cout << "Number of planes found: " << plane_coefficients.size() << std::endl;
    // for (size_t i = 0; i < plane_coefficients.size(); ++i) {
    //     std::cout << "Plane " << i + 1 << ": " 
    //               << plane_coefficients[i].values[0] << "x + "
    //               << plane_coefficients[i].values[1] << "y + "
    //               << plane_coefficients[i].values[2] << "z + "
    //               << plane_coefficients[i].values[3] << " = 0" << std::endl;
    // }
}








// ----------------------------------------------------------------------------------
// NORMAL EXTRACTION
// ----------------------------------------------------------------------------------

pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    int k_numbers)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(k_numbers);  // Adjust the value based on your data
    ne.compute(*normals);

    return normals;
}

void visualizeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& normals) {
    pcl::visualization::PCLVisualizer viewer("Normals Visualization");
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Dark background for better visibility
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");

    // Add normals to the viewer with a specific scale factor for better visibility
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 0.05, "normals");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}




// ----------------------------------------------------------------------------------
// NORMAL ANALYSIS
// ----------------------------------------------------------------------------------


// Perform PCA on Normals and visualize the results
void performPCAAndVisualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& normals) {
    Eigen::MatrixXf data(normals->points.size(), 3);

    for (size_t i = 0; i < normals->points.size(); ++i) {
        data(i, 0) = normals->points[i].normal_x;
        data(i, 1) = normals->points[i].normal_y;
        data(i, 2) = normals->points[i].normal_z;
    }

    Eigen::MatrixXf centered = data.rowwise() - data.colwise().mean();
    Eigen::MatrixXf cov = centered.adjoint() * centered;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);
    Eigen::Vector3f eigenvalues = eig.eigenvalues();
    Eigen::Matrix3f eigenvectors = eig.eigenvectors();

    std::cout << "Eigenvalues: \n" << eigenvalues << std::endl;
    std::cout << "Eigenvectors: \n" << eigenvectors << std::endl;

    // Visualize the principal components
    pcl::visualization::PCLVisualizer viewer("PCA Visualization");
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Dark background for better visibility
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");

    // Add the principal components as lines
    pcl::PointXYZ center(0.0, 0.0, 0.0);
    viewer.addLine<pcl::PointXYZ>(center, pcl::PointXYZ(eigenvectors(0, 0), eigenvectors(1, 0), eigenvectors(2, 0)), "principal_component_1");
    viewer.addLine<pcl::PointXYZ>(center, pcl::PointXYZ(eigenvectors(0, 1), eigenvectors(1, 1), eigenvectors(2, 1)), "principal_component_2");
    viewer.addLine<pcl::PointXYZ>(center, pcl::PointXYZ(eigenvectors(0, 2), eigenvectors(1, 2), eigenvectors(2, 2)), "principal_component_3");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}



// ----------------------------------------------------------------------------------
// POINTCLOUD CALLBACK
// ----------------------------------------------------------------------------------


// Main callback function for processing PointCloud2 messages
void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input_msg, ros::NodeHandle& nh)
{
    // Convert ROS PointCloud2 message to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_msg, *cloud);
    ROS_INFO("Raw PointCloud: %ld points", cloud->points.size());

    // Passthrough Filtering with Z-Axis : Vertical axis
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_passthrough_z = passthroughFilterZ(cloud);
    publishProcessedCloud(cloud_after_passthrough_z, pub_after_passthrough_z, input_msg);
    ROS_INFO("After Passthough filter Z: %ld points", cloud_after_passthrough_z->points.size());

    // Passthrough Filtering with X-Axis : Depth axis
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_passthrough_x = passthroughFilterX(cloud_after_passthrough_z);
    publishProcessedCloud(cloud_after_passthrough_x, pub_after_passthrough_x, input_msg);
    ROS_INFO("After Passthough filter X: %ld points", cloud_after_passthrough_x->points.size());

    // Passthrough Filtering with Y-Axis : Left-Right axis
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_passthrough_y = passthroughFilterY(cloud_after_passthrough_x);
    // publishProcessedCloud(cloud_after_passthrough_y, pub_after_passthrough_y, input_msg);
    // ROS_INFO("After Passthough filter Y: %ld points", cloud_after_passthrough_y->points.size());

    // Downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_downsampling = voxelGridDownsampling(
        cloud_after_passthrough_x, 0.10f, 0.10f, 0.01f);
    publishProcessedCloud(cloud_after_downsampling, pub_after_downsampling, input_msg);

    // // Log the number of points in the downsampled cloud directly
    ROS_INFO("After Downsampling: %ld points", cloud_after_downsampling->points.size());

    // ROS_INFO(" "); // Creating a line gap for better readability

    // ------------------------------------------------------   
    
    // Normal Estimation
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(cloud_after_downsampling, 50);
    // visualizeNormals(cloud_after_downsampling, cloud_normals);

    // PCA and Visualization
    performPCAAndVisualize(cloud_after_downsampling, cloud_normals);

    // Introducing a delay for analyzing results
    ROS_INFO("----------------------------------------------------------------");
    ros::Duration(2.0).sleep();
}





// ----------------------------------------------------------------------------------
// ROS MAIN FUNCTION
// ----------------------------------------------------------------------------------


// ROS main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_node");
    ros::NodeHandle nh;

    // Publishers
    pub_after_passthrough_x = nh.advertise<sensor_msgs::PointCloud2>("/passthrough_cloud_x", 1);
    // pub_after_passthrough_y = nh.advertise<sensor_msgs::PointCloud2>("/passthrough_cloud_y", 1);
    pub_after_passthrough_z = nh.advertise<sensor_msgs::PointCloud2>("/passthrough_cloud_z", 1);
    
    pub_after_downsampling = nh.advertise<sensor_msgs::PointCloud2>("/downsampled_cloud", 1);
    // pub_after_sor = nh.advertise<sensor_msgs::PointCloud2>("/sor_filtered_cloud", 1);
    
    // Subscribing to Lidar Sensor topic
    // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/scan_3D", 1, boost::bind(pointcloud_callback, _1, boost::ref(nh)));
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 1, boost::bind(pointcloud_callback, _1, boost::ref(nh)));
    
    
    ros::spin();

    return 0;
}