#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/correspondence.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_datatypes.h>
#include <flann/flann.h>
#include <iostream>
#include <unordered_map>
#include <string>
#include <ctime>

#define VISUALIZE
#define INTERNAL_SYNC