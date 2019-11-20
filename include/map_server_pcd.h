#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "cloud_msgs/cloud_info.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>     //PCL的PCD格式文件的输入输出头文件

#include "nav_msgs/GetMap.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"


using PointT = pcl::PointXYZI;
ros::Subscriber globalmap_sub_;
ros::Subscriber map_sub_;
bool saved_3dmap_;
bool saved_2dmap_;
std::string map_name_;                     //map name

void globalmap_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg);
void mapCallback(const nav_msgs::OccupancyGridConstPtr& map);