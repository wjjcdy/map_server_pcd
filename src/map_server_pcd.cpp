#include "../include/map_server_pcd.h"
#include <cv.h>
#include <highgui.h>
using namespace cv; 
int main(int argc, char** argv) {
  ros::init(argc, argv, "map_server_pcd");

  ros::NodeHandle nh("~");;

  std::string map_3d_topic;
  std::string map_2d_topic;
  nh.param<std::string>("file_name", map_name_, "test_pcd");
  nh.param<std::string>("map_3d_topic", map_3d_topic, "global_map");
  nh.param<std::string>("map_2d_topic", map_2d_topic, "map");
  std::cout<<"waiting receive map"<<std::endl;

  globalmap_sub_ = nh.subscribe(map_3d_topic, 1, &globalmap_callback);
  map_sub_ = nh.subscribe(map_2d_topic, 1, &mapCallback);
  saved_3dmap_ = false;
  saved_2dmap_ = false;
  while(!(saved_3dmap_ && saved_2dmap_)  && ros::ok())
    ros::spinOnce();
  return 0;
}

void globalmap_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
  if (saved_3dmap_) {
    return;
    }
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*points_msg, *cloud);
  pcl::io::savePCDFile (map_name_+".pcd", *cloud);
  saved_3dmap_ = true;
  ROS_WARN("3d map SAVED");
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
{
  if (saved_2dmap_) {
    return;
  }
  ROS_INFO("Received a %d X %d map @ %.3f m/pix",
            map->info.width,
            map->info.height,
            map->info.resolution);

  // std::string mapdatafile = map_name_ + ".pgm";
  // ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
  // FILE* out = fopen(mapdatafile.c_str(), "w");
  // if (!out)
  // {
  //   ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
  //   return;
  // }

  // fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
  //         map->info.resolution, map->info.width, map->info.height);

  Mat map_new( map->info.height, map->info.width,CV_8UC1);

  for(unsigned int y = 0; y < map->info.height; y++) 
  {
    for(unsigned int x = 0; x < map->info.width; x++)
    {
      unsigned int i = x + (map->info.height - y - 1) * map->info.width;
      if (map->data[i] == 0) 
      { 
        //occ [0,0.1)
        // fputc(254, out);
        map_new.at<char>(y,x) = 254;
      } 
      else if (map->data[i] >= +100) 
      { 
        //occ (0.65,1]
        // fputc(000, out);
        map_new.at<char>(y,x)  = 0;
      } 
      else if((map->data[i] >0 && map->data[i] <100))
      {
        // fputc(254- map->data[i]*254/100, out);
        //fputc(205,out);
        map_new.at<char>(y,x)  = 254- map->data[i]*254/100;
      }
      else 
      { 
        //occ [0.1,0.65]
        // fputc(128, out);
        map_new.at<char>(y,x)  = 128;
      }
    }
  }

  // fclose(out);
  IplImage img;
  std::string png_map_name = map_name_ + ".png";
  img = IplImage(map_new);
  cvSaveImage(png_map_name.c_str(),&img);

  std::string mapmetadatafile = map_name_ + ".yaml";
  ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
  FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

  /*
    resolution: 0.100000
    origin: [0.000000, 0.000000, 0.000000]
    #
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196

    */

  geometry_msgs::Quaternion orientation = map->info.origin.orientation;
  tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);

  fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
          png_map_name.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

  fclose(yaml);

  ROS_INFO("Done\n");
  saved_2dmap_ = true;
}


