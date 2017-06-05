#define GRY "\x1b[90m"
#define RED "\x1b[91m"
#define YEL "\x1b[93m"
#define BLU "\x1b[94m"
#define MAG "\x1b[95m"
#define CYN "\x1b[96m"
#define GRN "\x1b[92m"

#include <ros/ros.h>
#include <ros/timer.h>
#include <math.h>
#include <iostream>
#include <string>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <ros/package.h>

// Messages
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
std::string file_name;
void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg,cloud);
  if(file_name.empty())
  {
    ROS_ERROR("empty file name!!!,  saving failed");
    exit(0);
  }
  std::string path = ros::package::getPath("pc_maker") + "/pcd/" + file_name +".pcd";
  pcl::io::savePCDFileASCII (path, cloud);
  ROS_INFO_STREAM("file successfully saved to \n" << GRY << path);
  exit(0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_server");
  ros::NodeHandle n;
  switch (argc)
  {
  case 1:
    ROS_ERROR("Too few args, please enter either save our run + filename.pcd");
    break;
  case 3:
  {
    ROS_INFO_STREAM(argv[1]);
    std::string arg(argv[1]);
    std::string tmp(argv[2]);
    file_name = tmp;
    std::string cloud_topic("RL_cloud");
    if(arg.compare("save") == 0)
    {

      ROS_INFO_STREAM("Start subscribing to " << BLU << cloud_topic);
      ros::Subscriber cloud_sub = n.subscribe(cloud_topic,1,cloud_cb);
      ros::Rate r(10);
      while(ros::ok())
      {
        ros::spinOnce();
        r.sleep();
      }
    }
    else if(arg.compare("run") == 0)
    {
      if(file_name.empty())
      {
        ROS_ERROR("empty file name!!!,  saving failed");
        exit(0);
      }
      std::string path = ros::package::getPath("pc_maker") + "/pcd/" + file_name +".pcd";
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud_ptr) == -1) //* load the file
      {
        ROS_ERROR_STREAM("cloud not read the file \n" << MAG << path);
        return (-1);
      }
      ROS_INFO_STREAM("file opened successfully \n name:" << file_name << "\n path: " << MAG << path);
      ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2> (cloud_topic,1);
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(*cloud_ptr,cloud_msg);
      cloud_msg.header.frame_id = "laser";

      ros::Rate r(3);
      ROS_INFO_STREAM(GRN<<"Start publishing the cloud in topic " << MAG << cloud_topic);
      while(ros::ok())
      {
        cloud_msg.header.stamp = ros::Time::now();
        cloud_pub.publish(cloud_msg);
        r.sleep();
      }
    }
    else
      ROS_ERROR("unkown command!, use  save filename.pcd   or   run filename.pcd");
    break;
  }
  }
  return 0;
}
