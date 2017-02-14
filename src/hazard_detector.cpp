#include <ros/ros.h>
#include <ros/timer.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
//Sensor_msgs
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
//Standards
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <string>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
// Laser geometry

// Messages
#include "std_msgs/String.h"
#include "donkey_rover/Rover_Scanner.h"

// Service
class hazard_detector
{
  public:
  hazard_detector(ros::NodeHandle& node)
  {
     node = n_;
     sub_from_scanner = n_.subscribe("/RoverScannerInfo", 1, &hazard_detector::scanner_msg_cb,this);
     sub_from_laser  	= n_.subscribe("scan", 10, &hazard_detector::laser_cb,this);
     sub_from_obs_pc	= n_.subscribe("obstacle_cloud", 10, &hazard_detector::obs_pc_cb,this);


     hazard_pub = n_.advertise<std_msgs::String>("Hazard",1);

     //Z boundaries
     Z_ub =  0.90;
     Z_lb = -0.20;
     //Z_Fall = -0.50; falling of a cliff should be also avoidad
     Hazard_timestamp = ros::Time::now();
  }

  void handle()
  {
     ros::NodeHandle n_ptr("~");
     if(!n_ptr.getParam("circle", circle_))
     {
       circle_ = 2.00;
       ROS_WARN("No value received for the hazard circle it is set to default value of %f",circle_);
     }
     ros::Rate r(10);
     ros::Time current_time;
     ros::Duration Since_last_hazard;
     std_msgs::String Hazard_msg;
     double circle_new;
     while(n_.ok())
     {
       if(n_ptr.getParam("circle", circle_new))
       {
         if(abs(circle_new - circle_)>0.001)
         {
            circle_ = circle_new;
            ROS_WARN("Hazard_Detector: new value for circle received: %f ",circle_);
         }
       }
       current_time = ros::Time::now();
       Since_last_hazard = current_time - Hazard_timestamp;

       if(Since_last_hazard.toSec() > 3.00) //Reset the Hazard status if no new hazard is observed since 3 secs
         Hazard = "Green";
       Hazard_msg.data = Hazard;
       hazard_pub.publish(Hazard_msg);
       ros::spinOnce();
       r.sleep();
     }

  }

  protected:
  ros::NodeHandle n_;

  ros::Subscriber sub_from_laser;
  ros::Subscriber sub_from_scanner;
  ros::Subscriber sub_from_obs_pc;

  ros::Publisher hazard_pub;

  std::string Hazard; // 'R' Red, 'Y' Yellow, "G" Green
  char scanner_state; // 'U' unknown, 'R' rolling, "I" idle
  double circle_;
  float Z_ub,Z_lb;
  ros::Time Hazard_timestamp;

  private:
  void scanner_msg_cb(const donkey_rover::Rover_Scanner::ConstPtr& msg)
  {
      std::string state = msg->Scanner_State;
      scanner_state = 'U';
      if (state.compare("Idle") == 0 )  scanner_state = 'I';
      if (state.compare("Rolling") == 0 )  scanner_state = 'R';
  }

  void laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
      if(scanner_state != 'I') return;
      std::vector<float> range_vector = msg->ranges;
      std::vector<size_t> red_index;
      std::vector<size_t> yellow_index;
      for(size_t i=0;i<range_vector.size();i = i+2)
      {
        if(range_vector[i] < circle_)
        {
          if(range_vector[i] < circle_/3)
            red_index.push_back(i);
          else
            yellow_index.push_back(i);
        }
      }
      if(yellow_index.size() > 3)
      {
        Hazard = "Yellow";
        Hazard_timestamp = ros::Time::now();
      }
      if(red_index.size() > 3)
      {
        Hazard = "Red";
        Hazard_timestamp = ros::Time::now();
      }
  }

  void obs_pc_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
      if(scanner_state != 'R') return;
      int Red_count = 0;
      int Yellow_count = 0;
      pcl::PointCloud<pcl::PointXYZ> obs_pc;
      pcl::fromROSMsg(*msg,obs_pc);
      for(size_t i=0;i<obs_pc.size();i++)
      {
        if(obs_pc.points[i].z > Z_ub || obs_pc.points[i].z < Z_lb)
          continue;
        if(Point_rad_dist(obs_pc.points[i]) < circle_)
        {
          Hazard = "Yellow";
          Hazard_timestamp = ros::Time::now();
          if(Point_rad_dist(obs_pc.points[i]) < circle_/3.00) Hazard = "Red";
        }
      }
  }
  float Point_rad_dist(pcl::PointXYZ p)
  {
    float out;
    out = sqrt(pow(p.x,2)+pow(p.y,2));
    return out;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hazard_detector");
  ros::NodeHandle node;

  hazard_detector hz(node);

  hz.handle();

  return 0;
}
