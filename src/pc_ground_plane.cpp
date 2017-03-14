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
#include <laser_geometry/laser_geometry.h>
// Messages
#include "std_msgs/Float32.h"
#include "donkey_rover/Rover_Scanner.h"
#include "pc_maker/CloudMetaData.h"
#include <lms1xx/TiltScan.h>
// Service
#include "laser_assembler/AssembleScans.h"

#include <Eigen/Dense> 


union U_ANGLE
{
  float f;
  uint8_t b[4];
};

enum scannerSTATE {mid1,side1,mid2,side2};

struct LaserBim
{
  float angle;
  sensor_msgs::LaserScan bim;
};

class LaserToPcClass
{
	public:
		
	LaserToPcClass(ros::NodeHandle& node)
	{
			// Node handle
			n_=node;

			//subscribers
			SubFromScannerInfo_		= n_.subscribe("/RoverScannerInfo", 1, &LaserToPcClass::scanner_msg_callback,this);
      SubFromScannerangle_	= n_.subscribe("/Scanner_angle_sync", 1, &LaserToPcClass::scanner_angle_callback,this);
      SubFromLaser_			    = n_.subscribe("scan", 10, &LaserToPcClass::laser_call_back,this);
      SubFromScanTilt_       = n_.subscribe("Tilt_scan", 1, &LaserToPcClass::Tiltscan_cb,this);
			
			// publishers
			Right_cloud_pub_ 		 = n_.advertise<sensor_msgs::PointCloud2> ("Right_cloud", 1);
			Left_cloud_pub_ 		 = n_.advertise<sensor_msgs::PointCloud2> ("Left_cloud", 1);
			RL_cloud_pub_ 		 	 = n_.advertise<sensor_msgs::PointCloud2> ("RL_cloud", 1);
			RL_cloud_Metadata_pub_		 = n_.advertise<pc_maker::CloudMetaData> ("RL_cloud_MetaData", 1);


    			// Initializers
			right_published = false;
			left_published  = false;

      scanAngleEnc_last = 18383; // init val
      ScannerState = mid1;
	}
  void Tiltscan_cb(const lms1xx::TiltScan::ConstPtr &msg)
  {
    // init
    #define deltaScan msg->ang-scanAngleEnc_last
    if(scanAngleEnc_last == 18383)
    {
      scanEncMax = 0;
      scanAngleEnc_last = msg->ang;
      ROS_INFO("Scanner Encoder initialized!");
    }
    // angle handle
    if (deltaScan > 10000) {
      scanAngleEnc += deltaScan - 16383;

      }
    else if(deltaScan < -10000){
      scanAngleEnc += deltaScan + 16383;
      }
    else
      scanAngleEnc += deltaScan;
    scanEncMax = std::max(abs(scanAngleEnc),scanEncMax);
    scanAngleEnc_last = msg->ang;
    //stacking
    LaserBim temp;
    temp.angle = scanAngleEnc/244.6*M_PI/180;
//    ROS_INFO("Encoder: %d Radian: %5f",scanAngleEnc,scanAngleEnc/244.6);
    temp.bim =  msg->bim;
    scanstack.push_back(temp);
    WatchStack();

  }

  void WatchStack()
  {
     double roll_angle,Period;
     if(!ros::param::get("/rover_state/scanner_config/Roll_angle",roll_angle))
     {
       ROS_FATAL("Param /rover_state/scanner_config/Roll_angle is missing, we can't work like this");
       return;
     }
     ros::param::get("/rover_state/scanner_config/Period",Period);
     roll_angle = fabs((float) roll_angle);
     switch (ScannerState)
     {
     case mid1:
       ROS_INFO("Midd 1");
       if(fabs((float) scanAngleEnc/244.6*M_PI/180) > roll_angle*0.7) ScannerState = side1;     
       break;
     case side1:
       ROS_WARN("Side 1");
       if(fabs((float) scanAngleEnc/244.6*M_PI/180) < roll_angle*0.3) ScannerState = mid2;
       break;
     case mid2:
       ROS_INFO("Mid 2");
        if(fabs((float) scanAngleEnc/244.6*M_PI/180) > roll_angle*0.7) ScannerState = side2;

        break;
     case side2:
       ROS_WARN("Side 2");
       if(fabs((float) scanAngleEnc) > 0.95*scanEncMax)
       {
         scanEncMax = 0;
         scanAngleEnc =  0;
         scanAngleEnc_last = 0;
         ros::param::set("/rover_state/scanner_command","Home");
         ROS_INFO("Done, stack size: %d",(int)scanstack.size());

         // calculate the cloud
         scanstack.empty();
         ScannerState = mid1;
         std::string scnstate;
         do {
             ros::param::get("/rover_state/scanner_state",scnstate);
             ROS_WARN("Waiting for the LIDAR TO GO BACK TO HOME");
             ros::Duration(0.5).sleep();
         }while(ros::ok() && !scnstate.compare("horizontal") == 0);
       }
       break;
     }
  }


	float point_distance(pcl::PointXYZ point_a, pcl::PointXYZ point_b)
	{
		float dist;
		dist = sqrtf( powf((point_a.x - point_b.x),2) + powf((point_a.y - point_b.y),2) + powf((point_a.z - point_b.z),2));
		return dist;
	}
	
	void partial_pc(const sensor_msgs::LaserScan::ConstPtr& scan_in, float theta_in,float theta_out,pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud, int id)
	{
		clock_t start = clock();
		float theta = theta_in;
		int start_index = (int) floor( (scan_in->angle_max - M_PI/2) /scan_in->angle_increment);
		start_index += (int) floor(theta_in/scan_in->angle_increment);
		int end_index = (int) floor( (scan_in->angle_max - M_PI/2) /scan_in->angle_increment);
		end_index += (int) floor(theta_out/scan_in->angle_increment) + 1;
		pcl::PointXYZ point_last;
		point_last.x = 0.0;point_last.y = 0.0; point_last.z = 0.0;
		for (int i = start_index; i< end_index ;i++)
		{
			ROS_INFO_ONCE("Start index: %d, End index: %d",start_index ,end_index );
			pcl::PointXYZ point;
			Eigen::Matrix4f M;
			M = transform.matrix();
			Eigen::Vector4f temp;
			temp(0) = scan_in->ranges[i] * sin(theta);    //2
			temp(1) = 1*scan_in->ranges[i] * cos(theta); //0 
			temp(2) = 0.0;				      //1
			temp(3) = 1.0;
			temp = M * temp;
			
			point.x = temp(0);
			point.y = temp(1);
			point.z = temp(2);
			
			float distance = point_distance(point,point_last);
			theta += scan_in->angle_increment;
			temp_cloud->points.push_back(point);
			if (point.z > Z_d_limit && point.z < Z_u_limit && point.x < X_limit && point.x > 0.00 && distance > voxel_filter)
			{
				if(point.y > 0 && point.y < Y_limit) inc_cloud_3d_left.points.push_back(point);
				if(point.y < 0 && point.y > -1.0*Y_limit)  inc_cloud_3d_right.points.push_back(point);
				point_last = point; //differential
			}
			//point_last = point;
			
			
		}
		clock_t end = clock();
		ROS_WARN_ONCE("loop time %f",(float) (end - start)/CLOCKS_PER_SEC);
		cloud_outlier_removal(temp_cloud,temp_cloud);

	
	}
	
	void laser_call_back(const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		partial_pc(scan_in, 0.0,     M_PI,   temp_cloud   ,1);
		inc_cloud_2d = *temp_cloud;
	}
	
	void scanner_msg_callback(const donkey_rover::Rover_Scanner::ConstPtr& msg)
	{
		roll_anlge = fabs(msg->Scanner_roll_angle);
	}
	
	void scanner_angle_callback(const std_msgs::Float32::ConstPtr& msg)
	{
		Scanner_angle_curr = msg->data;		
		sensor_msgs::PointCloud2 cloud_rl;
		
		if (Scanner_angle_curr < 0.0f) //<
		{
			//inc_cloud_3d_right += inc_cloud_2d;
			if (fabs(Scanner_angle_curr+roll_anlge) < 0.002 && !right_published)
			{
				sensor_msgs::PointCloud2 cloud_r;
  				pcl::toROSMsg(inc_cloud_3d_right,cloud_r);
  				cloud_r.header.stamp = ros::Time::now();
  				cloud_r.header.frame_id = "/laser";
  				
  				Right_cloud_pub_.publish(cloud_r);
  				//ROS_INFO("Right cloud");
  				
  				inc_cloud_3d = inc_cloud_3d_left + inc_cloud_3d_right;
  				
  				pcl::toROSMsg(inc_cloud_3d,cloud_rl);
  				cloud_rl.header.stamp = ros::Time::now();
  				cloud_rl.header.frame_id = "/laser";
  				
  				RL_cloud_pub_.publish(cloud_rl);
				publish_MetaData_msg();
  				inc_cloud_3d_right.clear();
  				inc_cloud_3d_left.clear();
  				
  				right_published = true;
  				left_published = false;
			}	
		
		}
		
		if (Scanner_angle_curr > 0.0f) //>
		{
			//inc_cloud_3d_left += inc_cloud_2d;
			if (fabs(Scanner_angle_curr-roll_anlge) < 0.002 && !left_published)
			{
          sensor_msgs::PointCloud2 cloud_l;
  				pcl::toROSMsg(inc_cloud_3d_left,cloud_l);
  				cloud_l.header.stamp = ros::Time::now();
  				cloud_l.header.frame_id = "/laser";
  				
  				Left_cloud_pub_.publish(cloud_l);
  				
  				inc_cloud_3d = inc_cloud_3d_left + inc_cloud_3d_right;
  				
  				pcl::toROSMsg(inc_cloud_3d,cloud_rl);
  				cloud_rl.header.stamp = ros::Time::now();
  				cloud_rl.header.frame_id = "/laser";
  				
  				RL_cloud_pub_.publish(cloud_rl);
          publish_MetaData_msg();
  				
  				//ROS_INFO("Left cloud");
  				inc_cloud_3d_right.clear();
  				inc_cloud_3d_left.clear();
  				right_published = false;
  				left_published = true;
			}
				
		
		}		
		
		transform = Eigen::Affine3f::Identity();
		transform.rotate (Eigen::AngleAxisf (Scanner_angle_curr, Eigen::Vector3f::UnitX()));
		
		
		
		Scanner_angle_last = Scanner_angle_curr;
		
    	}
		
	void cloud_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
	{
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  		sor.setInputCloud (cloud_in);
  		sor.setMeanK (50);
  		sor.setStddevMulThresh (1.0);
  		sor.filter (*cloud_filtered);
	}

	void publish_MetaData_msg()
	{
		pc_maker::CloudMetaData msg;
		msg.header.stamp = ros::Time::now();
		msg.x_max = X_limit;
		msg.y_max =Y_limit; 
		msg.z_min = Z_d_limit;
		msg.z_max = Z_u_limit;
		msg.roll_anlge = roll_anlge;
		RL_cloud_Metadata_pub_.publish(msg);
	}
	

  void run()
	{

		ros::NodeHandle n_pr("~");	
		
		n_pr.param("X_Limit", X_limit, 3.00);
		n_pr.param("Y_Limit", Y_limit, 1.50);
		n_pr.param("Z_u_Limit", Z_u_limit, 0.50);
		n_pr.param("Z_d_Limit", Z_d_limit, -0.4);
		n_pr.param("voxel_filter", voxel_filter, 0.005);

		
    while(ros::ok())
    {
      ros::spinOnce();
      ros::Duration(1/50).sleep();
    }
		//ros::spin ();
	
	}
  	

	protected:
	
	// Node Handler
	ros::NodeHandle n_;

	// Subscribers
	ros::Subscriber SubFromScannerInfo_;
	ros::Subscriber SubFromScannerangle_;
	ros::Subscriber SubFromLaser_;
  ros::Subscriber SubFromScanTilt_;
	// Publishers
	ros::Publisher Right_cloud_pub_;
	ros::Publisher Left_cloud_pub_;
	ros::Publisher RL_cloud_pub_;
	ros::Publisher RL_cloud_Metadata_pub_;
	//thresholds
	double X_limit;
	double Y_limit;
	double Z_u_limit;
	double Z_d_limit;
	double voxel_filter;
  std::vector<LaserBim> scanstack;
	
	
	// Services
		
	//Class Global Variables
	
	Eigen::Affine3f transform;
	float roll_anlge; 
	pcl::PointCloud<pcl::PointXYZ> inc_cloud_2d;
	pcl::PointCloud<pcl::PointXYZ> inc_cloud_3d_left;
	pcl::PointCloud<pcl::PointXYZ> inc_cloud_3d_right;
	pcl::PointCloud<pcl::PointXYZ> inc_cloud_3d;
	float Scanner_angle_curr;
	float Scanner_angle_last;
	bool left_published;
	bool right_published;
  int scanAngleEnc;
  int scanAngleEnc_last;
  int scanEncMax;
  scannerSTATE ScannerState;
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pc_ground_plane");
	ros::NodeHandle node;

	LaserToPcClass laser_to_pc (node);
	
	laser_to_pc.run();
	
	return 0;
}

	
		
