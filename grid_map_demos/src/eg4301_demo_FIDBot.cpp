#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <vector>
#include <string>
#include <array>
#include <cmath>
#include <limits>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <math.h>
#include <geometry_msgs/PolygonStamped.h>
#include "grid_map_demos/IteratorsDemo.hpp"

//#include<message package>


using namespace grid_map;

class sensordata {
public:
  double roll;
  double pitch;
  double yaw;
  double initial_yaw;
  double x;
  double y;   
  double a;
  double b;
  double c; 

};

sensordata Sensor_data;


void RPYCallback(const std_msgs::Float64MultiArray RPY_points)//roll, pitch, yaw
    {
      Sensor_data.roll = RPY_points.data[0];
      Sensor_data.pitch = RPY_points.data[1];
      Sensor_data.yaw = RPY_points.data[2];
       //ROS_INFO("pitch123 = %f", Sensor_data.pitch);
       //ROS_INFO("pitch123736248 = %f", RPY_points.data[1]);

    }

void ImuCallback(const sensor_msgs::Imu Imu_points)
    {
      Sensor_data.a = Imu_points.angular_velocity.x;
      Sensor_data.b = Imu_points.angular_velocity.y;
      Sensor_data.c = Imu_points.angular_velocity.z;
      //ROS_INFO("pitch123736248 = %f", Sensor_data.a);
    }

void OdomCallback(const nav_msgs::Odometry Odom_points)
    {
      Sensor_data.x = Odom_points.pose.pose.position.x;
      Sensor_data.y = Odom_points.pose.pose.position.y;
      //ROS_INFO("I Odom: [%f]", Imu_data.r);
    }

void YawCallback(const std_msgs::Float64 Yaw_points)
    {
      Sensor_data.initial_yaw = Yaw_points.data;
    }

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_eg4301_demo_FIDBot");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  ros::Subscriber sub1 = nh.subscribe("/MPU9250_array", 100, RPYCallback);
  ros::Subscriber sub3 = nh.subscribe("/imu", 100, ImuCallback);
  ros::Subscriber sub2 = nh.subscribe("/odom", 100, OdomCallback);
  ros::Subscriber sub4 = nh.subscribe("/MPU9250_heading", 100, YawCallback);


  double inclination;

  //polygonPublisher_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>("polygon", 1, true);

  // Create grid map.
  GridMap map({"elevation", "elevation_arrow", "normal_x", "normal_y", "normal_z"});
  map.setFrameId("map");
  //map.setGeometry(Length(28.8,22.8), 0.03, Position(0.0, -0.1));
  map.setGeometry(Length(20,20), 0.03, Position(0, -0.1));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1),
    map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());

  // Work with grid map in a loop.
  ros::Rate rate(10.0);
  while (nh.ok()) {
    ros::Time time = ros::Time::now();
    Position position;

    if (Sensor_data.a <= 0.2 && Sensor_data.b <= 0.2 && Sensor_data.c <= 0.2
      && Sensor_data.a >= -0.2 && Sensor_data.b >= -0.2 && Sensor_data.c >= -0.2) {

      position.x() = Sensor_data.x;
      position.y() = Sensor_data.y;
    
      //taking roll as rotation in y-axis ,pitch in x-axis, calculate inclination from rms of roll and pitch based on 1st, 2nd quadrant
      if (Sensor_data.roll <= 0 && Sensor_data.pitch <= 0) { //4th quadrant highest, 2nd quad lowest, inclination negative
        inclination = -sqrt((Sensor_data.roll*Sensor_data.roll)+(Sensor_data.pitch*Sensor_data.pitch));
      }
        else if (Sensor_data.roll >= 0 && Sensor_data.pitch >= 0) { //2nd quad elevated, inclination positive
          inclination = sqrt((Sensor_data.roll*Sensor_data.roll)+(Sensor_data.pitch*Sensor_data.pitch));
      }
        else if (Sensor_data.roll >= 0 && Sensor_data.pitch <= 0) { //3rd quad highest, 1st quad lowest, inclination negative
          inclination = -sqrt((Sensor_data.roll*Sensor_data.roll)+(Sensor_data.pitch*Sensor_data.pitch));
      }
        else if (Sensor_data.roll <= 0 && Sensor_data.pitch >= 0) { //1st quad elevated, inclination positive
          inclination = sqrt((Sensor_data.roll*Sensor_data.roll)+(Sensor_data.pitch*Sensor_data.pitch));
      }
      ROS_INFO("roll = %f", Sensor_data.roll);
      ROS_INFO("pitch = %f", Sensor_data.pitch);
    

  //using only the coordinate to mark out points of pass and fail
    //map.atPosition("elevation", position) = inclination;

  //using circle to mark out points of pass and fail
   Position center(position.x(), position.y());  
    double radius = 0.2;

    for (grid_map::CircleIterator iterator(map, center, radius);
          !iterator.isPastEnd(); ++iterator) {
           map.at("elevation", *iterator) = inclination;
      }

    
  //using arrow as the indicator
  grid_map::Polygon polygon;  //polygon to create an arrow
  polygon.setFrameId(map.getFrameId());


   // position1.x()= average_r ;
   // position1.y()= average_s ;
   // position2.x()= average_r + 0.8*roll-0.1*roll;
   // position2.y()= average_s +0.8*pitch+0.1*roll*roll/pitch;
   // position3.x()= average_r +0.8*roll+0.1*roll;
   // position3.y()= average_s +0.8*pitch-0.1*roll*roll/pitch;


  //4 conditions to determine the robot's orientation
  double range_1a = Sensor_data.initial_yaw + 315;
  //double range_1b = Sensor_data.initial_yaw + 45;
  double range_2a = Sensor_data.initial_yaw + 45;
  //double range_2b = Sensor_data.initial_yaw + 135;
  double range_3a = Sensor_data.initial_yaw + 135;
  //double range_3b = Sensor_data.initial_yaw + 225;
  double range_4a = Sensor_data.initial_yaw + 225;
  //double range_4b = Sensor_data.initial_yaw + 315;


  double range_1aa;
  double range_1bb;
  double range_2aa;
  double range_2bb;
  double range_3aa;
  double range_3bb;
  double range_4aa;
  double range_4bb;



  if (range_1a >= 360) {
  	range_1aa = range_1a - 360;
  	range_4bb = range_1a - 360;
  } else {
  	range_1aa = range_1a;
  	range_4bb = range_1a;
  }

  if (range_2a >= 360) {
  	range_2aa = range_2a - 360;
  	range_1bb = range_2a - 360;
  } else {
  	range_2aa = range_2a;
  	range_1bb = range_2a;
  }

  if (range_3a >= 360) {
  	range_3aa = range_3a - 360;
  	range_2bb = range_3a - 360;
  } else {
  	range_3aa = range_3a;
  	range_2bb = range_3a;
  }

  if (range_4a >= 360) {
  	range_4aa = range_4a - 360;
  	range_3bb = range_4a - 360;
  } else {
  	range_4aa = range_4a;
  	range_3bb = range_4a;
  }

  double diff_1 = range_1bb - range_1aa;
  double diff_2 = range_2bb - range_2aa;
  double diff_3 = range_3bb - range_3aa;
  double diff_4 = range_4bb - range_4aa;

  if (diff_1 < 90) {
  	if (Sensor_data.yaw >= range_1aa && Sensor_data.yaw <= 360) { //robot facing upwards
    	double roll_ = -Sensor_data.roll;
    	double pitch_ = -Sensor_data.pitch;   		
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

	} else if (Sensor_data.yaw >= 0 && Sensor_data.yaw <= range_1bb){ //robot facing downwards
		double roll_ = -Sensor_data.roll;
    	double pitch_ = -Sensor_data.pitch;  
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

  	} else if (Sensor_data.yaw >= range_3aa && Sensor_data.yaw <= range_3bb){ //robot facing downwards
    	double roll_ = Sensor_data.roll;
    	double pitch_ = Sensor_data.pitch; 
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

  	} else if (Sensor_data.yaw > range_4aa && Sensor_data.yaw < range_4bb){ //robot facing leftwards
    	double roll_ = -Sensor_data.pitch;
    	double pitch_ = Sensor_data.roll;
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

  	} else  {//(yaw > 230 && yaw < 310){ //robot facing rightwards
    	double roll_ = Sensor_data.pitch;
    	double pitch_ = -Sensor_data.roll;
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));
  	}
  
  } else { //diff_1 = 90
  	}

  if (diff_3 < 90) {
  	if (Sensor_data.yaw >= range_1aa && Sensor_data.yaw <= range_1bb) { //robot facing upwards
    	double roll_ = -Sensor_data.roll;
    	double pitch_ = -Sensor_data.pitch;       	
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

  	} else if (Sensor_data.yaw >= range_3aa && Sensor_data.yaw <= 360){ //robot facing downwards
    	double roll_ = Sensor_data.roll;
    	double pitch_ = Sensor_data.pitch; 
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

  	} else if (Sensor_data.yaw >= 0 && Sensor_data.yaw <= range_3bb){ //robot facing downwards
    	double roll_ = Sensor_data.roll;
    	double pitch_ = Sensor_data.pitch; 
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

  	} else if (Sensor_data.yaw > range_4aa && Sensor_data.yaw < range_4bb){ //robot facing leftwards
    	double roll_ = -Sensor_data.pitch;
    	double pitch_ = Sensor_data.roll;
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

  	} else  {//(yaw > 230 && yaw < 310){ //robot facing rightwards
    	double roll_ = Sensor_data.pitch;
    	double pitch_ = -Sensor_data.roll;
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));
  	}
  
  } else { //diff_3 = 90
  	}
   
  if (diff_4 < 90) {
  	if (Sensor_data.yaw >= range_1aa && Sensor_data.yaw <= range_1bb) { //robot facing upwards
    	double roll_ = -Sensor_data.roll;
    	double pitch_ = -Sensor_data.pitch;
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

  	} else if (Sensor_data.yaw >= range_3aa && Sensor_data.yaw <= range_3bb){ //robot facing downwards
    	double roll_ = Sensor_data.roll;
    	double pitch_ = Sensor_data.pitch; 
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

  	} else if (Sensor_data.yaw > range_4aa && Sensor_data.yaw < 360){ //robot facing leftwards
    	double roll_ = -Sensor_data.pitch;
    	double pitch_ = Sensor_data.roll;
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

  	} else if (Sensor_data.yaw >= 0 && Sensor_data.yaw < range_4bb){ //robot facing leftwards
    	double roll_ = -Sensor_data.pitch;
    	double pitch_ = Sensor_data.roll;
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

  	} else  {//(yaw > 230 && yaw < 310){ //robot facing rightwards
    	double roll_ = Sensor_data.pitch;
    	double pitch_ = -Sensor_data.roll;
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));
  	}
  
  } else { //diff_4 = 90
  	}

  if (diff_2 < 90) {
  	if (Sensor_data.yaw >= range_1aa && Sensor_data.yaw <= range_1bb) { //robot facing upwards
    	double roll_ = -Sensor_data.roll;
    	double pitch_ = -Sensor_data.pitch;     	
    	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

  	} else if (Sensor_data.yaw >= range_3aa && Sensor_data.yaw <= range_3bb){ //robot facing downwards
    	double roll_ = Sensor_data.roll;
    	double pitch_ = Sensor_data.pitch; 
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

  	} else if (Sensor_data.yaw > range_4aa && Sensor_data.yaw < range_4bb){ //robot facing leftwards
    	double roll_ = -Sensor_data.pitch;
    	double pitch_ = Sensor_data.roll;
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

  	} else  {//(yaw > 230 && yaw < 310){ //robot facing rightwards
    	double roll_ = Sensor_data.pitch;
    	double pitch_ = -Sensor_data.roll;
      	polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x(),  position.y()));
      	polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      	polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));
  	}
  
  } else { //diff_2 = 90
  	}

  /*if (inclination <= 0) { //up facing arrow
    polygon.addVertex(Position( position.x(),  position.y()+0.2));
    polygon.addVertex(Position( position.x()-0.15,  position.y()+0.05));
    polygon.addVertex(Position( position.x()-0.05,  position.y()+0.05));
    polygon.addVertex(Position( position.x()-0.05,  position.y()-0.2));
    polygon.addVertex(Position( position.x(),  position.y()-0.2));
    polygon.addVertex(Position( position.x()+0.05,  position.y()-0.2));
    polygon.addVertex(Position( position.x()+0.05,  position.y()+0.05));
    polygon.addVertex(Position( position.x()+0.15,  position.y()+0.05));
  } else { //down facing arrow
    polygon.addVertex(Position( position.x(),  position.y()-0.2));
    polygon.addVertex(Position( position.x()-0.15,  position.y()-0.05));
    polygon.addVertex(Position( position.x()-0.05,  position.y()-0.05));
    polygon.addVertex(Position( position.x()-0.05,  position.y()+0.2));
    polygon.addVertex(Position( position.x(),  position.y()+0.2));
    polygon.addVertex(Position( position.x()+0.05,  position.y()+0.2));
    polygon.addVertex(Position( position.x()+0.05,  position.y()-0.05));
    polygon.addVertex(Position( position.x()+0.15,  position.y()-0.05));    
  }*/


  for (grid_map::PolygonIterator iterator(map, polygon);
      !iterator.isPastEnd(); ++iterator) {
    map.at("elevation_arrow", *iterator) = inclination;
  }
  }
   

    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
    ros::spinOnce();
    // Wait for next cycle.
    rate.sleep();
  }

  return 0;
}