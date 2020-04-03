#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <vector>
#include <string>
#include <array>
#include <cmath>
#include <limits>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <math.h>
#include <geometry_msgs/PolygonStamped.h>
#include "grid_map_demos/IteratorsDemo.hpp"

//#include<message package>


using namespace grid_map;

class Imudata {
public:
  double w; 
  double x;//std_msgs::Float64 x; 
  double y;//std_msgs::Float64 y; 
  double z;//std_msgs::Float64 z; 
  double a;//std_msgs::Float64 a; 
  double b;//std_msgs::Float64 b; 
  double c;//std_msgs::Float64 c; 
  double r;//std_msgs::Float64 r; 
  double s;//std_msgs::Float64 s;
  void ImuCallback(const sensor_msgs::Imu Imu_points);
};

Imudata Imu_data;
#define ARRAY_MAX 10000 
//std::vector<int> myArray_w;
double myArray_w[ARRAY_MAX], sum_w = 0;
unsigned int arr_length_w = 0;
double myArray_x[ARRAY_MAX], sum_x = 0;
unsigned int arr_length_x = 0;
double myArray_y[ARRAY_MAX], sum_y = 0;
unsigned int arr_length_y = 0;
double myArray_z[ARRAY_MAX], sum_z = 0;
unsigned int arr_length_z = 0;
double myArray_r[ARRAY_MAX], sum_r = 0;
unsigned int arr_length_r = 0;
double myArray_s[ARRAY_MAX], sum_s = 0;
unsigned int arr_length_s = 0;


void ImuCallback(const sensor_msgs::Imu Imu_points)
    {
      Imu_data.w = Imu_points.orientation.w;
      Imu_data.x = Imu_points.orientation.x;
      Imu_data.y = Imu_points.orientation.y;
      Imu_data.z = Imu_points.orientation.z;
      Imu_data.a = Imu_points.angular_velocity.x;
      Imu_data.b = Imu_points.angular_velocity.y;
      Imu_data.c = Imu_points.angular_velocity.z;

      if (Imu_data.a <= 0.003 && Imu_data.b <= 0.003 && Imu_data.c <= 0.003
      && Imu_data.a >= -0.003 && Imu_data.b >= -0.003 && Imu_data.c >= -0.003) {

        if (arr_length_w < ARRAY_MAX) {
          myArray_w[arr_length_w++] = Imu_data.w;
          myArray_x[arr_length_x++] = Imu_data.x;
          myArray_y[arr_length_y++] = Imu_data.y;
          myArray_z[arr_length_z++] = Imu_data.z;
          myArray_r[arr_length_r++] = Imu_data.r;
          myArray_s[arr_length_s++] = Imu_data.s;
          //arr_length_w ++; 
        }
        else{}
      }
      else {                                            // reset everything
        for(int erase = 0;erase < ARRAY_MAX;erase++) {  //clear the array
          myArray_w[erase] = '\n';
          myArray_x[erase] = '\n';
          myArray_y[erase] = '\n';
          myArray_z[erase] = '\n';
          myArray_r[erase] = '\n';
          myArray_s[erase] = '\n';
        }
        arr_length_w = 0;           // reset the array length to zero
        arr_length_x = 0;
        arr_length_y = 0;
        arr_length_z = 0;
        arr_length_r = 0;
        arr_length_s = 0;
        sum_w=0;
        sum_x=0;
        sum_y=0;
        sum_z=0;
        sum_r=0;
        sum_s=0;
      }
        /*for (int i=0; i<ARRAY_MAX;i++){             //check the array values
            ROS_INFO("w_array: [%f]", myArray_w[i]);
        }*/
    }

void OdomCallback(const nav_msgs::Odometry Odom_points)
    {
      Imu_data.r = Odom_points.pose.pose.position.x;
      Imu_data.s = Odom_points.pose.pose.position.y;
      //ROS_INFO("I Odom: [%f]", Imu_data.r);
    }

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_eg4301_demo");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  ros::Subscriber sub1 = nh.subscribe("/imu", 10, ImuCallback);
  ros::Subscriber sub2 = nh.subscribe("/odom", 10, OdomCallback);

  //std::vector<double> myArray_w;
  /*double sum_x = 0;
  double sum_y = 0;
  double sum_z = 0;
  double sum_r = 0;
  double sum_s = 0;*/
  double average_w; //define the term
  double average_x; //define the term
  double average_y; //define the term
  double average_z; //define the term
  double average_r; //define the term
  double average_s; //define the term
  double roll;
  double pitch;
  double angle;
  double yaw;
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

    // Add elevation and surface normal (iterating through grid map and adding data).
    /*for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      map.at("elevation", *it) = -0.3 + 2 * std::sin(position.y()/4-0.5) * std::sin( position.x()/3-0.5) ; 
      Eigen::Vector3d normal(position.y(), position.x() , 1.0);
      normal.normalize();
      map.at("normal_x", *it) = normal.x();
      map.at("normal_y", *it) = normal.y();
      map.at("normal_z", *it) = normal.z();
    }*/
	/*for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      geometry_msgs::Odometry odometry;
      if (odometry.pose.pose.position.x() == position.x() && odometry.pose.pose.position.y() == position.y()){
      	map.at("elevation", *it) = calculated elevation; 
      }
      Eigen::Vector3d normal(position.y(), position.x() , 1.0);
      normal.normalize();
      map.at("normal_x", *it) = normal.x();
      map.at("normal_y", *it) = normal.y();
      map.at("normal_z", *it) = normal.z();*/
    // Add elevation and surface normal (iterating through grid map and adding data).

    if (Imu_data.a <= 0.003 && Imu_data.b <= 0.003 && Imu_data.c <= 0.003
      && Imu_data.a >= -0.003 && Imu_data.b >= -0.003 && Imu_data.c >= -0.003) {


      /*if (arr_length_w < ARRAY_MAX) {
        myArray_w[arr_length_w++] = Imu_data.w;
      } else {}

      for (int i=0; i<ARRAY_MAX; ++i){
          ROS_INFO("w_array: [%f]", myArray_w[i]);
      }*/
      sum_w = 0;
      sum_x = 0;
      sum_y = 0;
      sum_z = 0;
      sum_r = 0;
      sum_s = 0;

    for (int i=0; i<arr_length_w; i++) {
    
      sum_w += myArray_w[i];
      sum_x += myArray_x[i];
      sum_y += myArray_y[i];
      sum_z += myArray_z[i];
      sum_r += myArray_r[i]; //x-axis
      sum_s += myArray_s[i]; //y-axis
    } 

    average_w = sum_w/arr_length_w;
    average_x = sum_x/arr_length_x;
    average_y = sum_y/arr_length_y;
    average_z = sum_z/arr_length_z;
    average_r = sum_r/arr_length_r;
    average_s = sum_s/arr_length_s;

    //ROS_INFO("w =" average_w, "x =" average_x, "y =" average_y, "z =" average_z)  
    //ROS_INFO("w = %f", average_w);
    /*ROS_INFO("sum_w = %f", sum_w);
    ROS_INFO("arr_length_w = %d", arr_length_w);
    ROS_INFO("inclination = %f", inclination);*/
    //ROS_INFO("x = %f", average_x);
    //ROS_INFO("y = %f", average_y);
    //ROS_INFO("z = %f", yaw);

    position.x() = average_r;//Imu_data.r;
    position.y() = average_s;//Imu_data.s;
    roll = atan(2*(average_w*average_x+average_y*average_z)/(1-2*(average_x*average_x+average_y*average_y)))*180/M_PI;
    pitch = asin(2*(average_w*average_y - average_z*average_x))*180/M_PI;
    yaw = atan2(2*(average_w*average_z+average_x*average_y), (1-2*(average_z*average_z+average_y*average_y)))*180/M_PI;
    //double inclination = atan(sqrt((tan(roll))*(tan(roll))+(tan(pitch))*(tan(pitch))));

    //taking roll as rotation in y-axis ,pitch in x-axis, calculate inclination from rms of roll and pitch based on 1st, 2nd quadrant
    if (roll <= 0 && pitch <= 0) { //4th quadrant highest, 2nd quad lowest, inclination negative
        inclination = -sqrt((roll*roll)+(pitch*pitch));
    }
        else if (roll >= 0 && pitch >= 0) { //2nd quad elevated, inclination positive
          inclination = sqrt((roll*roll)+(pitch*pitch));
    }
        else if (roll >= 0 && pitch <= 0) { //3rd quad highest, 1st quad lowest, inclination negative
          inclination = -sqrt((roll*roll)+(pitch*pitch));
    }
        else if (roll <= 0 && pitch >= 0) { //1st quad elevated, inclination positive
            inclination = sqrt((roll*roll)+(pitch*pitch));
    }
    ROS_INFO("roll = %f", roll);
    ROS_INFO("pitch = %f", pitch);


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
  /*if (yaw >= 0 && yaw <= 50) { //robot facing upwards

    polygon.addVertex(Position( position.x()+0.3*roll,  position.y()+0.3*pitch));
    polygon.addVertex(Position( position.x()+0.2*roll-0.10*roll,  position.y()+0.2*pitch+0.10*roll*roll/pitch));
    polygon.addVertex(Position( position.x()+0.2*roll-0.025*roll,  position.y()+0.2*pitch+0.025*roll*roll/pitch));
    polygon.addVertex(Position( position.x()-0.025*roll,  position.y()+0.025*roll*roll/pitch));
    polygon.addVertex(Position( position.x(),  position.y()));
    polygon.addVertex(Position( position.x()+0.025*roll,  position.y()-0.025*roll*roll/pitch));
    polygon.addVertex(Position( position.x()+0.2*roll+0.025*roll,  position.y()+0.2*pitch-0.025*roll*roll/pitch));
    polygon.addVertex(Position( position.x()+0.2*roll+0.10*roll,  position.y()+0.2*pitch-0.10*roll*roll/pitch));

  } else if (yaw >= 310 && yaw <= 360) { //robot facing upwards
    polygon.addVertex(Position( position.x()+0.3*roll,  position.y()+0.3*pitch));
    polygon.addVertex(Position( position.x()+0.2*roll-0.10*roll,  position.y()+0.2*pitch+0.10*roll*roll/pitch));
    polygon.addVertex(Position( position.x()+0.2*roll-0.025*roll,  position.y()+0.2*pitch+0.025*roll*roll/pitch));
    polygon.addVertex(Position( position.x()-0.025*roll,  position.y()+0.025*roll*roll/pitch));
    polygon.addVertex(Position( position.x(),  position.y()));
    polygon.addVertex(Position( position.x()+0.025*roll,  position.y()-0.025*roll*roll/pitch));
    polygon.addVertex(Position( position.x()+0.2*roll+0.025*roll,  position.y()+0.2*pitch-0.025*roll*roll/pitch));
    polygon.addVertex(Position( position.x()+0.2*roll+0.10*roll,  position.y()+0.2*pitch-0.10*roll*roll/pitch));  

  } else if (yaw >= 130 && yaw <= 230){ //robot facing downwards
    double roll_ = -roll;
    double pitch_ = -pitch; 
    polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
    polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
    polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
    polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
    polygon.addVertex(Position( position.x(),  position.y()));
    polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
    polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
    polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));

  } else if (yaw > 50 && yaw < 130){ //robot facing leftwards
    double roll_ = pitch;
    double pitch_ = -roll;
    polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
    polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
    polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
    polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
    polygon.addVertex(Position( position.x(),  position.y()));
    polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
    polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
    polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));

  } else  {//(yaw > 230 && yaw < 310){ //robot facing rightwards
    double roll_ = -pitch;
    double pitch_ = roll;
    polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
    polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
    polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
    polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
    polygon.addVertex(Position( position.x(),  position.y()));
    polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
    polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
    polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));

  }*/
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


  //4 conditions to determine the robot's orientation
  double initial_yaw = 40;  //need to change for the turtlebot
  double range_1a =  initial_yaw + 315;
  //double range_1b =  initial_yaw + 45;
  double range_2a =  initial_yaw + 45;
  //double range_2b =  initial_yaw + 135;
  double range_3a =  initial_yaw + 135;
  //double range_3b =  initial_yaw + 225;
  double range_4a =  initial_yaw + 225;
  //double range_4b =  initial_yaw + 315;


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

  /*if (diff_1 < 90) {
    if (yaw >= range_1aa && yaw <= 360) { //robot facing upwards
      polygon.addVertex(Position( position.x()+0.3*roll,  position.y()+0.3*pitch));
      polygon.addVertex(Position( position.x()+0.2*roll-0.10*roll,  position.y()+0.2*pitch+0.10*roll*roll/pitch));
      polygon.addVertex(Position( position.x()+0.2*roll-0.025*roll,  position.y()+0.2*pitch+0.025*roll*roll/pitch));
      polygon.addVertex(Position( position.x()-0.025*roll,  position.y()+0.025*roll*roll/pitch));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll,  position.y()-0.025*roll*roll/pitch));
      polygon.addVertex(Position( position.x()+0.2*roll+0.025*roll,  position.y()+0.2*pitch-0.025*roll*roll/pitch));
      polygon.addVertex(Position( position.x()+0.2*roll+0.10*roll,  position.y()+0.2*pitch-0.10*roll*roll/pitch));

  } else if (yaw >= 0 && yaw <= range_1bb){ //robot facing downwards
      polygon.addVertex(Position( position.x()+0.3*roll,  position.y()+0.3*pitch));
      polygon.addVertex(Position( position.x()+0.2*roll-0.10*roll,  position.y()+0.2*pitch+0.10*roll*roll/pitch));
      polygon.addVertex(Position( position.x()+0.2*roll-0.025*roll,  position.y()+0.2*pitch+0.025*roll*roll/pitch));
      polygon.addVertex(Position( position.x()-0.025*roll,  position.y()+0.025*roll*roll/pitch));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll,  position.y()-0.025*roll*roll/pitch));
      polygon.addVertex(Position( position.x()+0.2*roll+0.025*roll,  position.y()+0.2*pitch-0.025*roll*roll/pitch));
      polygon.addVertex(Position( position.x()+0.2*roll+0.10*roll,  position.y()+0.2*pitch-0.10*roll*roll/pitch));

    } else if (yaw >= range_3aa && yaw <= range_3bb){ //robot facing downwards
      double roll_ = -roll;
      double pitch_ = -pitch; 
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));

    } else if (yaw > range_4aa && yaw < range_4bb){ //robot facing leftwards
      double roll_ = pitch;
      double pitch_ = -roll;
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));

    } else  {//(yaw > 230 && yaw < 310){ //robot facing rightwards
      double roll_ = -pitch;
      double pitch_ = roll;
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));
    }
  
  } else { //diff_1 = 90
    }

  if (diff_3 < 90) {
    if ( yaw >= range_1aa &&  yaw <= range_1bb) { //robot facing upwards
      polygon.addVertex(Position( position.x()+0.3* roll,  position.y()+0.3* pitch));
      polygon.addVertex(Position( position.x()+0.2* roll-0.10* roll,  position.y()+0.2* pitch+0.10* roll* roll/ pitch));
      polygon.addVertex(Position( position.x()+0.2* roll-0.025* roll,  position.y()+0.2* pitch+0.025* roll* roll/ pitch));
      polygon.addVertex(Position( position.x()-0.025* roll,  position.y()+0.025* roll* roll/ pitch));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025* roll,  position.y()-0.025* roll* roll/ pitch));
      polygon.addVertex(Position( position.x()+0.2* roll+0.025* roll,  position.y()+0.2* pitch-0.025* roll* roll/ pitch));
      polygon.addVertex(Position( position.x()+0.2* roll+0.10* roll,  position.y()+0.2* pitch-0.10* roll* roll/ pitch));

    } else if ( yaw >= range_3aa &&  yaw <= 360){ //robot facing downwards
      double roll_ = - roll;
      double pitch_ = - pitch; 
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));

    } else if ( yaw >= 0 &&  yaw <= range_3bb){ //robot facing downwards
      double roll_ = - roll;
      double pitch_ = - pitch; 
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));

    } else if ( yaw > range_4aa &&  yaw < range_4bb){ //robot facing leftwards
      double roll_ =  pitch;
      double pitch_ = - roll;
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));

    } else  {//(yaw > 230 && yaw < 310){ //robot facing rightwards
      double roll_ = - pitch;
      double pitch_ =  roll;
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));
    }
  
  } else { //diff_3 = 90
    }
   
  if (diff_4 < 90) {
    if ( yaw >= range_1aa &&  yaw <= range_1bb) { //robot facing upwards
      polygon.addVertex(Position( position.x()+0.3* roll,  position.y()+0.3* pitch));
      polygon.addVertex(Position( position.x()+0.2* roll-0.10* roll,  position.y()+0.2* pitch+0.10* roll* roll/ pitch));
      polygon.addVertex(Position( position.x()+0.2* roll-0.025* roll,  position.y()+0.2* pitch+0.025* roll* roll/ pitch));
      polygon.addVertex(Position( position.x()-0.025* roll,  position.y()+0.025* roll* roll/ pitch));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025* roll,  position.y()-0.025* roll* roll/ pitch));
      polygon.addVertex(Position( position.x()+0.2* roll+0.025* roll,  position.y()+0.2* pitch-0.025* roll* roll/ pitch));
      polygon.addVertex(Position( position.x()+0.2* roll+0.10* roll,  position.y()+0.2* pitch-0.10* roll* roll/ pitch));

    } else if ( yaw >= range_3aa &&  yaw <= range_3bb){ //robot facing downwards
      double roll_ = - roll;
      double pitch_ = - pitch; 
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));

    } else if ( yaw > range_4aa &&  yaw < 360){ //robot facing leftwards
      double roll_ =  pitch;
      double pitch_ = - roll;
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));

    } else if ( yaw >= 0 &&  yaw < range_4bb){ //robot facing leftwards
      double roll_ =  pitch;
      double pitch_ = - roll;
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));

    } else  {//(yaw > 230 && yaw < 310){ //robot facing rightwards
      double roll_ = - pitch;
      double pitch_ =  roll;
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));
    }
  
  } else { //diff_4 = 90
    }

  if (diff_2 < 90) {
    if ( yaw >= range_1aa &&  yaw <= range_1bb) { //robot facing upwards
      polygon.addVertex(Position( position.x()+0.3* roll,  position.y()+0.3* pitch));
      polygon.addVertex(Position( position.x()+0.2* roll-0.10* roll,  position.y()+0.2* pitch+0.10* roll* roll/ pitch));
      polygon.addVertex(Position( position.x()+0.2* roll-0.025* roll,  position.y()+0.2* pitch+0.025* roll* roll/ pitch));
      polygon.addVertex(Position( position.x()-0.025* roll,  position.y()+0.025* roll* roll/ pitch));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025* roll,  position.y()-0.025* roll* roll/ pitch));
      polygon.addVertex(Position( position.x()+0.2* roll+0.025* roll,  position.y()+0.2* pitch-0.025* roll* roll/ pitch));
      polygon.addVertex(Position( position.x()+0.2* roll+0.10* roll,  position.y()+0.2* pitch-0.10* roll* roll/ pitch));

    } else if ( yaw >= range_3aa &&  yaw <= range_3bb){ //robot facing downwards
      double roll_ = - roll;
      double pitch_ = - pitch; 
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));

    } else if ( yaw > range_4aa &&  yaw < range_4bb){ //robot facing leftwards
      double roll_ =  pitch;
      double pitch_ = - roll;
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));

    } else  {//(yaw > 230 && yaw < 310){ //robot facing rightwards
      double roll_ = - pitch;
      double pitch_ =  roll;
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10*roll_,  position.y()+0.2*pitch_+0.10*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025*roll_,  position.y()+0.2*pitch_+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025*roll_,  position.y()+0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025*roll_,  position.y()-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025*roll_,  position.y()+0.2*pitch_-0.025*roll_*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10*roll_,  position.y()+0.2*pitch_-0.10*roll_*roll_/pitch_));
    }
  
  } else { //diff_2 = 90
    }*/

  if (diff_1 < 90) {
    if (yaw >= range_1aa && yaw <= 360) { //robot facing upwards
      double roll_ = -roll;
      double pitch_ = -pitch;      
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

  } else if (yaw >= 0 && yaw <= range_1bb){ //robot facing downwards
      double roll_ = -roll;
      double pitch_ = -pitch;        
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

    } else if (yaw >= range_3aa && yaw <= range_3bb){ //robot facing downwards
      double roll_ = roll;
      double pitch_ = pitch; 
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

    } else if (yaw > range_4aa && yaw < range_4bb){ //robot facing leftwards
      double roll_ = -pitch;
      double pitch_ = roll;
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

    } else  {//(yaw > 230 && yaw < 310){ //robot facing rightwards
      double roll_ = pitch;
      double pitch_ = -roll;
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
    if ( yaw >= range_1aa &&  yaw <= range_1bb) { //robot facing upwards
      double roll_ = -roll;
      double pitch_ = -pitch;      
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

    } else if ( yaw >= range_3aa &&  yaw <= 360){ //robot facing downwards
      double roll_ =  roll;
      double pitch_ =  pitch; 
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

    } else if ( yaw >= 0 &&  yaw <= range_3bb){ //robot facing downwards
      double roll_ =  roll;
      double pitch_ =  pitch; 
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

    } else if ( yaw > range_4aa &&  yaw < range_4bb){ //robot facing leftwards
      double roll_ =  -pitch;
      double pitch_ =  roll;
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));
    } else  {//(yaw > 230 && yaw < 310){ //robot facing rightwards
      double roll_ =  pitch;
      double pitch_ = - roll;
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
    if ( yaw >= range_1aa &&  yaw <= range_1bb) { //robot facing upwards
      double roll_ = -roll;
      double pitch_ = -pitch;   
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

    } else if ( yaw >= range_3aa &&  yaw <= range_3bb){ //robot facing downwards
      double roll_ = roll;
      double pitch_ =  pitch; 
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

    } else if ( yaw > range_4aa &&  yaw < 360){ //robot facing leftwards
      double roll_ =  -pitch;
      double pitch_ =  roll;
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

    } else if ( yaw >= 0 &&  yaw < range_4bb){ //robot facing leftwards
      double roll_ =  -pitch;
      double pitch_ =  roll;
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

    } else  {//(yaw > 230 && yaw < 310){ //robot facing rightwards
      double roll_ =  pitch;
      double pitch_ =  -roll;
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
    if ( yaw >= range_1aa &&  yaw <= range_1bb) { //robot facing upwards
      double roll_ = -roll;
      double pitch_ = -pitch;   
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

    } else if ( yaw >= range_3aa &&  yaw <= range_3bb){ //robot facing downwards
      double roll_ =  roll;
      double pitch_ =  pitch; 
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

    } else if ( yaw > range_4aa &&  yaw < range_4bb){ //robot facing leftwards
      double roll_ =  -pitch;
      double pitch_ =  roll;
      polygon.addVertex(Position( position.x()+0.3*roll_,  position.y()+0.3*pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.10,  position.y()+0.2*pitch_+0.10*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_-0.025,  position.y()+0.2*pitch_+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()-0.025,  position.y()+0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x(),  position.y()));
      polygon.addVertex(Position( position.x()+0.025,  position.y()-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.025,  position.y()+0.2*pitch_-0.025*roll_/pitch_));
      polygon.addVertex(Position( position.x()+0.2*roll_+0.10,  position.y()+0.2*pitch_-0.10*roll_/pitch_));

    } else  {//(yaw > 230 && yaw < 310){ //robot facing rightwards
      double roll_ =  pitch;
      double pitch_ = - roll;
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