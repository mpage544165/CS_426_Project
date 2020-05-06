
/*  @File: World_view_interpreter.cpp

    @Brief: Generates a 3D map of the drone's environment using the LiDAR and Octomap Library

    @Author: Matthew Page

    @Date: 1/27/2020

*/

#include <math.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Float32.h"
#include <nav_msgs/Odometry.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

class WorldViewInterpreter{

  // Declare pointclouds
  std::vector<geometry_msgs::Point32> pointCloud;
  sensor_msgs::PointCloud rosPointCloud;

  // Declare ROS node, publishers, and subscribers
  ros::NodeHandle node;
  ros::Subscriber laserSub;
  ros::Subscriber lidarOrientationSub;
  ros::Subscriber navDataSub;
  ros::Publisher pointCloudPub;
  ros::Rate loop_rate = ros::Rate(30);

  // create ocTree with resolution .1
  octomap::OcTree ocTree = octomap::OcTree(.1);

  // the origin of the sensor 
  octomap::point3d sensorOrigin = octomap::point3d(0, 0, 0);

  // Current oriantation and position of the lidar
  float orientation = 0;
  float x = 0;
  float y = 0;
  float z = 0;

  // written by courtney
  // defining the minimumDist variable and minimumDistance publisher
  ros::Publisher minimumDistancePub;
  std_msgs::Float32 minDist;
  // end of code written by courtney

  public:

  WorldViewInterpreter(){

    // initialize laser scan subscriber
    laserSub = node.subscribe("/laser_publisher/laser_scan", 100, &WorldViewInterpreter::laserToPoint, this);

    // initialize subsriber to lidar orientation
    lidarOrientationSub = node.subscribe("/laser_publisher/lidar_orientation", 100, &WorldViewInterpreter::onRotation, this);

    // subscribe to drone's navigation data
    navDataSub = node.subscribe("/ground_truth/state", 100, &WorldViewInterpreter::onPosUpdate, this);

    // create point cloud publisher
    pointCloudPub = node.advertise<sensor_msgs::PointCloud>("/point_cloud", 1);
    minimumDistancePub = node.advertise<std_msgs::Float32>("/minimum_distance", 100);
  }

  /* @Brief: converts a lidar laser scan data to a set of 3D cartesian coordinates
     @Param: msg - the ROS Laser Scan message produced from the Lidar
     @Return: void
  */ 
  void laserToPoint(const sensor_msgs::LaserScan::ConstPtr& msg){

    // check if the Lidar is enabled
    bool is_reading_LiDAR;
    node.getParam("/is_reading_LiDAR", is_reading_LiDAR);

    if(is_reading_LiDAR == false)
	return;

    // get number of lasers
    int numLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    geometry_msgs::Point32 point;

    float distance;
    float distanceSum = 0;
    float horizontalAngle;
    float verticalAngle;

    octomap::Pointcloud ocPointCloud;

    // loop through each laser instance in the laser scan
    for(int i = 0; i < 64; i++){

      // calculate parameters of each individual laser beam
      distance = msg->ranges[i];
      distanceSum += distance;
      horizontalAngle = this->orientation;
      verticalAngle= (1.5708 - (msg->angle_min + (i * msg->angle_increment)) + 0.785398);

      // convert laser parameters to a point
      point.x = getX(distance, horizontalAngle, verticalAngle);
      point.y = getY(distance, horizontalAngle, verticalAngle);
      point.z = -getZ(distance, verticalAngle);

      // add point to ROS and Octomap point cloud
      this->pointCloud.push_back(point);
      
      ocPointCloud.push_back(point.x, point.y, point.z);
    }

    // written by courtney
    // set the minimum distance variable to the average ot the lidar point distances
    minDist.data = distanceSum / numLasers;
    minimumDistancePub.publish(minDist);
    loop_rate.sleep();
    minDist.data = 20;
    // end of code written by courtney

    // update ocTree with new pointcloud
    ocTree.insertPointCloud(ocPointCloud, this->sensorOrigin); 

    // if the save_octomap button is pressed, save the octomap to a file
    bool save_octomap;
    node.getParam("/save_octomap", save_octomap);

    if(save_octomap == true){
      ocTree.writeBinary("simple_tree.bt"); // save ocTree
      node.setParam("/save_octomap", false);
    }
    

    // populate ROS point cloud for visualization using rviz
    rosPointCloud.points.resize(pointCloud.size());
    std::copy(pointCloud.begin(), pointCloud.end(), std::back_inserter(rosPointCloud.points));

    rosPointCloud.header.frame_id = "my_frame";

    // publish ros point cloud
    pointCloudPub.publish(rosPointCloud);
    loop_rate.sleep();
  }

  /* @Brief: Retrives the current lidar orientation after the lidar rotates
     @Param: msg - the published lidar orientation parameter
     @Return: void
  */
  void onRotation(const std_msgs::Float32ConstPtr& msg){
    this->orientation = msg->data;
  }

  /* @Brief: Retrives the current position of the drone
     @Param: msg - the drone's published navigation data 
     @Return: void
  */
  void onPosUpdate(const nav_msgs::OdometryConstPtr& msg){
    this->x = msg->pose.pose.position.x;
    this->y = msg->pose.pose.position.y;
    this->z = msg->pose.pose.position.z;
  }

  /* @Brief: Calculates the x coordinate of a laser beam endpoint
     @Param: distance - the distance of the laser beam
     @Param: horizontalAngle - the horizontal angle of the laser beam
     @Param: verticalAngle - the vertical angle of the laser beam
     @Return: the x coordinate of the laser beam endpoint
  */
  float getX(float distance, float horizontalAngle, float verticalAngle){
    return distance * sin(verticalAngle) * cos(horizontalAngle) + this->x; 
  }

  /* @Brief: Calculates the y coordinate of a laser beam endpoint
     @Param: distance - the distance of the laser beam
     @Param: horizontalAngle - the horizontal angle of the laser beam
     @Param: verticalAngle - the vertical angle of the laser beam
     @Return: the y coordinate of the laser beam endpoint
  */
  float getY(float distance, float horizontalAngle, float verticalAngle){
    return distance * sin(verticalAngle) * sin(horizontalAngle) + this->y;
  }

  /* @Brief: Calculates the z coordinate of a laser beam endpoint
     @Param: distance - the distance of the laser beam
     @Param: verticalAngle - the vertical angle of the laser beam
     @Return: the z coordinate of the laser beam endpoint
  */
  float getZ(float distance, float verticalAngle){
    return -distance * cos(verticalAngle) + this->z;
  }
  
};

// main driver function
int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_listener");
  WorldViewInterpreter interpreter;
  ros::spin();
  return 0;
}
