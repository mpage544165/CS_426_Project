#include <math.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Float32.h"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

class WorldViewInterpreter{

  std::vector<geometry_msgs::Point32> pointCloud;
  sensor_msgs::PointCloud rosPointCloud;

  ros::NodeHandle nodeHandle;
  ros::Subscriber laserSub;
  ros::Subscriber lidarOrientationSub;
  ros::Publisher pointCloudPub;
  ros::Rate loop_rate = ros::Rate(30);

  octomap::OcTree ocTree = octomap::OcTree(.1); // create ocTree with resolution .1
  octomap::point3d sensorOrigin = octomap::point3d(0, 0, 0); // the origin of the sensor
  
  float orientation = 0;

  public:

  WorldViewInterpreter(){

    // initialize laser scan subscriber
    laserSub = nodeHandle.subscribe("/laser_publisher/laser_scan", 100, &WorldViewInterpreter::laserToPoint, this);

    // initialize subsriber to lidar orientation
    lidarOrientationSub = nodeHandle.subscribe("/laser_publisher/lidar_orientation", 100, &WorldViewInterpreter::onRotation, this);

    // publish point cloud
    pointCloudPub = nodeHandle.advertise<sensor_msgs::PointCloud>("/point_cloud", 1);
  }

  // callback function for laser scan subscriber
  void laserToPoint(const sensor_msgs::LaserScan::ConstPtr& msg){
    
    bool is_reading_LiDAR;
    nodeHandle.getParam("/is_reading_LiDAR", is_reading_LiDAR);

    if(is_reading_LiDAR == false)
	      return;

    int numLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    std::cout << "numLasers: " << numLasers << std::endl;
    geometry_msgs::Point32 point;

    float distance;
    float horizontalAngle;
    float verticalAngle;

    octomap::Pointcloud ocPointCloud;

    // loop through each laser instance in the laser scan
    for(int i = 0; i < 64; i++){
      // calculate parameters of laser 
      distance = msg->ranges[i];
      horizontalAngle = this->orientation;
      verticalAngle= (1.5708 - (msg->angle_min + (i * msg->angle_increment)) + 0.785398);

      // convert laser parameters to point
      point.x = getX(distance, horizontalAngle, verticalAngle);
      point.y = getY(distance, horizontalAngle, verticalAngle);
      point.z = -getZ(distance, verticalAngle);

      this->pointCloud.push_back(point); // add point to point cloud
      
      ocPointCloud.push_back(point.x, point.y, point.z); // add
    }

    ocTree.insertPointCloud(ocPointCloud, this->sensorOrigin); // update ocTree
    
    bool save_octomap;
    nodeHandle.getParam("/save_octomap", save_octomap);

    if(save_octomap == true){
      ocTree.writeBinary("simple_tree.bt"); // save ocTree
      nodeHandle.setParam("/save_octomap", false);
    }
    
    //std::cout << "Done" << std::endl;

    // populate ros point cloud
    rosPointCloud.points.resize(pointCloud.size());
    std::copy(pointCloud.begin(), pointCloud.end(), std::back_inserter(rosPointCloud.points));

    rosPointCloud.header.frame_id = "my_frame";

    pointCloudPub.publish(rosPointCloud); // publish ros point cloud
    loop_rate.sleep();
  }

  // callback function for lidar rotation
  void onRotation(const std_msgs::Float32ConstPtr& msg){
    this->orientation = msg->data;
  }

  // get x coordinate
  float getX(float distance, float horizontalAngle, float verticalAngle){
    return distance * sin(verticalAngle) * cos(horizontalAngle); 
  }

  // get y coordinate
  float getY(float distance, float horizontalAngle, float verticalAngle){
    return distance * sin(verticalAngle) * sin(horizontalAngle);
  }

  // get z coordinate
  float getZ(float distance, float verticalAngle){
    return distance * cos(verticalAngle);
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_listener");
  WorldViewInterpreter interpreter;
  ros::spin();
  return 0;
}
