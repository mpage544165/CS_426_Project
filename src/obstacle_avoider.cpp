
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

class ObstacleAvoider{

  public:

  ObstacleAvoider() {

    laserSub = node.Subscribe("/laser_publisher/laser_scan", 1, &ObstacleAvoider::calcAvgDistance, this);

    //lidarOrientationSub = node.Subscribe("/laser_publisher/laser_scan", 1, &ObstacleAvoider::, this);

    lidarOrientationSub = message_filters::Subscriber<Float32>(node, "/laser_publisher/lidar_orientation", 1);
    velSub = message_filters::Subscriber<Twist>(node, "/cmd_vel", 1);
    TimeSynchronizer<Float32, Twist> sync(lidarOrientationSub, velSub, 10);
    sync.registerCallback(boost::bind(&reorientLidar, _1, _2));

  }

  reorientLidar(const TwistConstPtr& vel, const Float32ConstPtr& orientation){
    
  }

  float calcAvgDistance(const sensor_msgs::LaserScan::ConstPtr& laserMsg){
    
    float sumOfDistances = 0;

    for(int i = 0; i < 64; i++){
      sumOfDistances += laserMsg->ranges[i];
    }
    return sumOfDistances / 64.0;
  }

  private:

  ros::NodeHandle node;
  ros::Subscriber laserSub;
  message_filters::Subscriber<Twist> velSub;
  message_filters::Subscriber<Float32> lidarOrientationSub;

  ros::Publisher velPub;
  ros::Publisher lidarOrientationPub;

  float averageDistance;

};

int main(int argc, char** argv){

  ros::init(argc, argv, "obstacle_avoider");
  ObstacleAvoider obstacleAvoider;
  ros::spin();
  return 0;
}
