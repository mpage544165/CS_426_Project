
#include "ros/ros.h"
#include "geometry_msgs/Point32.h"

class Localization{

  public:

  Localization(){

    imageSub = message_filters::Subscriber<Image>(node, "/ardrone/front/image_raw", 1);

    imuSub = message_filters::Subscriber<Imu>(node, "/ardrone/imu", 1);

    TimeSynchronizer<Float32, Twist> sync(lidarOrientationSub, velSub, 10);
    sync.registerCallback(boost::bind(&reorientLidar, _1, _2));
  }
  
  void sensorFusion(){
    
  }

  private:

  ros::NodeHandle node;

  geometry_msgs::Point32 cameraPos;

  geometry_msgs::Point32 imuPos;

  message_filters::Subscriber<Image> imageSub;
  
  message_filters::Subscriber<Imu> imuSub;
  

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization");
  Localization localization;
  ros::spin();
  return 0;
}
