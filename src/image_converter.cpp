#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "geometry_msgs/Twist.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/tracking.hpp>

static const std::string OPENCV_WINDOW = "Image window";

static bool initialized = false;
static bool calibrated = false;

//known measurements in inches
static int const KNOWN_DISTANCE = 157;
static int const KNOWN_WIDTH = 39;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher velPub_;

  ros::Rate loop_rate = ros::Rate(30);

  

  cv::Rect2d roi;
  cv::Ptr<cv::Tracker> tracker = cv::TrackerKCF::create();

  cv::RotatedRect rect;
  int focalLength; 

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/front/image_raw", 1,
      &ImageConverter::imageCb, this);

    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    velPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    cv::namedWindow(OPENCV_WINDOW);

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  cv::RotatedRect findMarker(cv::Mat image){
    cv::Mat gray;
    cv::cvtColor(image, gray, CV_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    cv::Mat edges;
    cv::Canny(gray, edges, 35, 125);

    cv::imshow("edges", edges);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(edges, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

    int maxArea = cv::contourArea(contours[0]);
    int index = 0;

    for(int i = 1; i < contours.size(); i++){
        if(cv::contourArea(contours[i]) > maxArea){
            maxArea = cv::contourArea(contours[i]);
            index = i;
        }
    }

    return cv::minAreaRect(contours[index]);
  }

  int distanceToCamera(int knownWidth, int focalLength, int perWidth){
    try{
        return (knownWidth * focalLength) / perWidth;
    }
    catch(int e){
        return 0;
    }
  }

  void updateOrientation(cv::Mat image, cv::Rect2d rect, int distance){
    int center_x = rect.x + (rect.width / 2);
    int image_center_x = image.size().width / 2;

    geometry_msgs::Twist msg;

    if(center_x > image_center_x - 10 and center_x < image_center_x + 10){
        msg.angular.z = 0.0;
    }

    else if(center_x > image_center_x){
        msg.angular.z = -0.5;
    }

    else if(center_x < image_center_x){
        msg.angular.z = 0.5;
    }

    if(distance > 250){
        msg.linear.x = 0.5;    
    }
    else{
        msg.linear.x = 0;
    }

    velPub_.publish(msg);
    loop_rate.sleep();
  }

  void updatePosition(int distance){

    geometry_msgs::Twist msg;

    if(distance > 200){
        msg.linear.x = 1.0;    
    }
    else{
        msg.linear.x = 0;
    }

    velPub_.publish(msg);
    loop_rate.sleep();
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if(!initialized){
        //cap >> frame;
        roi = selectROI("tracker", cv_ptr->image);

        tracker->init(cv_ptr->image, roi);

        initialized = true;
    }

    if(!calibrated){
        //save reference image
        //cv::imwrite("ref_image.png", cv_ptr->image);
        

        // calibrate camera
        cv::Mat im = cv::imread("ref_image.png");
        rect = findMarker(im);
        focalLength = (rect.size.width * KNOWN_DISTANCE) / KNOWN_WIDTH;
        std::cout << "focalLength: " << focalLength;

        calibrated = true;
    }

    rect = findMarker(cv_ptr->image);
    int inches = distanceToCamera(KNOWN_WIDTH, focalLength, rect.size.width);
 
    // display distance
    std::stringstream ss;
    ss << inches;
    putText(cv_ptr->image, ss.str() + " inches", cv::Point(20, 60), 
               cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 0));

    cv::rectangle(cv_ptr->image, rect.boundingRect(), cv::Scalar( 0, 255, 0 ), 2, 1 );

    //updatePosition(inches);

    cv::imshow("image", cv_ptr->image);

    // update the tracking result
    tracker->update(cv_ptr->image, roi);

    // draw the tracked object
    cv::rectangle(cv_ptr->image, roi, cv::Scalar( 255, 0, 0 ), 2, 1 );

    updateOrientation(cv_ptr->image, roi, inches);

    

    // show image with the tracked object
    //imshow("tracker", cv_ptr->image);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
