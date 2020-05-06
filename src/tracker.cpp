
/*  @File: tracker.cpp

    @Brief: This class implements an OpenCV Tracker to allow the drone to follow and track an object.

    @Author: Matthew Page

    @Date: 11/6/2019

*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "geometry_msgs/Twist.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/features2d.hpp>

static bool initialized = false;

class Tracker
{

    // Ros node handle, publishers, and subscribers
    ros::NodeHandle node;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    ros::Publisher velPub;

    // Ros publish rate
    ros::Rate loop_rate = ros::Rate(30);

    // Initialize openCV tracker and blob detector
    cv::Rect2d roi;
    cv::Ptr<cv::Tracker> tracker = cv::TrackerKCF::create();
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create();

    public:

    Tracker()
      : it(node)
    {
        // Subscribe to input video feed and publish output video feed
        image_sub = it.subscribe("/ardrone/front/image_raw", 1, &Tracker::imageCb, this);

        image_pub = it.advertise("/tracker/output_video", 1);

        velPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    }

    /* @Brief: Finds and detects blobs in the drone's image feed
       @Param: image - the image from the drone's camera
       @Param: roi - the rectangular Region of Interest for tracking
       @Return: the area of the tracked blob
    */
    float findBlobs(cv::Mat image, cv::Rect2d roi){
        // convert to grayscale
        cv::cvtColor(image, image, CV_BGR2GRAY);
    
        // Detect blobs using detector
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(image, keypoints);

        // Draw detected blobs as red circles.
        cv::Mat im_with_keypoints;
        cv::drawKeypoints(image, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

        float radius = 0;

        // iterate through each blob detected and sort by 
        for(int i = 0; i < keypoints.size(); i++){

            // if the center of a blob is close to the center of the selection rectangle, highlight it in green
            if(sqrt(pow(keypoints[i].pt.x - (roi.x + ((float)roi.width / 2.0)), 2) 
                  + pow(keypoints[i].pt.y - (roi.y + ((float)roi.height / 2.0)), 2)) < 50){

                // save the radius of the blob
                radius = keypoints[i].size / 2.0;

                // Draw green circle around target object
                cv::circle(im_with_keypoints, cv::Point(keypoints[i].pt.x, keypoints[i].pt.y), radius, cv::Scalar(0,255,0), 2.0); 
            }
        }

        cv::imshow("blobs", im_with_keypoints);

        //return the area of the blob closest to region of interest
        return 3.1415 * (pow(radius / 2.0, 2));
    }

    /* @Brief: Updates the drone's position and orientation to keep the tracked object in the image frame
       @Param: image - the image from the drone's camera
       @Param: rect - the rectangular Region of Interest for tracking
       @Param: area - the area of the blob being tracked
       @Return: void
    */
    void updatePose(cv::Mat image, cv::Rect2d rect, float area){
        int center_x = rect.x + (rect.width / 2);
        int image_center_x = image.size().width / 2;

        geometry_msgs::Twist msg;

        // update orientation
        if(center_x > image_center_x - 10 and center_x < image_center_x + 10){
            msg.angular.z = 0.0;
        }

        else if(center_x > image_center_x){
            msg.angular.z = -0.4;
        }

        else if(center_x < image_center_x){
            msg.angular.z = 0.4;
        }

        // update position
        if(area < 500){
            msg.linear.x = 0.4;    
        }
        else{
            msg.linear.x = 0;
        }

        // publish the generated velocity command
        velPub.publish(msg);
        loop_rate.sleep();
    }

    /* @Brief: Converts the ROS image message to an openCV image and performs tracking/position updates
       @Param: msg - the ROS image from the drone's front camera
       @Return: void
    */
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        // convert the ROS image message to openCV image
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

        // check if tracking is enabled
        bool start_tracking;
        node.getParam("/start_tracking", start_tracking);

        if(start_tracking == false){
            return;
        }

        // initialize tracker once tracking is enabled
        if(initialized == false && start_tracking == true){
            //cap >> frame;
            roi = selectROI("tracker", cv_ptr->image);

            tracker->init(cv_ptr->image, roi);

            initialized = true;
        }

        // update the tracking result
        tracker->update(cv_ptr->image, roi);

        //perform blob detection
        float area = findBlobs(cv_ptr->image, roi);

        // draw tracked object
        cv::rectangle(cv_ptr->image, roi, cv::Scalar( 255, 0, 0 ), 2, 1 );

        // Check if autonomous mode is enabled before updating position
        bool autonomous_movement;
        node.getParam("/autonomous_movement", autonomous_movement);

        if(autonomous_movement == true)
	    updatePose(cv_ptr->image, roi, area);

        cv::waitKey(3);

        // Output modified video stream
        image_pub.publish(cv_ptr->toImageMsg());
    }
};

// main driver function
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tracker");
  Tracker tracker;
  ros::spin();
  return 0;
}
