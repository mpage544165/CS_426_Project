// This class handles everything to do with manual movement and keyboard input
// Written by Courtney Palmer
// Started 2/15/20

/*
  Note: to test this file, run the following: 
    Terminal 1: Setting up the ROS environment
      cd catkin_ws/catkin_ws_AR
      source /opt/ros/kinetic/setup.bash && source devel/setup.bash && catkin_make && roscore

    Terminal 2: Opening up Gazebo
        source /opt/ros/kinetic/setup.bash && source devel/setup.bash && roslaunch cvg_sim_gazebo ardrone_box.launch
    
    Terminal 3: Running the autonomy node
        source /opt/ros/kinetic/setup.bash && source devel/setup.bash && rosrun ardrone_autonomy manual_node

*/

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include "std_msgs/Empty.h"
#include <nav_msgs/Odometry.h>
#include "fcntl.h"
#include <math.h>
#include "std_msgs/Float32.h"

class ManualMovement{

public:
  ros::NodeHandle nodeHandler;
  ros::Subscriber movementSub;

  ros::Publisher velocityPublisher;
  ros::Publisher liftoffPublisher;
  ros::Publisher landingPublisher;

  ros::Subscriber currentPosSub;
  ros::Subscriber minimumDistanceSub;

  struct termios cooked, raw;

  //These 3 values represent the current position of the drone
  float xPos = 0.0f;
  float yPos = 0.0f;
  float zPos = 0.0f;
  float lastXpos = 0.0f;
  float lastYpos = 0.0f;
  float lastZpos = 0.0f;
  float Xdir = 0.0f;
  float Ydir = 0.0f;
  float Zdir = 0.0f;

  float currentmin;

  ros::Rate loop_rate = ros::Rate(30);
  
  /**
    * @desc Initializes a ManualMovement Object.
    * @note ManualMovement Object has the ability to get drone position, launch a drone, 
    *   land a drone, and move a drone
    * @return void
  */
    ManualMovement(){
      // allows the ability to edit the cmd_vel node and change drone velocity
      velocityPublisher = nodeHandler.advertise<geometry_msgs::Twist>("cmd_vel", 1);
      // allows the ability to edit the std_msgs node and launch the drone
      liftoffPublisher = nodeHandler.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
      //alows ability to edit the std_msgs node and  land the drone
      landingPublisher = nodeHandler.advertise<std_msgs::Empty>("/ardrone/land", 1);
      // topic used to get drone's current position
      currentPosSub = nodeHandler.subscribe("/ground_truth/state", 100, &ManualMovement::posChecker, this);
      minimumDistanceSub = nodeHandler.subscribe("/minimum_distance", 100, &ManualMovement::getMin, this);
    }

  void getMin(const std_msgs::Float32& msg){
    currentmin = msg.data;
  }

  /**
    * @desc moves the drone's position in the environment. It starts by checking the drone is in a safe position, then 
    *   reading in keyboard input to change drone velocity
    * @param const nav_msgs::OdometryConstPtr& msg - represennts the appropriate message type that is used to access the
    *     drone's current positino
    * @return void
  */
  void posChecker(const nav_msgs::OdometryConstPtr& msg){
    //declare variables, set the current drone position
    geometry_msgs::Twist Tmsg;
    std_msgs::Empty Lmsg;
    xPos = msg->pose.pose.position.x;
    yPos = msg->pose.pose.position.y;
    zPos = msg->pose.pose.position.z;
    Xdir = (xPos - lastXpos) * 1;
    Ydir = (yPos - lastYpos) * 1;
    Zdir = (zPos - lastZpos) * 1;
    lastXpos = xPos;
    lastYpos = yPos;
    lastZpos = zPos;

    float directionInRads = atan2(Ydir, Xdir);
    float directionInDegrees = (directionInRads * 180) / 3.1415;

    // std::cout << "Dir VALUES-- X: " << Xdir << " Y: " << Ydir << " Z: " << Zdir << std::endl;
   // std::cout << "Degree is: " << directionInDegrees << std::endl;

    // using current drone position, verify that drone is still within its allowed flyable boundry
    int currentDroneState = VerifySafePosition(xPos, yPos, zPos);
    if(currentmin < 0.1){
      currentDroneState = 1;
    }
    if(currentDroneState != 0){ // if VerifySafePosition returns a value, this means the drone has crossed a boundry!
      // drone is in an unsafe position! Halt all current movement and call ReverseFromBoundry
      move(Tmsg, 0, 0, 0, 0, 0, 0);
      ReverseFromBoundry(currentDroneState, Tmsg);
    }

    //otherwise, the drone is in a safe position and we are free to search for key presses
    else{
      if(PrepTerminal()){ // if terminal is set in raw mode
        std::cout << "Ready to Recieve Input..." << std::endl;
        std::cout << "Press 'h' for help" << std::endl;
      }

      else{ // terminal was unable to be set for raw mode
        perror("Unable to prepare the terminal for input");
        exit(-1);
      }
      char key; // hold the current key press
      int flags = fcntl(0, F_GETFL, 0); /* get current file status flags */
      flags |= O_NONBLOCK;          /* turn off blocking flag */
      fcntl(0, F_SETFL, flags);         /* set up non-blocking read */
      if(read(0, &key, 1) < 0){ // take next incoming input

      }
      //if a key was successfully recorded, call Keyboard()
      else{
        Keyboard(key, xPos, yPos, zPos, Tmsg, Lmsg);
      }
    }
  }
  /*
  get drones x and z positions to get the horizontal direction of the drone and get velocity from there
    get that from the nav data or ground truth topic
  get the orientation of the lidar
    have dot product between angle of lidar and direction of drone to be zero 
    that way lidar is directed where drone is flying
  
  */

//

  /**
    * @desc sets the drone's linear and angular velocity and publishes it
    * @param geometry_msgs::Twist msg - represennts the appropriate message type that will be published 
    *       to the node 'cmd_vel'
    * @param int Lx, Ly, Lz - represents the Linear x, y, and z velocity
    * @param int Ax, Ay, Az - represents the Angular x, y, and z velocity
    * @return void
  */
    void move(geometry_msgs::Twist msg, int Lx, int Ly, int Lz, int Ax, int Ay, int Az){
      
      msg.linear.x = Lx;
      msg.linear.y = Ly;
      msg.linear.z = Lz;
      msg.angular.x = Ax;
      msg.angular.y = Ay;
      msg.angular.z = Az;

      // TODO
        // set a range of velocity speeds between a minimum and ma

      velocityPublisher.publish(msg);
      loop_rate.sleep();
    }

  /**
    * @desc controls both the initial take off and eventual landing procedures
    * @param bool isLift - toggles whether or not this is a takeoff or landing procedure
    * @param std_msgs::Empty msg - passed in message that is published to the takeoff node
    * @return void
  */
    void LiftAndLandProcedure(bool isLift, std_msgs::Empty msg){
      if(isLift){ //we are enacting a liftoff proceddure
        liftoffPublisher.publish(msg);
        loop_rate.sleep();
      }
      else{ //we are enacting a Landing Procedure
        landingPublisher.publish(msg);
        loop_rate.sleep();
      }
    }

    /**
      * @desc sets up the terminal to accept keyboard input in raw mode
      * @return bool - success or failure
    */
    bool PrepTerminal(){

      //NOTE: the following lines of code was referenced within the ROS tutorial sample: 
        // http://docs.ros.org/lunar/api/turtlesim/html/teleop__turtle__key_8cpp_source.html

      // first, we want our terminal in 'raw mode', meaning we want any data passed in as-is
          // without interpreting special characters                                                           
      tcgetattr(0, &cooked); 
      memcpy(&raw, &cooked, sizeof(struct termios));
      // here, we disable the ICANON flag so that no input processing is performed (allows for immediate input)
          // we disable the ECHO flag so that any input is not repeated in the terminal
      raw.c_lflag &=~ (ICANON | ECHO); //disabling icanon sets disables canonical mode

      // Setting a new line, then end of file                         
      raw.c_cc[VEOL] = 1;
      raw.c_cc[VEOF] = 2;
      tcsetattr(0, TCSANOW, &raw); // set new terminal settings
      return true;
    }

    /**
      * @desc opens a modal window to display a message
      * @param string $msg - the message to be displayed
      * @return bool - success or failure
    */
    void Keyboard(char key, float xpos, float ypos, float zpos, geometry_msgs::Twist msg, std_msgs::Empty Lmsg){
      switch(key){
        case 'h': // Help
          std::cout << " ==== Control Scheme ==== " << std::endl;
          std::cout << " |l| - Liftoff " << std::endl;
          std::cout << " |k| - Land " << std::endl;
          std::cout << " |w| - Move Forward " << std::endl;
          std::cout << " |s| - Move Backward " << std::endl;
          std::cout << " |a| - Move Left " << std::endl;
          std::cout << " |d| - Move Right " << std::endl;
          std::cout << " |r| - Increase Height " << std::endl;
          std::cout << " |f| - Decrease Height " << std::endl;
          std::cout << " |q| - Spin Left " << std::endl;
          std::cout << " |e| - Spin Right " << std::endl;
          std::cout << " |x| - Halt Movement " << std::endl << std::endl;

          std::cout << "Ready to Recieve Input..." << std::endl;
          std::cout << "Press 'h' for help" << std::endl;
          break;

        case 'l': // lisftoff
          LiftAndLandProcedure(true, Lmsg); //set liftoff command
          break;
        case 'k': // land
          LiftAndLandProcedure(false, Lmsg); //set land command
          break;
        case 'w': //forward
          move(msg, 7, 0, 0, 0, 0, 0);
          break;
        case 's': //backward
          move(msg, -7, 0, 0, 0, 0, 0);
          break;
        case 'a': //left
          move(msg, 0, 7, 0, 0, 0, 0);
          break;
        case 'd': //right
          move(msg, 0, -7, 0, 0, 0, 0);
          break;
        case 'r': //elevate
          move(msg, 0, 0, 7, 0, 0, 0);
          break;
        case 'f': //decend
          move(msg, 0, 0, -7, 0, 0, 0);
          break;
        case 'q': //rotate left
          move(msg, 0, 0, 0, 0, 0, 7);
          break;
        case 'e': // rotate right:
          move(msg, 0, 0, 0, 0, 0, -7);
          break;
        case 'x': //stop movement
          move(msg, 0, 0, 0, 0, 0, 0);
          break;
      }
    return;
  }

    void ReverseFromBoundry(int currentDroneCase, geometry_msgs::Twist msg){
      move(msg, 0, -5, 0, 0, 0, 0); // ensure the drone is initially stationary
      if (currentDroneCase == 1){
        std::cout << "[!WARNING!] X-Facing Boundry Encountered! Moving BACKWARDS for 1 second..." << std::endl;
        move(msg,-5, 0, 0, 0, 0, 0);
        ros::Duration(1).sleep();
      }
      if (currentDroneCase == 2){
        std::cout << "[!WARNING!] NEG X-Facing Boundry Encountered! Moving FORWARDS for 1 second..." << std::endl;
        move(msg, 5, 0, 0, 0, 0, 0);
        ros::Duration(1).sleep();
      }
      if (currentDroneCase == 3){
        std::cout << "[!WARNING!] Y-Facing Boundry Encountered! Moving RIGHT for 1 second..." << std::endl;
        move(msg, 0, -5, 0, 0, 0, 0);
        ros::Duration(1).sleep();
      }
      if (currentDroneCase == 4){
        std::cout << "[!WARNING!] NEG Y-Facing Boundry Encountered! Moving RIGHT for 1 second..." << std::endl;
        move(msg, 0, 5, 0, 0, 0, 0);
        ros::Duration(1).sleep();
      }
      if (currentDroneCase == 5){
        std::cout << "[!WARNING!] Z-Facing Boundry Encountered! Moving DOWNWARDS for 1 second..." << std::endl;
        move(msg, 0, 0, -5, 0, 0, 0);
        ros::Duration(1).sleep();
      }
      if (currentDroneCase == 6){
        std::cout << "[!WARNING!] X-Facing Object Encountered! Moving BACKWARDS for 0.5 seconds..." << std::endl;
        move(msg, 0, -5, 0, 0, 0, 0);
        ros::Duration(1).sleep();
      }
      std::cout << "Manuver Completed. Awaiting next Command." << std::endl;

      move(msg, 0, 0, 0, 0, 0, 0); // one second has elapsed, halt the drone
      
    }

  /**
    * @desc Compares current position compared to the nearest physical object. If object is 
    *     within a certain distance, the drone automatically halts all movement
    * @return bool - success or failure
  */
  int VerifySafePosition(float xP, float yP, float zP){
   // ROS_INFO("x: %f", xPos);
    if(xP > 10){ // drone is too far forward
     
      return 1;
    }
    
    else if (xP < -10){ // drone is too far backwards
      return 2;
    }
    
    else if(yP > 10){ // drone is too far to the left
      return 3;
    } 
    
    else if (yP < -10){ // drone is too far to the right
      return 4;
    }

    else{
      return 0;
    }
  }

  /**
    * @desc Procedure to halt all movement
    * @return void
  */
  bool EmergencyHalt(){
    //TODO
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "manual_move");
  ManualMovement manmove;
  ros::spin();
  return 0;
}
