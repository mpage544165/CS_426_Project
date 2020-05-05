// The purpose of this class is to handle keyboard input, drone movement, and safety checking features. 
// Written by Courtney Palmer
// Started 2/15/20

/*
  Note: to test this file, run the following: 
    Terminal 1: Setting up the ROS environment
      cd catkin_ws/catkin_ws_AR
      source /opt/ros/kinetic/setup.bash && source devel/setup.bash && catkin_make && roscore

    Terminal 2: Running Manual Movement alone
        source /opt/ros/kinetic/setup.bash && source devel/setup.bash && roslaunch roslaunch manual_node
    
   Terminal 2: ALTERNATIVE: Run the complete project with a launch file
      	source /opt/ros/kinetic/setup.bash && source devel/setup.bash && roslaunch cvg_sim_gazebo ardrone_cave.launch 
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
  //this is the last recorded positin of the drone (recorded from the last loop)
  float lastXpos = 0.0f;
  float lastYpos = 0.0f;
  float lastZpos = 0.0f;
  //This is the drone's x y z direction
  //this is calculated by taking the difference of the last xyz position and the current xyz position
  float Xdir = 0.0f;
  float Ydir = 0.0f;
  float Zdir = 0.0f;

  //this represents the boundry lines that a drone is not allowed to cross
  float xNegBound = -10;
  float xPosBound = 10;
  float yPosBound = 10;
  float yNegBound = -10;

  //recorded minimum distance from LIDAR data
  float currentmin;

  //records whether drone is flying or not
  //this is used to ignore commands when drone is landed
  bool isFlying = false;

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
      //this topic is used to get the minimum distance from the minimum_distance ROS topic
      minimumDistanceSub = nodeHandler.subscribe("/minimum_distance", 100, &ManualMovement::getMin, this);
    }

  /**
    * @desc records the minimum distance from the given LIDAR data
    * @param std_msgs::Float32 msg: this is the message type used to locate relavent data
    *     from the minimum_distance ROS topic
    * @return void
  */
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
    
    // NOTE uncomment out the following function if identifying the drone's direction is necessary
    //getCurrentDirection();


    // using current drone position, verify that drone is still within its allowed flyable boundry
    // currentDroneState will record if the drone has crossed a specific boundry (if any)
    int currentDroneState = VerifySafePosition(xPos, yPos, zPos);

    // next, we check if the distance calculated from LIDAR data is smaller than 0.9 units
    // if so, then we are too close to an object and must back away
    if((currentmin < 0.9) && isFlying){
     currentDroneState = 6;    
    } 

    // if the current drone state has been set to an int value, then the drone is in an unsafe position! Either it
        // has crossed a boundry or it has gone too close to an object. Either way, we will halt all current movement
        // then call Reverse from boundry to 'rescue it'
    if(currentDroneState != 0){ 
      move(Tmsg, 0, 0, 0, 0, 0, 0);
      ReverseFromBoundry(currentDroneState, Tmsg);
      currentDroneState = 0;
    }
    // Otherwise, the drone is in a safe position and we are free to search for key presses
    else{
      if(PrepTerminal()){ // check that terminal is set in raw mode
        std::cout << "Ready to Recieve Input..." << std::endl;
        std::cout << "Press 'h' for help" << std::endl;
      }

      else{ // otherwise, something went wrong with terminal settings and we must exit
        perror("Unable to prepare the terminal for input");
        exit(-1);
      }
      char key; // hold the current key press
      
      // Normally, the command ' read(0, &key, 1) ' will force our function to halt here until keyboard input
          // is detected. This was an issue because our callback function 'posChecker' must be continously looped 
          // the following three lines disables the halting functionality so that 'posChecker' can loop
      int flags = fcntl(0, F_GETFL, 0); /* get current file status flags */
      flags |= O_NONBLOCK;          /* turn off blocking flag */
      fcntl(0, F_SETFL, flags);         /* set up non-blocking read */
      
      // check if keyboard input was detected or not
      if(read(0, &key, 1) < 0){ 

      }
      //if a key was successfully recorded, call Keyboard()
      else{
        Keyboard(key, Tmsg, Lmsg);
      }
    }
    currentDroneState = 0;
  }

  /**
    * @desc calculated the drone's current direction by taking the difference between the current xyz pos and the last
    *     recorded xyz pos. 
    * @return void
  */
  float getCurrentDirection(){
    Xdir = (xPos - lastXpos);
    Ydir = (yPos - lastYpos);
    Zdir = (zPos - lastZpos);
    // std::cout << "Direction Values X: " << Xdir << " Y: " << Ydir << " Z: " << Zdir << std::endl;
    lastXpos = xPos;
    lastYpos = yPos;
    lastZpos = zPos;

    float directionInRads = atan2(Ydir, Xdir);
    float directionInDegrees = (directionInRads * 180) / 3.1415;
    // std::cout << "Degree is: " << directionInDegrees << std::endl;

    return directionInDegrees;

  }

  /**
    * @desc sets the drone's linear and angular velocity and publishes it
    * @param geometry_msgs::Twist msg - represennts the appropriate message type that will be published 
    *       to the node 'cmd_vel'
    * @param int Lx, Ly, Lz - represents the Linear x, y, and z velocity
    * @param int Ax, Ay, Az - represents the Angular x, y, and z velocity
    * @return void
  */
    void move(geometry_msgs::Twist msg, int Lx, int Ly, int Lz, int Ax, int Ay, int Az){
      
      //set linear and angular velocity
      msg.linear.x = Lx;
      msg.linear.y = Ly;
      msg.linear.z = Lz;
      msg.angular.x = Ax;
      msg.angular.y = Ay;
      msg.angular.z = Az;

      //publish new value and loop to ensure that the change was made
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
      * @desc this function interprets a given keyboard input and calls the appropriate function
      * @param char key: represents the current recorded key press
      * @param geometry::twist msg: this message will carry a movement-type command
      * @param std_msgs::Empty Lmsg: this message will carry a Lift/Land type command
      * @return void
    */
    void Keyboard(char key, geometry_msgs::Twist msg, std_msgs::Empty Lmsg){
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
          isFlying = true;
          LiftAndLandProcedure(true, Lmsg); //set liftoff command
          break;
        case 'k': // land
          isFlying = false;
          LiftAndLandProcedure(false, Lmsg); //set land command
          break;
        case 'w': //forward
          move(msg, 1, 0, 0, 0, 0, 0);
          break;
        case 's': //backward
          move(msg, -3, 0, 0, 0, 0, 0);
          break;
        case 'a': //left
          move(msg, 0, 3, 0, 0, 0, 0);
          break;
        case 'd': //right
          move(msg, 0, -3, 0, 0, 0, 0);
          break;
        case 'r': //elevate
          move(msg, 0, 0, 3, 0, 0, 0);
          break;
        case 'f': //decend
          move(msg, 0, 0, -3, 0, 0, 0);
          break;
        case 'q': //rotate left
          move(msg, 0, 0, 0, 0, 0, 2);
          break;
        case 'e': // rotate right:
          move(msg, 0, 0, 0, 0, 0, -2);
          break;
        case 'x': //stop movement
          move(msg, 0, 0, 0, 0, 0, 0);
          break;
      }
    return;
  }

    /**
      * @desc this function will interpret the drones flight 'case' and react by moving the drone
      *     away from a danger if a danger was encountered
      * @param int currentDroneCase: This signifies which state the drone is currently in
      * @param geometry::twist msg: this message will carry a movement-type command
      * @return void
    */
    void ReverseFromBoundry(int currentDroneCase, geometry_msgs::Twist msg){
      // first we ensure that the drone is initially stationary
      move(msg, 0, 0, 0, 0, 0, 0); 
      // move(msg, 0, -5, 0, 0, 0, 0);

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
        std::cout << "[!WARNING!] NEG Y-Facing Boundry Encountered! Moving LEFT for 1 second..." << std::endl;
        move(msg, 0, 5, 0, 0, 0, 0);
        ros::Duration(1).sleep();
      }
      if (currentDroneCase == 5){
        std::cout << "[!WARNING!] Z-Facing Boundry Encountered! Moving DOWNWARDS for 1 second..." << std::endl;
        move(msg, 0, 0, -5, 0, 0, 0);
        ros::Duration(1).sleep();
      }
      if (currentDroneCase == 6){
        std::cout << "[!WARNING!] X-Facing Object Encountered! Moving BACKWARDS for 2 seconds..." << std::endl;
        move(msg, -3, 0, 0, 0, 0, 0);
        ros::Duration(2).sleep();
      }
      std::cout << "Manuver Completed. Awaiting next Command." << std::endl;

      // one second has elapsed, halt the drone
      move(msg, 0, 0, 0, 0, 0, 0); 
      currentDroneCase = 0;
      
    }

  /**
    * @desc Compares current position compared to the nearest physical object. If object is 
    *     within a certain distance, the drone automatically halts all movement
    * @param float xP, xP, zP represents the current xyz position of the drone
    * @return bool - success or failure
  */
  int VerifySafePosition(float xP, float yP, float zP){
   // ROS_INFO("x: %f", xPos);
    if(xP > xPosBound){ // drone is too far forward
     
      return 1;
    }
    
    else if (xP < xNegBound){ // drone is too far backwards
      return 2;
    }
    
    else if(yP > yPosBound){ // drone is too far to the left
      return 3;
    } 
    
    else if (yP < yNegBound){ // drone is too far to the right
      return 4;
    }

    else{
      return 0;
    }
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "manual_move");
  ManualMovement manmove;
  ros::spin();
  return 0;
}
