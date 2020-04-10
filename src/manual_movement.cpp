// This class handles everything to do with manual movement and keyboard input
// Written by Courtney Palmer
// Started 2/15/20

/*
Note: to test this file, run the following: 
  Terminal 1:
    source /opt/ros/kinetic/setup.bash && source devel/setup.bash
    catkin_make && roscore

  Terminal 2:
      source /opt/ros/kinetic/setup.bash && source devel/setup.bash && roslaunch cvg_sim_gazebo ardrone_testworld.launch
  
  Terminal 3:
      source /opt/ros/kinetic/setup.bash && source devel/setup.bash && rosrun ardrone_autonomy manual_node



  Notes: Physical Drone Flight and Testing

    roslaunch drone_application launch_drone.launch

    rosrun image_view image_view image:=/ardrone/image_raw

    rostopic pub -1 ardrone/takeoff std_msgs/Empty
    rostopic pub -1 ardrone/land std_msgs/Empty

    source /opt/ros/kinetic/setup.bash && source devel/setup.bash && rostopic pub -1 ardrone/takeoff std_msgs/Empty



    roslaunch pid servo_sim.launch

    rosrun pid autotune

    rosrun rqt_gui rqt_gui


*/

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include "std_msgs/Empty.h"

class ManualMovement{

  ros::NodeHandle nodeHandler;
  ros::Subscriber movementSub;

  ros::Publisher velocityPublisher;
  ros::Publisher liftoffPublisher;
  ros::Publisher landingPublisher;

  struct termios cooked, raw;

  public:
   
  /**
    * @desc Initializes a ManualMovement Object.
    * @return void
  */
    ManualMovement(){
      // allows access to 'listen to' the cmd_vel node, which tracks the current Linear and Angular Velocity
      movementSub = nodeHandler.subscribe("cmd_vel", 100, &ManualMovement::inputChecker, this);
      // allows the ability to edit the cmd_vel node
      velocityPublisher = nodeHandler.advertise<geometry_msgs::Twist>("cmd_vel", 1);
      // allows the ability to edit the std_msgs node
      liftoffPublisher = nodeHandler.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
      //alows ability to edit the std_msgs node
      landingPublisher = nodeHandler.advertise<std_msgs::Empty>("/ardrone/land", 1);

    }

  /**
    * @desc This function verifies the current linear velocity of the drone
    * @param geometry_msgs::Twist::ConstPtr& msg
    * @return void
  */
  void inputChecker(const geometry_msgs::Twist::ConstPtr& msg){ 
    double xVal = msg->linear.x;
    double yVal = msg->linear.y;
    double zVal = msg->linear.z;
    
    ROS_INFO("x: %f,  y: %f, z: %f", xVal, yVal, zVal);
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
      msg.linear.x = Lx;
      msg.linear.y = Ly;
      msg.linear.z = Lz;
      msg.angular.x = Ax;
      msg.angular.y = Ay;
      msg.angular.z = Az;

      // TODO
        // set a range of velocity speeds between a minimum and ma

      velocityPublisher.publish(msg);
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
      }
      else{ //we are enacting a Landing Procedure
        landingPublisher.publish(msg);
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
    void Keyboard(){
      char key;
      geometry_msgs::Twist msg;
      std_msgs::Empty Lmsg;

      if(PrepTerminal()){ // if terminal is set in raw mode
        std::cout << "Ready to Recieve Input..." << std::endl;
        std::cout << "Press 'h' for help" << std::endl;
      }

      else{ // terminal was unable to be set for raw mode
        perror("Unable to prepare the terminal for input");
        exit(-1);
      }

      while(true){
        if(read(0, &key, 1) < 0){ // take next incoming input
          perror("Unable to recieve input");
          exit(-1);
        }

      // move drone based upon the key recorded
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
          move(msg, 10, 0, 0, 0, 0, 0);
          break;
        case 's': //backward
          move(msg, -10, 0, 0, 0, 0, 0);
          break;
        case 'a': //left
          move(msg, 0, 10, 0, 0, 0, 0);
          break;
        case 'd': //right
          move(msg, 0, -10, 0, 0, 0, 0);
          break;
        case 'r': //elevate
          move(msg, 0, 0, 10, 0, 0, 0);
          break;
        case 'f': //decend
          move(msg, 0, 0, -10, 0, 0, 0);
          break;
        case 'q': //rotate left
          move(msg, 0, 0, 0, 0, 0, 10);
          break;
        case 'e': // rotate right:
          move(msg, 0, 0, 0, 0, 0, -10);
          break;
        case 'x': //stop movement
          move(msg, 0, 0, 0, 0, 0, 0);
          break;
      }

      // if(!verifySafePosition()){
        // EmergencyHalt();
      // }

    }
    return;
  }

  /**
    * @desc Compares current position compared to the nearest physical object. If object is 
    *     within a certain distance, the drone automatically halts all movement
    * @return bool - success or failure
  */
  bool VerifySafePosition(){
    //TODO
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
  manmove.Keyboard();
  ros::spin();
  return 0;
}
