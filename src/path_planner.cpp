
/*  @File: path_planner.cpp

    @Brief: This class implements some helper functions for the path planning algorithms

    @Author: Matthew Page

    @Date: 12/30/2019

*/

#include <stack>
#include <queue>
#include <math.h>
#include <fstream>

#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// Pair representing x, y coords
typedef std::pair<int, int> Pair;

// Define A* node
struct Node
{
    Pair coords;
    int prev_x = -1;
    int prev_y = -1;

    float gCost = 1000000;
    float fCost = 1000000;
    float hCost = 1000000;

    Node* previousNode = NULL;
};

// comparison operator to make std::prioirty_queue a min priority queue
struct LessThanByCost
{
  bool operator()(const Node& lhs, const Node& rhs) const
  {
    return lhs.fCost > rhs.fCost;
  }
};

class PathPlanner{

  public:

  PathPlanner(octomap::OcTree* tree){

    map = tree;
  }

  // A star pathing algorithm
  // Param: the starting point of the drone
  // Param: the destination of the drone
  // Return: A vector of coordinates that denote the path
  std::vector<Pair> A_star(octomap::point3d start, octomap::point3d end){

    // Was to be completed by Gary Lougheed
  }

  /* @Brief: converts a lidar laser scan data to a set of 3D cartesian coordinates
     @Param: msg - the ROS Laser Scan message produced from the Lidar
     @Return: void
  */ 
  bool isValid(Node node){

    if(gridMap[node.coords.first][node.coords.second] > 0)
      return false;

    return true;
  }

  /* @Brief: Calculate the manhattan distance between two coordinates
     @Param: start_x - starting x coordinate
     @Param: start_y - starting y coordinate
     @Param: end_x - ending x coordinate
     @Param: end_y - ending y coordinate
     @Return: the heuristic value
  */ 
  float calculate_H(float start_x, float start_y, float end_x, float end_y){
    return (std::abs(end_x - start_x) +
           std::abs(end_y - start_y));
  }

  /* @Brief: Helper funcion to find all neighbors of a node
     @Param: x - the x coordinate of the node
     @Param: y - the y coordinate of the node
     @Return: List of neighbor nodes
  */ 
  std::vector<Node> getNeighbors(int x, int y){

    std::vector<Node> neighbors;
    Node node;

    for(int colPosition = x - 1; colPosition <= x + 1; colPosition++){
      for(int rowPosition = y - 1; rowPosition <= y + 1; rowPosition++){

        if(!( (colPosition == x) && (rowPosition == y) )){

          if(! ((colPosition < 0) && (rowPosition < 0) &&
                (colPosition > gridMap.size()) && (rowPosition > gridMap[0].size())) ){

            node.coords.first = colPosition;
            node.coords.second = rowPosition;
            neighbors.push_back(node);
          }
        }
      }
    }

    return neighbors;
  }

  /* @Brief: converts a 3D octomap to a 2D Occupancy Grid Map
     @Param: none
     @Return: void
  */ 
  void generateGridMap(){
    double x, y, z;
    double res = map->getResolution();

    int length, width, height;
    
    // get map dimensions
    map->getMetricSize(x, y, z);
    length = x / res;
    width = y / res;
    height = z / res;

    // resize gridmap according to map dimensions
    gridMap.resize(length);
    for(int i = 0; i < gridMap.size(); i++){
      gridMap[i].resize(width);
    }

    // iterate through octomap and project down to 2D plane
    for(octomap::OcTree::leaf_iterator it = map->begin_leafs(), end = map->end_leafs(); it != end; ++it){

      // only get highest resolution voxels with certain occupancy value
      if(it->getOccupancy() > .4 && it.getDepth() == 16){

        // only get voxels below certain height
        if(it.getZ() < .15){
          
          //convert map coordinate to gridmap index
          int x_index = (floor(it.getX() * 10) + length/ 2); std::cout << x_index << ", ";

          int y_index = (floor(it.getY() * 10) + width / 2); std::cout << y_index << std::endl;

          int offset = 1; //2 * it.getSize() / res;

          // plot obstacle in gridmap
          for(int i = 0; i < offset; i++){
            for(int j = 0; j < offset; j++){
              if(x_index >= 0 && y_index >= 0 && x_index < length && y_index < width)
                gridMap[x_index][y_index] = 1;
            }
            std::cout << std::endl;
          }

        }
      }
    }

    // print gridmap
    for(int i = 0; i < gridMap.size(); i++){
      for(int j = 0; j < gridMap[i].size(); j++){
        std::cout << gridMap[i][j] << " ";
      }
      std::cout << std::endl;
    }

    // write gridmap to file
    std::ofstream file;
    file.open ("Gridmap.txt");

    for(int i = 0; i < gridMap.size(); i++){
      for(int j = 0; j < gridMap[i].size(); j++){
        if(gridMap[i][j] > 0)
	  file << '#' << ' ';
        else
          file << "  ";
      }
      file << std::endl;
    }
    file.close();

  }

  /* @Brief: converts a ROS point to octomap 3D point
     @Param: point - A ROS point message
     @Return: Octomap 3D point
  */ 
  octomap::point3d ros_to_oc_point(geometry_msgs::Point32 point){
    return octomap::point3d(point.x, point.y, point.z);
  }

  /* @Brief: Retrieve Octomap
     @Param: none
     @Return: A pointer to the Octomap
  */ 
  octomap::OcTree* getMap(){
    return map;
 }

  private:

  octomap::OcTree* map; //pointer to the octomap

  std::vector<std::vector<float>> gridMap; //2D occupancy gridmap

};

int main(int argc, char** argv)
{
  octomap::OcTree ocMap = octomap::OcTree("simple_tree.bt"); //get octomap from file

  PathPlanner planner = PathPlanner(&ocMap);

  octomap::point3d start = octomap::point3d(-2, -2, -2); // start point in map coordinates

  octomap::point3d end = octomap::point3d(3, 3, 3); // end point in map cordinates

  planner.generateGridMap();

  //planner.A_star(start, end);

  return 0;
}

