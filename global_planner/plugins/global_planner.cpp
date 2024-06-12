#include <pluginlib/class_list_macros.h>
#include "global_planner.h"
#include "ros/ros.h"
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/Int16MultiArray.h"
#include "cmath"
#include "iostream"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h> 
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <vector>
#include <nav_msgs/Path.h>
#include <mutex>

PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

int map_row, map_col;
int mapSize;
bool* OGM;
static const float INFINITY_COST = INT_MAX;
float infinity = std::numeric_limits< float >::infinity();
float start_x, start_y;
float goal_x, goal_y;
// int goal_coords[2], start_coords[2];
static int check_start = 1;
nav_msgs::OccupancyGrid map;
int check;
int newGoal = 0;
int done = 0;
int checkGetCost;
std::vector<int> astar_route;
std::mutex myMutex;

int sub2ind(int rowIndex, int colIndex, int numCols);
std::vector<int> ind2sub(int index, int numCols);
std::pair<double, int> findMin(const std::vector<double>& array);

std::vector<int> ToanAStar(nav_msgs::OccupancyGrid input_map, int start_coords[2], int goal_coords[2]);

namespace global_planner {

    GlobalPlanner::GlobalPlanner () {

    }

    // GlobalPlanner::GlobalPlanner(ros::NodeHandle &n) {
    //     ROSNodeHandle = n;
    // }

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        initialize(name, costmap_ros);
    }


    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        if (!initialized_)
        {
            // ros::NodeHandle private_n("~/"+name);
            
            
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            originX = costmap_->getOriginX();
            originY = costmap_->getOriginY();

            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            resolution = costmap_->getResolution();
            mapSize = width*height;

            OGM =   new bool [mapSize];
            ROS_INFO("originX = %f, originY = %f, resolution = %f",originX,originY,resolution);
            ROS_INFO("width = %d, height = %d, mapSize = %d",width,height,mapSize);

            ROS_INFO("A STAR GLOBAL PLANNER INITIALIZED SUCCESSFULLY"); 
            initialized_ = true;           
        }
    }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ) {
        
        tf::Stamped < tf::Pose > goal_tf;
        tf::Stamped < tf::Pose > start_tf;
        tf::poseStampedMsgToTF(goal,goal_tf);
        tf::poseStampedMsgToTF(start,start_tf);
        
        // ros::NodeHandle n;
        ROS_INFO("START MAKEPLAN");
        static int makePlanCall = 0;
        makePlanCall++;
        float startX = start.pose.position.x;
        float startY = start.pose.position.y;
        std::vector<int> startCoords = getCellCoord(startX,startY);

        float goalX = goal.pose.position.x;
        float goalY = goal.pose.position.y;
        std::vector<int> goalCoords = getCellCoord(goalX,goalY);

        std::vector<int> path;

        path.clear();
        plan.clear();
        int startCell = getCellIndex(startCoords[0],startCoords[1]);
        int goalCell = getCellIndex(goalCoords[0],goalCoords[1]);
        static int preGoalCell = 0;

        ROS_INFO("Goal cost = %d",costmap_->getCost(goalCoords[0],goalCoords[1]));
        if (costmap_->getCost(goalCoords[0],goalCoords[1]) > 203) {
            ROS_WARN("Goal is obstacle or possibly in collision");
            return false;
        }

        if (startCell != goalCell) {
            path = ToanAStar(startCoords,goalCoords);
            // preGoalCell = goalCell;
            // makePlanCall = 0;

        if (path.size()>0) {
            plan.push_back(start);
            for (int i = 0; i < path.size(); i++) {
                std::vector<float> wCoord = getWorldCoord(path[i]);
                geometry_msgs::PoseStamped pose = goal;
                pose.header.frame_id = goal.header.frame_id;

                pose.pose.position.x = wCoord[0];
                pose.pose.position.y = wCoord[1];
                pose.pose.position.z = 0.0;

                pose.pose.orientation.x = goal.pose.orientation.x;
                pose.pose.orientation.y = goal.pose.orientation.y;
                pose.pose.orientation.z = goal.pose.orientation.z;
                pose.pose.orientation.w = goal.pose.orientation.w;
                // pose.pose.orientation.x = 0.0;
                // pose.pose.orientation.y = 0.0;
                // pose.pose.orientation.z = 0.0;
                // pose.pose.orientation.w = 1.0;

                plan.push_back(pose);
            }
            plan.push_back(goal);

            nav_msgs::Path path_msg;
            path_msg.header.frame_id="map";
            path_msg.poses = plan;
            plan_pub.publish(path_msg);

            return true;
        }
        else {
            ROS_INFO("Can't find path");
            return false;
        }
        }
        
        else return false;
        

        
        // plan.push_back(start);
        // plan.push_back(goal);

        // return true;
    }

    

    std::vector<int> GlobalPlanner::getCellCoord(float wx, float wy)
    {
        int mx, my;
        mx = (int)((wx-originX)/resolution);
        my = (int)((wy-originY)/resolution);
        return {mx,my};
    }
    
    std::vector<int> GlobalPlanner::ToanAStar(std::vector<int> start_coords,std::vector<int> goal_coords)
    {
        std::vector<int> route;
        route.clear();
        ROS_INFO("START ASTAR");
        int start_node = costmap_->getIndex(start_coords[0],start_coords[1]);
        int goal_node = costmap_->getIndex(goal_coords[0],goal_coords[1]);
        double infinity = std::numeric_limits<double>::infinity();

        std::vector<double> open_list(mapSize,infinity);

        std::vector<double> close_list(mapSize,infinity);

        std::vector<int> parent(mapSize,0);
        
        std::vector<double> G_cost(mapSize,infinity);

        std::vector<double> G_cost_calculating(mapSize,infinity);

        std::vector<double> H_cost(mapSize,0);

        std::vector<double> F_cost(mapSize,infinity);

        std::vector<double> F_cost_open_node(mapSize,infinity);

        // Calculate Hcost for every node in the map
        // int node = 0;
        // for (node = 0; node < mapSize; node++) {
        //     std::vector<int> node_coords = indexToCellCoord(node);
        //     H_cost[node] = std::sqrt(std::pow(goal_coords[0] - node_coords[0],2) + std::pow(goal_coords[1] - node_coords[1],2))*20;
        // }

        // start node
        G_cost[start_node] = 0;
        // F_cost[start_node]= G_cost[start_node] + H_cost[start_node];
        F_cost[start_node]= G_cost[start_node] + getHcost(goal_coords[0],goal_coords[1],start_coords[0],start_coords[1]);

        // add start node to open
        open_list[start_node] = 1;
        F_cost_open_node[start_node] = F_cost[start_node]*open_list[start_node];

        while(true) {
            // check node in the open list with the lowest Fcost
            
            // int open_node;
            // for (open_node = 0; open_node < mapSize; open_node++) {
            //     F_cost_open_node[open_node] = F_cost[open_node]*open_list[open_node];
            // }
            
            std::pair<double, int> minF = findMin(F_cost_open_node);
            int current_node = minF.second;
            if (current_node == goal_node || minF.first == infinity ) {
                break;
            }

            // remove current node from open list
            open_list[current_node] = infinity;
            F_cost_open_node[current_node] = infinity;

            // add current node to close list
            close_list[current_node] = 1;

            std::vector<int> current_node_coords = indexToCellCoord(current_node);
            int searchRange = 4;
            if (abs(current_node_coords[0]-goal_coords[0])<10 && abs(current_node_coords[1]-goal_coords[1])<10) {
                searchRange = 1;
            }
            else searchRange = 4;

            int neighbor_row, neighbor_col;
            // visit each neighbor of the current node
            for (int n = 1; n <= 8; n++) {
                if  (n == 1) {
                    neighbor_col = current_node_coords[0]-searchRange;
                    neighbor_row = current_node_coords[1];
                }
                else if (n == 2) {
                    neighbor_col = current_node_coords[0]+searchRange;
                    neighbor_row = current_node_coords[1];
                }
                else if (n == 3) {
                    neighbor_col = current_node_coords[0];
                    neighbor_row = current_node_coords[1]-searchRange;
                }
                else if (n == 4) {
                    neighbor_col = current_node_coords[0];
                    neighbor_row = current_node_coords[1]+searchRange;
                }
                else if (n == 5) {
                    neighbor_col = current_node_coords[0]-searchRange;
                    neighbor_row = current_node_coords[1]-searchRange;
                }
                else if (n == 6) {
                    neighbor_col = current_node_coords[0]-searchRange;
                    neighbor_row = current_node_coords[1]+searchRange;
                }
                else if (n == 7) {
                    neighbor_col = current_node_coords[0]+searchRange;
                    neighbor_row = current_node_coords[1]-searchRange;
                }
                else if (n == 8) {
                    neighbor_col = current_node_coords[0]+searchRange;
                    neighbor_row = current_node_coords[1]+searchRange;
                }
                // if out of the map -> skip it
                if (neighbor_row<0 || neighbor_row>=height) {
                    continue;
                }
                if (neighbor_col<0 || neighbor_col>=width) {
                    continue;
                }
                // set neighbor
                int neighbor = getCellIndex(neighbor_col,neighbor_row);
                // if neighbor is obstacle or in close -> skip it
                if  (costmap_->getCost(neighbor_col,neighbor_row) > 203) { // obstacle
                    continue;
                }

                double collision = 3.6;
                if (costmap_->getCost(neighbor_col,neighbor_row) > 127) {
                    collision = 36;
                }
                else if (costmap_->getCost(neighbor_col,neighbor_row) > 90) {
                    collision = 10;
                }
                else collision = 3.6;

                if (close_list[neighbor] == 1) { // in close list
                    continue;
                }

                G_cost_calculating[neighbor] = G_cost[current_node] + (std::sqrt(std::pow(current_node_coords[1]-neighbor_row,2)+std::pow(current_node_coords[0]-neighbor_col,2)))*36;
                // G_cost_calculating[neighbor] = G_cost[current_node] + std::sqrt(std::pow(current_node_coords[1]-neighbor_row,2)+std::pow(current_node_coords[0]-neighbor_col,2));
                if (G_cost_calculating[neighbor]<G_cost[neighbor]) {
                    G_cost[neighbor] = G_cost_calculating[neighbor];
                    parent[neighbor] = current_node; // set current node to parent of neighbor
                }

                // F_cost[neighbor] = G_cost[neighbor] + H_cost[neighbor];
                F_cost[neighbor] = G_cost[neighbor] + getHcost(goal_coords[0],goal_coords[1],neighbor_col,neighbor_row) + costmap_->getCost(neighbor_col,neighbor_row)*collision;

                if (open_list[neighbor] == infinity) {
                    open_list[neighbor] = 1; // add neighbor to open list
                    F_cost_open_node[neighbor] = F_cost[neighbor];
                }
            }
        }
        if (F_cost[goal_node] == infinity) {
            bool noPath = true;
        }
        else {
            int node = goal_node;
            while (true) {
                route.insert(route.begin(),node);
                node = parent[node];
                if (node == start_node) {
                    // done = 1;
                    ROS_INFO("DONE ASTAR");
                    break;
                }
            }
        }

        return route;
    }
};

