#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>
#include <vector>

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace global_planner {

    class GlobalPlanner : public nav_core::BaseGlobalPlanner {
        public:

        GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        // GlobalPlanner(ros::NodeHandle &);
        // ros::NodeHandle ROSNodeHandle;

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan);

        // ros::Publisher global_plan_pub;
        // int main(int argc, char **argv) {
        //     ros::init(argc, argv, "send_path");
        //     ros::NodeHandle n;
        //     global_plan_pub = n.advertise<nav_msgs::Path>("/move_base/NavfnROS/plan", 100);
        //     return 0;
        // }
        ros::NodeHandle nh;
        ros::Publisher plan_pub = nh.advertise<nav_msgs::Path>("move_base/NavfnROS/plan",1);

        int sub2ind(int rowIndex, int colIndex, int numCols) {
            return rowIndex*numCols+colIndex;
        }

        std::vector<int> ind2sub(int index, int numCols) {
            int rowIndex = index/numCols;
            int colIndex = index%numCols;
            return {rowIndex,colIndex};
        }
        
        std::pair<double, int> findMin(const std::vector<double>& array) {
            auto minIt = std::min_element(array.begin(), array.end());
            double minValue = *minIt;
            int minIndex = std::distance(array.begin(),minIt);
            return {minValue, minIndex};
        }

        std::vector<int> ToanAStar(std::vector<int> start_coords,std::vector<int> goal_coords);
        std::vector<int> getCellCoord (float x, float y);
        std::vector<int> indexToCellCoord (int index) {
            int rowIndexY = index/width;
            int colIndexX = index%width;
            return {colIndexX,rowIndexY};
        }
        int getCellIndex(int colIndexX, int rowIndexY) {
            int index = rowIndexY*width+colIndexX;
            return index;
        }
        std::vector<float> getWorldCoord (int index) {
            std::vector<int> cellCoord = indexToCellCoord(index);
            float wx = cellCoord[0]*resolution+originX;
            float wy = cellCoord[1]*resolution+originY;
            return {wx,wy};
        }
        double getHcost(int goalCoordX, int goalCoordY, int nodeCoordX, int nodeCoordY) {
            double  Hcost = std::sqrt(std::pow(goalCoordX - nodeCoordX,2) + std::pow(goalCoordY - nodeCoordY,2))*36;
            return Hcost;
        }
        
        int map_row, map_col;
        float start_x, start_y;
        float goal_x, goal_y;
        // int goal_coords[2], start_coords[2];

        nav_msgs::Path path_msg;
        float originX;
        float originY;
        float resolution;
        costmap_2d::Costmap2DROS* costmap_ros_;
        costmap_2d::Costmap2D* costmap_;
        bool initialized_;
        int width;
        int height;
    };
};
#endif