#include "GlobalPlannerAdapter.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlannerAdapter, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace global_planner {

GlobalPlannerAdapter::GlobalPlannerAdapter (){
        ros::NodeHandle nh;
        pathSub = nh.subscribe<nav_msgs::Path>("pathtransformPlanner/path", 10, boost::bind(pathCallback, _1, &path));
        path = nav_msgs::Path();
}

GlobalPlannerAdapter::GlobalPlannerAdapter(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
        ros::NodeHandle nh;
        pathSub = nh.subscribe<nav_msgs::Path>("pathtransformPlanner/path", 10, boost::bind(pathCallback, _1, &path));
        path = nav_msgs::Path();
}


void GlobalPlannerAdapter::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

}

bool GlobalPlannerAdapter::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

        if(path.poses.size()>1) {
                plan = path.poses;
                return true;
        }else{
                return false;
        }

}

};
