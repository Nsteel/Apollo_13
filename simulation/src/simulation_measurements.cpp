#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <vector>
#include <string>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <simulation/telemetry_msg.h>
#include <automap/automap_ctrl_msg.h>

static const double NSEC_PER_SEC = 1000000000.0;

void mapMetaCallback(const nav_msgs::MapMetaData::ConstPtr& metaMsg, nav_msgs::MapMetaData *meta){
        *meta = *metaMsg;
}

void ctrlCallback(const automap::automap_ctrl_msg::ConstPtr& ctrlMsg, automap::automap_ctrl_msg* ctrlSignals){
        *ctrlSignals = *ctrlMsg;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg, nav_msgs::OccupancyGrid *map){
        *map = *mapMsg;
}

void telemetryCallback(const simulation::telemetry_msg::ConstPtr& tele, simulation::telemetry_msg* telemetry){
        *telemetry = *tele;
}

cv::Mat occupancyGridToImage(const nav_msgs::OccupancyGrid& grid){
        int width = grid.info.width;
        int height = grid.info.height;
        cv::Mat map = cv::Mat(width,height,CV_8UC1);
        if(grid.data.size()>0) {
                for(int i = 0; i<height; i++) {
                        for(int j = 0; j<width; j++) {
                                int value = grid.data[i*width+j];
                                int col = 0;
                                if(value==0) {
                                        col = 255;
                                }else if(value==100) {
                                        col = 0;
                                }else{
                                        col = 205;
                                }
                                map.at<uchar>(height-(i+1), j) = col;
                        }
                }
        }
        return map;
}

void setGridPosition(geometry_msgs::Pose& laser, nav_msgs::MapMetaData& mapInfo, cv::Point * gridPose){
        unsigned int grid_x = (unsigned int)((laser.position.x - mapInfo.origin.position.x) / mapInfo.resolution);
        unsigned int grid_y = (unsigned int)((-laser.position.y - mapInfo.origin.position.y) / mapInfo.resolution);
        gridPose->x = grid_x;
        gridPose->y = grid_y;
}

void getPositionInfo(const std::string& base_frame, const std::string& target_frame,
                     const tf::TransformListener& listener, geometry_msgs::Pose * position, std::vector<double> * rpy){
        bool transformReady = false;
        tf::StampedTransform stf;
        tf::Stamped<tf::Pose> tmp;
        geometry_msgs::PoseStamped tmp2;
        tf::Quaternion q;

        try{
                transformReady = listener.waitForTransform (base_frame, target_frame, ros::Time(0), ros::Duration(0.01));

        }
        catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());

        }

        if(transformReady) {
                try{
                        listener.lookupTransform(base_frame, target_frame, ros::Time(0), stf);

                }
                catch (tf::TransformException ex) {
                        ROS_ERROR("%s",ex.what());

                }
                tmp = tf::Stamped<tf::Pose>(stf, stf.stamp_, base_frame);
                tf::poseStampedTFToMsg(tmp,tmp2);
                *position = tmp2.pose;

                try{
                        tf::quaternionMsgToTF(tmp2.pose.orientation, q);
                        tf::Matrix3x3(q).getRPY((*rpy)[0], (*rpy)[1], (*rpy)[2]);
                }catch(std::exception ex) {
                        //ROS_ERROR("%s", ex.what());
                }

        }
}


int main(int argc, char **argv){

        ros::init(argc, argv, "simulation_measurements");
        ros::NodeHandle nh;

        // exploration timer
        ros::Time initTime = ros::Time::now();

        nav_msgs::MapMetaData mapMetaData;
        nav_msgs::OccupancyGrid grid;
        //cv::Mat map;

        simulation::telemetry_msg telemetry;

        // ctrl signals used for termination
        automap::automap_ctrl_msg ctrlSignals;

        tf::TransformListener listener;
        geometry_msgs::Pose position;
        std::vector<double> rpy(3, 0.0);
        cv::Point gridPose;

        ros::Subscriber mapMetaSub = nh.subscribe<nav_msgs::MapMetaData>("map_metadata", 10, boost::bind(mapMetaCallback, _1, &mapMetaData));
        ros::Subscriber ctrlSub = nh.subscribe<automap::automap_ctrl_msg>("automap/ctrl_msg", 10, boost::bind(ctrlCallback, _1, &ctrlSignals));
        ros::Subscriber mapSub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 10, boost::bind(mapCallback, _1, &grid));
        ros::Subscriber robotInfo = nh.subscribe<simulation::telemetry_msg>("telemetry", 10, boost::bind(telemetryCallback, _1, &telemetry));

        //wait for map - server
        ros::Duration d = ros::Duration(2, 0);
        ros::spinOnce();
        while(mapMetaData.resolution == 0 && ros::ok()) {
                ROS_INFO("Waiting for the map server to come up...");
                d.sleep();
                ros::spinOnce();
        }
        while(grid.data.size() == 0 && ros::ok()) {
                ROS_INFO("Waiting for the map server to come up...");
                d.sleep();
                ros::spinOnce();
        }

        // Begin: exploration metrics
        // array for drawing the robots path
        std::vector<cv::Point> explorationPath;

        // use onle this signal for termination
        ctrlSignals.control_On=false;

        // Loop starts here:
        ros::Rate loop_rate(5);
        while(ros::ok() && !ctrlSignals.control_On) {

                //Get Position Information...
                getPositionInfo("map", "base_footprint", listener, &position, &rpy);
                setGridPosition(position, mapMetaData, &gridPose);
                //Remember path
                explorationPath.push_back(gridPose);

                ros::spinOnce();
                loop_rate.sleep();
        }

        ROS_INFO("Map exploration finished, aborting loop...");

        std::string imgMetaPath = ros::package::getPath("simulation") + "/data/output/map.yaml";
        std::string imgStatPath = ros::package::getPath("simulation") + "/data/output/exploration_statistics.txt";
        std::string imgPath = ros::package::getPath("simulation") + "/data/output/map.pgm";
        std::string recordedPathPath = ros::package::getPath("simulation") + "/data/output/path.png";

        cv::Mat map = occupancyGridToImage(grid);

        //Save explored map
        std::vector<int> com_param;
        com_param.push_back(CV_IMWRITE_PNG_COMPRESSION);
        com_param.push_back(9);
        try {
                cv::imwrite(imgPath, map, com_param);
                ROS_INFO("Map written to: %s", imgPath.c_str());

        } catch (std::runtime_error& ex) {
                std::cout << "Exception converting img to PNG: " << ex.what() << std::endl;
        }

        //Save exploration path
        cv::Mat pathMap = map.clone();
        cv::cvtColor(pathMap, pathMap, CV_GRAY2RGB);
        cv::polylines(pathMap, explorationPath, false, cv::Scalar(0, 0, 255), 1, 8);
        try {
                cv::imwrite(recordedPathPath, pathMap, com_param);
                ROS_INFO("Path written to: %s", recordedPathPath.c_str());

        } catch (std::runtime_error& ex) {
                std::cout << "Exception converting img to PNG: " << ex.what() << std::endl;
        }

        YAML::Emitter Y_out;
        Y_out << YAML::BeginMap;
        Y_out << YAML::Key << "image";
        Y_out << YAML::Value << "map.pgm";
        Y_out << YAML::Key << "resolution";
        Y_out << YAML::Value << mapMetaData.resolution;
        Y_out << YAML::Key << "origin";
        Y_out << YAML::Flow;
        Y_out << YAML::BeginSeq << mapMetaData.origin.position.x<<mapMetaData.origin.position.y
              <<mapMetaData.origin.position.z << YAML::EndSeq;
        Y_out << YAML::Key << "negate";
        Y_out << YAML::Value << 0;
        Y_out << YAML::Key << "occupied_thresh";
        Y_out << YAML::Value << 0.65;
        Y_out << YAML::Key << "free_thresh";
        Y_out << YAML::Value << 0.196;
        Y_out << YAML::EndMap;


        YAML::Emitter Y_out_statistics;
        Y_out_statistics << YAML::BeginMap;
        Y_out_statistics << YAML::Key << "driven distance during exploration(m)";
        Y_out_statistics << YAML::Value << telemetry.radial_distance;
        Y_out_statistics << YAML::Key << "time elapsed during exploration(s)";
        Y_out_statistics << YAML::Value << (ros::Time::now()-initTime).toNSec()/NSEC_PER_SEC;

        // calc quality of map
        std::string godMapPath = ros::package::getPath("simulation") + "/data/map/map.pgm";
        cv::Mat godMap = cv::imread(godMapPath,1);
        cv::cvtColor(godMap, godMap, CV_RGB2GRAY);
        double diffMap = cv::norm(godMap, map, CV_L2);

        Y_out_statistics << YAML::Key << "in comparison to god map (L2Norm)";
        Y_out_statistics << YAML::Value << diffMap;
        Y_out_statistics << YAML::EndMap;

        std::ofstream outfile_yaml(imgMetaPath);
        try{
                outfile_yaml<<Y_out.c_str();
                ROS_INFO("MapMetaData written to: %s", imgMetaPath.c_str());
        }catch (std::runtime_error& ex) {
                std::cout << "Exception writing .yaml-file: " << ex.what() << std::endl;
        }
        outfile_yaml.close();

        std::ofstream outfile_statistics(imgStatPath);
        try{
                outfile_statistics<<Y_out_statistics.c_str();
                ROS_INFO("Statistics data written to: %s", imgStatPath.c_str());
        }catch (std::runtime_error& ex) {
                std::cout << "Exception writing .txt-file: " << ex.what() << std::endl;
        }
        outfile_statistics.close();

        ros::shutdown();

        ros::spin();
}
