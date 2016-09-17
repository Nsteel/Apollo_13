#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <PathtransformPlanner.h>
#include <ExplorationPlanner.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <vector>
#include <string>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <simulation/telemetry_msg.h>
#include <automap/automap_ctrl_msg.h>
#include <dynamic_reconfigure/server.h>
#include <automap/ExplorationConfig.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static const double NSEC_PER_SEC = 1000000000.0;

void mapMetaCallback(const nav_msgs::MapMetaData::ConstPtr& metaMsg, nav_msgs::MapMetaData *meta){
        *meta = *metaMsg;
}
void ctrlCallback(const automap::automap_ctrl_msg::ConstPtr& ctrlMsg, automap::automap_ctrl_msg* ctrlSignals){
        *ctrlSignals = *ctrlMsg;
}
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg, cv::Mat *map){
        int width = mapMsg->info.width;
        int height = mapMsg->info.height;
        *map = cv::Mat(width,height,CV_8UC1);
        if(mapMsg->data.size()>0) {
                for(int i = 0; i<height; i++) {
                        for(int j = 0; j<width; j++) {
                                int value = mapMsg->data[i*width+j];
                                int col = 0;
                                if(value==0) {
                                        col = 255;
                                }else if(value==100) {
                                        col = 0;
                                }else{
                                        col = 205;
                                }
                                map->at<uchar>(height-(i+1), j) = col;
                        }
                }
        }

}
void telemetryCallback(const simulation::telemetry_msg::ConstPtr& tele, simulation::telemetry_msg* telemetry){
        *telemetry = *tele;
}

void setGridPosition(geometry_msgs::Pose& laser, nav_msgs::MapMetaData& mapInfo, cv::Point * gridPose){
        unsigned int grid_x = (unsigned int)((laser.position.x - mapInfo.origin.position.x) / mapInfo.resolution);
        unsigned int grid_y = (unsigned int)((-laser.position.y - mapInfo.origin.position.y) / mapInfo.resolution);
        gridPose->x = grid_x;
        gridPose->y = grid_y;
}

void getPositionInfo(const std::string& base_frame, const std::string& target_frame, const tf::TransformListener& listener, geometry_msgs::Pose * position, std::vector<double> * rpy){
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

void sendGoal(nav_msgs::Path& p, MoveBaseClient& ac){
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose =p.poses[p.poses.size()-1];

        ROS_INFO("Sending goal for x:%lf / y:%lf",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y);
        ac.sendGoal(goal);
        //ac.waitForResult();
}

void configCallback(automap::ExplorationConfig& config, uint32_t level, PathtransformPlanner* planner,
  ExplorationPlanner* ePlanner, automap::ExplorationConfig* dynConfig){
        planner->setConfig(config);
        ePlanner->setConfig(config);
        *dynConfig = config;
        ROS_DEBUG("Config was set");
}

int main(int argc, char **argv){

        ros::init(argc, argv, "automap");
        ros::NodeHandle nh;

        // create dynamic reconfigure object
        dynamic_reconfigure::Server<automap::ExplorationConfig> server;
        dynamic_reconfigure::Server<automap::ExplorationConfig>::CallbackType f;
        automap::ExplorationConfig dynConfig;

        ros::Time initTime = ros::Time::now();

        nav_msgs::MapMetaData mapMetaData;
        cv::Mat map;
        simulation::telemetry_msg telemetry;
        sensor_msgs::ImagePtr edgeImageMsg;

        MoveBaseClient ac("move_base", true);
        while(!ac.waitForServer(ros::Duration(5.0)) && ros::ok())
        {
                ROS_INFO("Waiting for the move_base action server to come up...");
        }
        //control_On : motion control on/off / detection_On : sensing on/off /  NBV_On : nbv on/off
        automap::automap_ctrl_msg ctrlSignals;

        tf::TransformListener listener;
        geometry_msgs::Pose position;
        std::vector<double> rpy(3, 0.0);
        cv::Point gridPose;

        ros::Subscriber mapMetaSub = nh.subscribe<nav_msgs::MapMetaData>("map_metadata", 10, boost::bind(mapMetaCallback, _1, &mapMetaData));
        ros::Subscriber ctrlSub = nh.subscribe<automap::automap_ctrl_msg>("automap/ctrl_msg", 10, boost::bind(ctrlCallback, _1, &ctrlSignals));
        ros::Subscriber mapSub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 10, boost::bind(mapCallback, _1, &map));
        ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("pathtransformPlanner/path", 10);
        ros::Subscriber robotInfo = nh.subscribe<simulation::telemetry_msg>("telemetry", 10, boost::bind(telemetryCallback, _1, &telemetry));

        image_transport::ImageTransport it(nh);
        image_transport::Publisher edgePub = it.advertise("floatingEdges", 1);

        //wait for map - server
        ros::Duration d = ros::Duration(2, 0);
        ros::spinOnce();
        while(mapMetaData.resolution == 0 && ros::ok()) {
                ROS_INFO("Waiting for the map server to come up...");
                d.sleep();
                ros::spinOnce();
        }
        while(map.cols==0 && map.rows==0 && ros::ok()) {
                ROS_INFO("Waiting for the map server to come up...");
                d.sleep();
                ros::spinOnce();
        }

        PathtransformPlanner pPlanner(mapMetaData);
        ExplorationPlanner ePlanner(&pPlanner);

        // Begin: exploration metrics
        // array for drawing the robots path
        std::vector<cv::Point> explorationPath;
        // average planner time
        ros::Time planner_timer;
        double t_planner = 0;
        unsigned int t_planner_counter = 0;
        // average ex-planner time
        ros::Time explanner_timer;
        double t_explanner = 0;
        unsigned int t_explanner_counter = 0;
        // average busy loop time
        ros::Time loop_timer;
        double t_loop = 0;
        unsigned int t_loop_counter = 0;

        double timediff = 0;

        f = boost::bind(&configCallback, _1, _2, &pPlanner, &ePlanner, &dynConfig);
        server.setCallback(f);

        ctrlSignals.control_On=dynConfig.node_control_on;
        ctrlSignals.detection_On=dynConfig.node_sensing_on;
        ctrlSignals.NBV_On=dynConfig.node_use_nbv;

        bool finished = false;
        bool finalCheck = false;
        int retry = dynConfig.node_retries;
        cv::Mat old = cv::Mat::zeros(map.rows, map.cols, CV_8UC1);
        // Loop starts here:
        ros::Rate loop_rate(dynConfig.node_loop_rate);
        while(ros::ok() && !finished) {
                //start measure processing time
                loop_timer = ros::Time::now();

                //calculate information gain
                double gain = cv::norm(old, map, CV_L2);
                //Get Position Information...
                getPositionInfo("map", "base_footprint", listener, &position, &rpy);
                setGridPosition(position, mapMetaData, &gridPose);
                //Remember path
                explorationPath.push_back(gridPose);

                if(gain>dynConfig.node_information_gain || retry==0) {
                        ROS_INFO("Enough information gained: %lf",gain);
                        old = map.clone();
                        retry = dynConfig.node_retries;

                        //Feed pathtransformPlanner...
                        try{
                                ROS_INFO("Feeding PathtransformPlanner...");
                                //start measure processing time
                                planner_timer = ros::Time::now();
                                pPlanner.updateTransformMatrices(map, gridPose);
                                //stop measure processing time
                                timediff = (ros::Time::now()-planner_timer).toNSec()/NSEC_PER_SEC;
                                t_planner += timediff;
                                t_planner_counter++;
                                ROS_INFO("Path planning took: %lf",timediff);

                        }catch(std::exception& e) {
                                std::cout<<e.what()<<std::endl;
                        }
                        bool w = false;

                        if(ctrlSignals.detection_On) {
                                //start measure processing time
                                explanner_timer = ros::Time::now();
                                w = ePlanner.findBestPlan(map, gridPose, rpy[2], ctrlSignals.NBV_On);
                                //stop measure processing time
                                timediff = (ros::Time::now()-explanner_timer).toNSec()/NSEC_PER_SEC;
                                t_explanner += timediff;
                                t_explanner_counter++;
                                ROS_INFO("Exploration planning took: %lf",timediff);
                                if(!w && !finalCheck){
                                  finalCheck = true;
                                  continue;
                                }

                        }else{
                                w = true;
                        }



                        if(w) {
                                finalCheck =false;

                                ROS_INFO("Best next Plan found!");
                                std_msgs::Header genericHeader;
                                genericHeader.stamp = ros::Time::now();
                                genericHeader.frame_id = "map";


                                // send map with valid detected Edges
                                if(dynConfig.node_show_exploration_planner_result){
                                  cv::Mat out = ePlanner.drawFrontiers();
                                  edgeImageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", out).toImageMsg();
                                  edgePub.publish(edgeImageMsg);
                                }


                                nav_msgs::Path frontierPath;
                                if(ctrlSignals.detection_On) {
                                        frontierPath = ePlanner.getBestPlan(genericHeader);
                                }


                                if(ctrlSignals.control_On && ctrlSignals.detection_On) {
                                        ROS_INFO("Sending Plan..");
                                        pathPub.publish(frontierPath);
                                        ros::spinOnce();
                                        ROS_INFO("Sending Goal..");
                                        sendGoal(frontierPath, ac);
                                }

                        }else{
                                ROS_INFO("Map exploration finished, aborting loop...");
                                //ac.waitForResult();
                                ac.cancelGoal();
                                ac.waitForResult();

                                std::string imgMetaPath = ros::package::getPath("automap") + "/data/output/map.yaml";
                                std::string imgStatPath = ros::package::getPath("automap") + "/data/output/exploration_statistics.txt";
                                std::string imgPath = ros::package::getPath("automap") + "/data/output/map.pgm";
                                std::string recordedPathPath = ros::package::getPath("automap") + "/data/output/path.png";

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
                                Y_out_statistics << YAML::Key << "average path planner processing time(s)";
                                Y_out_statistics << YAML::Value << t_planner/t_planner_counter;
                                Y_out_statistics << YAML::Key << "average exploration planner processing time(s)";
                                Y_out_statistics << YAML::Value << t_explanner/t_explanner_counter;
                                Y_out_statistics << YAML::Key << "average busy loop processing time(s)";
                                Y_out_statistics << YAML::Value << t_loop/t_loop_counter;

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

                                finished = true;
                                ros::shutdown();


                        }


                }else{
                        ROS_INFO("Not enough information gained: %lf",gain);
                        retry--;
                }
                // resend map with valid detected Edges
                if(dynConfig.node_show_exploration_planner_result){
                  cv::Mat out = ePlanner.drawFrontiers();
                  edgeImageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", out).toImageMsg();
                  edgePub.publish(edgeImageMsg);
                }

                //stop measure processing time
                timediff = (ros::Time::now()-loop_timer).toNSec()/NSEC_PER_SEC;
                t_loop += timediff;
                t_loop_counter++;
                ROS_INFO("Whole loop took: %lf",timediff);

                ros::spinOnce();
                loop_rate.sleep();
        }

        ros::spin();
}
