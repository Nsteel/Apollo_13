#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/MapMetaData.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <Raycaster.h>
#include <string>
#include <stdexcept>
#include <sensor_msgs/Range.h>
#include <yaml-cpp/yaml.h>

typedef std::shared_ptr<sensor_msgs::LaserScan> scan_msg_ptr;
typedef std::shared_ptr<geometry_msgs::Pose> pose_msg_ptr;

namespace YAML {
template<>
struct convert<geometry_msgs::Pose> {
        static Node encode(const geometry_msgs::Pose& rhs) {
                Node node;
                node.push_back(rhs.position.x);
                node.push_back(rhs.position.y);
                node.push_back(rhs.position.z);
                return node;
        }

        static bool decode(const Node& node, geometry_msgs::Pose& rhs) {
                if(!node.IsSequence() || node.size() != 3) {
                        return false;
                }

                rhs.position.x = node[0].as<double>();
                rhs.position.y = node[1].as<double>();
                rhs.position.z = node[2].as<double>();
                return true;
        }
};
}

rc::vec2i_ptr setGridPosition(geometry_msgs::Pose& laser, nav_msgs::MapMetaData& mapInfo){
        unsigned int grid_x = (unsigned int)((laser.position.x - mapInfo.origin.position.x) / mapInfo.resolution);
        unsigned int grid_y = (unsigned int)((-laser.position.y - mapInfo.origin.position.y) / mapInfo.resolution);
        return std::make_shared<cv::Vec2i>(cv::Vec2i(grid_x, grid_y));
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
                        ROS_ERROR("%s", ex.what());
                }

        }
}

int main(int argc, char **argv){

        ros::init(argc, argv, "simulation_laserscan");
        ros::NodeHandle nh;

        ros::Publisher scanPub = nh.advertise<sensor_msgs::LaserScan>("scan", 10);

        sensor_msgs::LaserScan scan;

        // get "god" map meta info
        nav_msgs::MapMetaData mapInfo;
        std::string imgMetaPath = ros::package::getPath("simulation") + "/data/map/map.yaml";
        YAML::Node imgMetaInfo = YAML::LoadFile(imgMetaPath);
        double resolution = imgMetaInfo["resolution"].as<double>();
        geometry_msgs::Pose origin = imgMetaInfo["origin"].as<geometry_msgs::Pose>();
        mapInfo.origin = origin;
        mapInfo.resolution = resolution;

        // get "god" map
        std::string imagePath = ros::package::getPath("simulation") + "/data/map/map.pgm";
        cv::Mat map = cv::imread(imagePath,1);

        // sensor object
        rc::Raycaster rc_laser = rc::Raycaster(map, 0.05, 120.0, 90.0, 0.5);

        // variables needed for transformations
        tf::TransformListener listener;
        geometry_msgs::Pose position;
        std::vector<double> rpy(3, 0.0);
        rc::vec2i_ptr gridPose;

        ros::Time currentTime = ros::Time::now();

        scan.header.stamp = currentTime;
        scan.header.frame_id = "scan";
        scan.angle_increment = rc::degToRad(0.5);
        scan.range_min = 0.0;
        scan.range_max = 5.8;
        std::pair<double,double> minMax = *rc_laser.angleMinMax(0);
        scan.angle_min = rc::degToRad(minMax.first);
        scan.angle_max = rc::degToRad(minMax.second);

        // Loop starts here:
        ros::Rate loop_rate(50);
        while(ros::ok()) {
                currentTime = ros::Time::now();
                scan.header.stamp = currentTime;

                //get laser range infos
                getPositionInfo("map", "scan", listener, &position, &rpy);
                gridPose = setGridPosition(position, mapInfo);
                scan.ranges = *rc_laser.getRangeInfo(gridPose, rc::radToDeg(rpy[2]));

                //publish sensor msgs
                scanPub.publish(scan);
                ros::spinOnce();
                loop_rate.sleep();
        }

        ros::spin();
}
