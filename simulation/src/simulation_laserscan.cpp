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

typedef std::shared_ptr<sensor_msgs::LaserScan> scan_msg_ptr;
typedef std::shared_ptr<geometry_msgs::Pose> pose_msg_ptr;

const double loopRate = 40.0;
nav_msgs::MapMetaData mapInfo;

void callMapMetaData(const nav_msgs::MapMetaData::ConstPtr& mapData)
{
        mapInfo = *mapData;
}

pose_msg_ptr setLaserPosition(const tf::TransformListener& listener){
        tf::StampedTransform stf;
        listener.lookupTransform("map", "scan", ros::Time(0), stf);
        tf::Stamped<tf::Pose> tmp(stf, stf.stamp_, "map");
        geometry_msgs::PoseStamped tmp2;
        tf::poseStampedTFToMsg(tmp,tmp2);
        return std::make_shared<geometry_msgs::Pose>(tmp2.pose);
}

rc::vec2i_ptr setGridPosition(geometry_msgs::Pose& laser){
        unsigned int grid_x = (unsigned int)((laser.position.x - mapInfo.origin.position.x) / mapInfo.resolution);
        unsigned int grid_y = (unsigned int)((-laser.position.y - mapInfo.origin.position.y) / mapInfo.resolution);

        return std::make_shared<cv::Vec2i>(cv::Vec2i(grid_x, grid_y));
}

double getYaw(geometry_msgs::Pose& laser){
        double roll, pitch, yaw;
        try{
                tf::Quaternion q;
                //q = q.normalize();
                tf::quaternionMsgToTF(laser.orientation, q);
                //ROS_INFO("x %f y %f z %f w %f" ,laser.orientation.x, laser.orientation.y, laser.orientation.z, laser.orientation.w);
                //q = q.normalize();
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        }catch(std::exception ex) {
                //ROS_ERROR("%s", ex.what());
        }
        //ROS_INFO("YAW: %f", yaw);
        if(!isnan(yaw)) {
                return rc::radToDeg(yaw);
        }else{
                return 0.0;
        }
}

int main(int argc, char **argv){

        ros::init(argc, argv, "simulation_laserscan");
        ros::NodeHandle nh;

        ros::Publisher scanPub = nh.advertise<sensor_msgs::LaserScan>("scan", 10);
        ros::Subscriber mapMetaData = nh.subscribe<nav_msgs::MapMetaData>("map_metadata", 10, callMapMetaData);

        sensor_msgs::LaserScan scan;

        std::string imagePath = ros::package::getPath("simulation") + "/data/map/map.pgm";
        cv::Mat map = cv::imread(imagePath,1);
        rc::Raycaster rc_laser = rc::Raycaster(map, 0.05, 120.0, 90.0, 0.5);
        tf::TransformListener listener;
        geometry_msgs::Pose laserPosition;
        rc::vec2i_ptr gridPose;
        double yaw;

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
        ros::Rate loop_rate(loopRate);
        while(ros::ok()) {
                currentTime = ros::Time::now();
                scan.header.stamp = currentTime;
                try{
                        laserPosition = *setLaserPosition(listener);
                }catch(tf::TransformException ex) {
                        //ROS_ERROR("%s", ex.what());
                }
                gridPose = setGridPosition(laserPosition);
                yaw = getYaw(laserPosition);
                scan.ranges = *rc_laser.getRangeInfo(gridPose, yaw);

                scanPub.publish(scan);
                ros::spinOnce();
                loop_rate.sleep();
        }

        ros::spin();
}
