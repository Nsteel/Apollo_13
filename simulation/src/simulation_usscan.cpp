#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/MapMetaData.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <Raycaster.h>
#include <string>
#include <stdexcept>
#include <sensor_msgs/Range.h>

typedef std::shared_ptr<sensor_msgs::Range> scan_msg_ptr;
typedef std::shared_ptr<geometry_msgs::Pose> pose_msg_ptr;

const double loopRate = 40.0;
nav_msgs::MapMetaData mapInfo;

void callMapMetaData(const nav_msgs::MapMetaData::ConstPtr& mapData)
{
        mapInfo = *mapData;
}

pose_msg_ptr setSensorPosition(const tf::TransformListener& listener, const std::string perspective){
        tf::StampedTransform stf;
        listener.lookupTransform("map", perspective, ros::Time(0), stf);
        tf::Stamped<tf::Pose> tmp(stf, stf.stamp_, "map");
        geometry_msgs::PoseStamped tmp2;
        tf::poseStampedTFToMsg(tmp,tmp2);
        return std::make_shared<geometry_msgs::Pose>(tmp2.pose);
}

rc::vec2i_ptr setGridPosition(geometry_msgs::Pose& sensor){
        unsigned int grid_x = (unsigned int)((sensor.position.x - mapInfo.origin.position.x) / mapInfo.resolution);
        unsigned int grid_y = (unsigned int)((-sensor.position.y - mapInfo.origin.position.y) / mapInfo.resolution);

        return std::make_shared<cv::Vec2i>(cv::Vec2i(grid_x, grid_y));
}

double getYaw(geometry_msgs::Pose& laser){
        double roll, pitch, yaw;
        try{
                tf::Quaternion q;
                tf::quaternionMsgToTF(laser.orientation, q);
                q = q.normalize();
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        }catch(std::exception ex) {
                //ROS_ERROR("%s", ex.what());
        }

        if(!isnan(yaw)) {
                return rc::radToDeg(yaw);
        }else{
                return 0.0;
        }
}

int main(int argc, char **argv){

        ros::init(argc, argv, "simulation_usscan");
        ros::NodeHandle nh;

        ros::Subscriber mapMetaData = nh.subscribe<nav_msgs::MapMetaData>("map_metadata", 10, callMapMetaData);
        ros::Publisher front_us_range = nh.advertise<sensor_msgs::Range>("front_us_range", 10);
        ros::Publisher left_us_range = nh.advertise<sensor_msgs::Range>("left_us_range", 10);
        ros::Publisher right_us_range = nh.advertise<sensor_msgs::Range>("right_us_range", 10);

        sensor_msgs::Range front_range, left_range, right_range;

        std::string imagePath = ros::package::getPath("simulation") + "/data/map/map.pgm";
        cv::Mat map = cv::imread(imagePath,1);
        rc::Raycaster rc_ussensor = rc::Raycaster(map, 0.05, 60.0, 45.0, 0.5);

        tf::TransformListener listener;
        geometry_msgs::Pose ussensorPosition;
        rc::vec2i_ptr gridPose;
        double yaw;

        ros::Time currentTime = ros::Time::now();

        front_range.header.stamp = currentTime;
        front_range.header.frame_id = "front_sensor";
        front_range.radiation_type = 0;
        front_range.field_of_view = rc::degToRad(45.0);
        front_range.min_range = 0.00;
        front_range.max_range = 2.7;

        left_range.header.stamp = currentTime;
        left_range.header.frame_id = "left_sensor";
        left_range.radiation_type = 0;
        left_range.field_of_view = rc::degToRad(45.0);
        left_range.min_range = 0.0;
        left_range.max_range = 2.7;

        right_range.header.stamp = currentTime;
        right_range.header.frame_id = "right_sensor";
        right_range.radiation_type = 0;
        right_range.field_of_view = rc::degToRad(45.0);
        right_range.min_range = 0.00;
        right_range.max_range = 2.7;

        // Loop starts here:
        ros::Rate loop_rate(loopRate);
        while(ros::ok()) {
                currentTime = ros::Time::now();
                front_range.header.stamp = currentTime;
                left_range.header.stamp = currentTime;
                right_range.header.stamp = currentTime;
                //front sensor
                try{
                        ussensorPosition = *setSensorPosition(listener, "front_sensor");
                }catch(tf::TransformException ex) {
                        //ROS_ERROR("%s", ex.what());
                }
                gridPose = setGridPosition(ussensorPosition);
                yaw = getYaw(ussensorPosition);
                front_range.range = rc_ussensor.getUsRangeInfo(gridPose, yaw);
                //left sensor
                try{
                        ussensorPosition = *setSensorPosition(listener, "left_sensor");
                }catch(tf::TransformException ex) {
                        // ROS_ERROR("%s", ex.what());
                }
                gridPose = setGridPosition(ussensorPosition);
                yaw = getYaw(ussensorPosition);
                left_range.range = rc_ussensor.getUsRangeInfo(gridPose, yaw);
                //right sensor
                try{
                        ussensorPosition = *setSensorPosition(listener,"right_sensor");
                }catch(tf::TransformException ex) {
                        // ROS_ERROR("%s", ex.what());
                }
                gridPose = setGridPosition(ussensorPosition);
                yaw = getYaw(ussensorPosition);
                right_range.range = rc_ussensor.getUsRangeInfo(gridPose, yaw);

                front_us_range.publish(front_range);
                right_us_range.publish(right_range);
                left_us_range.publish(left_range);

                ros::spinOnce();
                loop_rate.sleep();
        }

        ros::spin();
}
