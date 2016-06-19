#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <CarModel.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <simulation/ctrl_msg.h>
#include <simulation/telemetry_msg.h>

geometry_msgs::Twist motion;
simulation::ctrl_msg control;
std::vector<double>simPose;
int sbplFlag;


void motionCommands(const geometry_msgs::Twist::ConstPtr& motionCMD)
{
        motion = *motionCMD;
        sbplFlag = 1;
}

void controlCommands(const simulation::ctrl_msg::ConstPtr& ctrlCMD)
{
        control = *ctrlCMD;
        sbplFlag = 0;
}

int main(int argc, char **argv){

        ros::init(argc, argv, "simulation_control");
        ros::NodeHandle nh;
        ros::Time currentTime = ros::Time::now();

        CarModel car(0.25, currentTime);
        motion = geometry_msgs::Twist();
        control = simulation::ctrl_msg();
        simulation::telemetry_msg telemetry;
        tf::TransformBroadcaster odom_broadcaster;
        sbplFlag = 1;

        ros::Subscriber motionControl = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, motionCommands);
        ros::Subscriber steeringControl = nh.subscribe<simulation::ctrl_msg>("robot_control", 10, controlCommands);
        ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("odom", 10);
        ros::Publisher telemetryPub = nh.advertise<simulation::telemetry_msg>("telemetry", 10);



        // Loop starts here:
        ros::Rate loop_rate(100);
        while(ros::ok()) {
                currentTime = ros::Time::now();
                if(sbplFlag==0) {
                        simPose = *car.getUpdate(control.steering, control.speed, currentTime);
                }else{
                        simPose = *car.getUpdateTwist(motion, currentTime);
                }

                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(simPose[2]);

                // first, we'll publish the transform over tf
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = currentTime;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_footprint";

                odom_trans.transform.translation.x = simPose[1];
                odom_trans.transform.translation.y = -simPose[0];
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = odom_quat;
                // send the transform
                odom_broadcaster.sendTransform(odom_trans);

                // next, we'll publish the odometry message over ROS
                nav_msgs::Odometry odom;
                odom.header.stamp = currentTime;
                odom.header.frame_id = "odom";
                // set the position
                odom.pose.pose.position.x = simPose[1];
                odom.pose.pose.position.y = -simPose[0];
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = odom_quat;
                // set the velocity
                odom.child_frame_id = "base_footprint";
                odom.twist.twist.linear.x = car.getVelocity()*std::cos(simPose[2]);
                odom.twist.twist.linear.y = car.getVelocity()*std::sin(simPose[2]);
                odom.twist.twist.angular.z = car.getAngularVelocity();

                // Send Telemetry
                telemetry.header.stamp = currentTime;
                telemetry.steering = car.getSteering();
                //telemetry.speed = car.getSpeed();
                telemetry.steering_angle = car.getSteeringAngle();
                telemetry.radial_distance = car.getDistance();
                telemetry.v_radial = car.getVelocity();
                telemetry.v_linear.x=car.getVelocity()*std::cos(simPose[2]);
                telemetry.v_linear.y=car.getVelocity()*std::sin(simPose[2]);
                telemetry.v_angular.z=car.getAngularVelocity();

                // publish the messages
                odomPub.publish(odom);
                telemetryPub.publish(telemetry);

                ros::spinOnce();
                loop_rate.sleep();
        }

        ros::spin();
}
