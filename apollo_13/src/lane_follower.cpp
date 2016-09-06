/*
 * lane_follower.cpp
 *
 *      Author:
 *         Nicolas Acero
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cv.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PolygonStamped.h>
#include <teb_local_planner/ObstacleMsg.h>
#include <lane_detector/Lane.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include<std_msgs/Int32.h>
#include<limits>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher obstacles_pub;
ros::Publisher path_pub;
std_msgs::Int32 steering_msg;
std_msgs::Int32 motor_msg;

const double angleArray[101]={25.0, 24.6, 24.2, 23.8, 23.4, 23.0, 22.6, 22.2, 21.8, 21.4,
						21.0, 20.6, 20.2, 19.8, 19.4, 19.0, 18.6, 18.2, 17.8, 17.4,
						17.0, 16.5, 16.0, 15.5, 15.0, 14.5, 14.0, 13.5, 13.0, 12.5,
						12.0, 11.7, 11.4, 11.1, 10.8, 10.5, 10.2, 9.9, 9.6, 9.3,
						9.0, 8.7, 8.4, 8.1, 7.8, 7.5, 7.2, 6.9, 6.6, 6.3,
						6.0, 5.2, 4.4, 3.6, 2.8, 2.0, 1.2, 0.4, -0.4, -1.2,
						-2.0, -2.3, -2.6, -2.9, -3.2, -3.5, -3.8, -4.1, -4.4, -4.7,
						-5.0, -5.4, -5.8, -6.2, -6.6, -7.0, -7.4, -7.8, -8.2, -8.6,
						-9.0, -9.4, -9.8, -10.2, -10.6, -11.0, -11.4, -11.8, -12.2, -12.6,
						-13.0, -13.5, -14.0, -14.5, -15.0, -15.5, -16.0, -16.5, -17.0, -17.5, -18.0};

/* Determines the appropriate steering level to a given steering angle, using
 * the lookup table from above.
 */
int angleToSteering(double alpha){
    if(alpha>=angleArray[0]){
      return -50;
    }else if(alpha<=angleArray[100]){
      return 50;
    }else{
      for(int i = 1; i <= 100; i++){
        if(alpha < angleArray[i-1] && alpha >= angleArray[i]){
          return i-50;
        }
      }
    }
}


std::vector<geometry_msgs::Point32> splineSampling(const std::vector<geometry_msgs::Point32>& spline) {

  assert(spline.size() >= 4);
  geometry_msgs::Point32 p1 = spline[0];
  geometry_msgs::Point32 p2 = spline[(spline.size()-1)*0.3333];
  geometry_msgs::Point32 p3 = spline[(spline.size()-1)*0.6666];
  geometry_msgs::Point32 p4 = spline[(spline.size()-1)];
  std::vector<geometry_msgs::Point32> output {p1, p2, p3, p4};

  return output;
}

void sendGoal(nav_msgs::Path& p, MoveBaseClient* ac)
{
      //if(ac->getState() != actionlib::SimpleClientGoalState::ACTIVE) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = p.poses.back();
        double yaw_final = std::atan2(p.poses.back().pose.position.y - (*(p.poses.end()-2)).pose.position.y, p.poses.back().pose.position.x - (*(p.poses.end()-2)).pose.position.x);
        double yaw_start = std::atan2((*(p.poses.begin()+1)).pose.position.y - (*(p.poses.begin())).pose.position.y, (*(p.poses.begin()+1)).pose.position.y - (*(p.poses.begin())).pose.position.y);
        tf::Quaternion goal_quat = tf::createQuaternionFromYaw(yaw_final);
        tf::Quaternion start_quat = tf::createQuaternionFromYaw(yaw_start);

        p.poses.front().pose.orientation.x = start_quat.x();
        p.poses.front().pose.orientation.y = start_quat.y();
        p.poses.front().pose.orientation.z = start_quat.z();
        p.poses.front().pose.orientation.w = start_quat.w();

        goal.target_pose.pose.orientation.x = goal_quat.x();
        goal.target_pose.pose.orientation.y = goal_quat.y();
        goal.target_pose.pose.orientation.z = goal_quat.z();
        goal.target_pose.pose.orientation.w = goal_quat.w();
        ROS_INFO("Sending goal for x:%lf / y:%lf / start_yaw:%lf / end_yaw:%lf",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y, yaw_start, yaw_final);
        ac->sendGoal(goal);
        //ac.waitForResult();
      //}
}

void laneCB(const lane_detector::Lane::ConstPtr& lane, tf::TransformListener* listener, MoveBaseClient* ac) {

  teb_local_planner::ObstacleMsg obstacle_msg;
  obstacle_msg.header.stamp = ros::Time::now();
  obstacle_msg.header.frame_id = "base_laser";

  //Left lane added as obstacle
  obstacle_msg.obstacles.push_back(geometry_msgs::PolygonStamped());

  obstacle_msg.obstacles[0].polygon.points = lane->left_line;

  //right lane added as obstacle
  obstacle_msg.obstacles.push_back(geometry_msgs::PolygonStamped());
  obstacle_msg.obstacles[1].polygon.points = lane->right_line;

  //obstacles_pub.publish(obstacle_msg);

    //float w = std::atan2(y,x);
    nav_msgs::Path path;
    //float w = std::atan2(detectedPoints->polygon.points[0].y, detectedPoints->polygon.points[0].x);
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    std::vector<geometry_msgs::PoseStamped> pVector;

    if(lane->guide_line.size() >= 4) {
      std::vector<geometry_msgs::Point32> sampled_guide_line = splineSampling(lane->guide_line);

      tf::StampedTransform transform;
      try{
        listener->lookupTransform("/map", "/base_laser",
                                 ros::Time(0), transform);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

      for(int i = 0; i< sampled_guide_line.size(); i++) {
              geometry_msgs::PoseStamped new_goal;

              new_goal.header = path.header;

              new_goal.pose.position.x = transform.getOrigin().x() + sampled_guide_line[i].x;
              new_goal.pose.position.y = transform.getOrigin().y() + sampled_guide_line[i].y;

              double yaw = std::atan2(transform.getOrigin().y() + sampled_guide_line[i].y, transform.getOrigin().x() + sampled_guide_line[i].x);
              //std::cout << 180/CV_PI*yaw << std::endl;
              //std::cout << "x: " << transform.getOrigin().x() << " y: " << transform.getOrigin().y() << " Yaw: " << 180/CV_PI*yaw << std::endl;
              tf::Quaternion goal_quat = tf::createQuaternionFromYaw(yaw);

              new_goal.pose.orientation.x = goal_quat.x();
              new_goal.pose.orientation.y = goal_quat.y();
              new_goal.pose.orientation.z = goal_quat.z();
              new_goal.pose.orientation.w = goal_quat.w();
              pVector.push_back(new_goal);
      }

      path.poses = pVector;
      path_pub.publish(path);
      ros::spinOnce();
      sendGoal(path, ac);
  }
}

void laneCB2(const lane_detector::Lane::ConstPtr& lane) {
  if(lane->guide_line.size() >= 4) {
    double yaw = std::atan2(lane->guide_line.back().y, lane->guide_line.back().x) * 180/CV_PI;
    int steering = angleToSteering(yaw);
    steering_msg.data = steering;
    ROS_DEBUG("Steering Angle: %i", steering);
  }
}

void laneCB3(const lane_detector::Lane::ConstPtr& lane) {
  if(lane->guide_line.size() >= 4) {
		int index = (lane->guide_line.size()-1)*0.5;
		double x_mid = lane->guide_line[index].x;
		double y_mid = lane->guide_line[index].y;
		index = (lane->guide_line.size()-1)*0.3333;
		double x1 = lane->guide_line[index].x;
		double y1 = lane->guide_line[index].y;
		index = (lane->guide_line.size()-1)*0.6666;
		double x2 = lane->guide_line[index].x;
		double y2 = lane->guide_line[index].y;

		double dx = (x2 - x1)/(y2 - y1 + std::numeric_limits<double>::epsilon());
		double dy = (y2 - y1)/(x2 - x1 + std::numeric_limits<double>::epsilon());
		double tmp = y2 - y_mid;
		double ddx = (x2 - (2.0*x_mid) + x1)/(tmp*tmp + std::numeric_limits<double>::epsilon());
		tmp = x2 - x_mid;
		double ddy = (y2 - (2.0*y_mid) + y1)/(tmp*tmp + std::numeric_limits<double>::epsilon());
		double num = std::pow(1.0 + dx*dx, 1.5);
		double den = ddx;
		double r = num/(den + std::numeric_limits<double>::epsilon());

		double yaw = std::atan2(0.25, std::sqrt(r*r - 0.25))*180/CV_PI;
		int steering = angleToSteering(yaw);

    std::cout << "dx: " << dx << " dy: " << dy << " r: " << r << " alpha: " << yaw << " steering: " << steering << std::endl;
  }
}


int main(int argc, char **argv){

    ros::init(argc, argv, "lane_follower");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle nh;

    tf::TransformListener listener;
    //image_transport::ImageTransport it(nh);


		/**
	   * The advertise() function is how you tell ROS that you want to
	   * publish on a given topic name. This invokes a call to the ROS
	   * master node, which keeps a registry of who is publishing and who
	   * is subscribing. After this advertise() call is made, the master
	   * node will notify anyone who is trying to subscribe to this topic name,
	   * and they will in turn negotiate a peer-to-peer connection with this
	   * node.  advertise() returns a Publisher object which allows you to
	   * publish messages on that topic through a call to publish().  Once
	   * all copies of the returned Publisher object are destroyed, the topic
	   * will be automatically unadvertised.
	   *
	   * The second parameter to advertise() is the size of the message queue
	   * used for publishing messages.  If messages are published more quickly
	   * than we can send them, the number here specifies how many messages to
	   * buffer up before throwing some away.
	   */

     //MoveBaseClient ac("move_base", true);

     //image_transport::Subscriber pointCloud_sub = it.subscribe("camera/depth/image_raw", 1, readPointCloud);
     //ros::Subscriber lane_sub = nh.subscribe<lane_detector::Lane>("/lane_detector/lane", 1, std::bind(laneCB, std::placeholders::_1, &listener, &ac));
     ros::Subscriber lane_sub = nh.subscribe<lane_detector::Lane>("/lane_detector/lane", 1, std::bind(laneCB2, std::placeholders::_1));
		 //ros::Subscriber lane_sub = nh.subscribe<lane_detector::Lane>("/lane_detector/lane", 1, std::bind(laneCB3, std::placeholders::_1));
     obstacles_pub = nh.advertise<teb_local_planner::ObstacleMsg>("/move_base/TebLocalPlannerROS/obstacles", 10);
     path_pub = nh.advertise<nav_msgs::Path>("/pathtransformPlanner/path", 10);
     ros::Publisher motorControl_pub = nh.advertise<std_msgs::Int32>("car_handler/motor", 10);
     ros::Publisher steeringControl_pub = nh.advertise<std_msgs::Int32>("car_handler/steering", 10);
     //wait for the action server to come up
     /*while(!ac.waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting for the move_base action server to come up");
     }*/

     ros::Rate loop_rate(20);
     while(ros::ok()) {
        motor_msg.data = 3;
        motorControl_pub.publish(motor_msg);
        steeringControl_pub.publish(steering_msg);
        ros::spinOnce();
        loop_rate.sleep();
     }

    return 0;
}
