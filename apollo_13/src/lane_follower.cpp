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


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
#define FOCAL_LENGTH 335.906

//cv_bridge::CvImageConstPtr currentFrame_ptr;
MoveBaseClient* ac;
ros::Publisher obstacles_pub;

/*{

  try
  {
    currentFrame_ptr = cv_bridge::toCvShare(pointCloud, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

}*/

void detectedPointsCB(const geometry_msgs::PolygonStampedConstPtr& detectedPoints) {

  //Vanishing Point with real world coordinates in meters
  float vp_x = 1.0; //Asumming the vanishing point to be one meter in front of the car
  float vp_y = vp_x*cvRound(640/2 - detectedPoints->polygon.points[0].x)/FOCAL_LENGTH;

  teb_local_planner::ObstacleMsg obstacle_msg;
  obstacle_msg.header.stamp = ros::Time::now();
  obstacle_msg.header.frame_id = "base_footprint";
  geometry_msgs::Point32 line_start;
  geometry_msgs::Point32 line_end;

  //Left lane added as obstacle
  obstacle_msg.obstacles.push_back(geometry_msgs::PolygonStamped());
  line_start.x = 0;
  line_start.y = 0.19; //detectedPoints->polygon.points[1].y;
  line_end.x = vp_x;
  line_end.y = vp_y;
  obstacle_msg.obstacles[0].polygon.points = {line_start, line_end};

  //right lane added as obstacle
  obstacle_msg.obstacles.push_back(geometry_msgs::PolygonStamped());
  line_start.x = 0;
  line_start.y = -0.19;//detectedPoints->polygon.points[2].y;
  line_end.x = vp_x;
  line_end.y = vp_y;
  obstacle_msg.obstacles[1].polygon.points = {line_start, line_end};
  obstacles_pub.publish(obstacle_msg);

  if(ac->getState() != actionlib::SimpleClientGoalState::ACTIVE) {
    ROS_INFO("State: %s",ac->getState().toString().c_str());
    //float w = std::atan2(y,x);
    float w = std::atan2(detectedPoints->polygon.points[0].y, detectedPoints->polygon.points[0].x);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = vp_x;
    goal.target_pose.pose.position.y = vp_y;
    goal.target_pose.pose.orientation.w = w;
    ROS_INFO("x:%f, y:%f, w:%f", vp_x, vp_y, 180*w/CV_PI);
    ac->sendGoal(goal);
  }
  ROS_INFO("State: %s",ac->getState().toString().c_str());
}


int main(int argc, char **argv){

    ros::init(argc, argv, "lane_follower");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle nh;
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

     //image_transport::Subscriber pointCloud_sub = it.subscribe("camera/depth/image_raw", 1, readPointCloud);
     ros::Subscriber detectedPoints_sub = nh.subscribe("lane_detector/vanishing_point", 1, detectedPointsCB);
     obstacles_pub = nh.advertise<teb_local_planner::ObstacleMsg>("/obstacles", 1);
     ac = new MoveBaseClient("move_base", true);

     //wait for the action server to come up
     while(!ac->waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting for the move_base action server to come up");
     }

     ros::spin();


    return 0;
}
