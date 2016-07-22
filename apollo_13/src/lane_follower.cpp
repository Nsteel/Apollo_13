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


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//cv_bridge::CvImageConstPtr currentFrame_ptr;
MoveBaseClient* ac;
ros::Publisher obstacles_pub;
ros::Publisher path_pub;

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

std::vector<geometry_msgs::Point32> splineSampling(const std::vector<geometry_msgs::Point32>& spline) {

  assert(spline.size() >= 4);
  geometry_msgs::Point32 p1 = spline[0];
  geometry_msgs::Point32 p2 = spline[(spline.size()-1)*0.3333];
  geometry_msgs::Point32 p3 = spline[(spline.size()-1)*0.6666];
  geometry_msgs::Point32 p4 = spline[(spline.size()-1)];
  std::vector<geometry_msgs::Point32> output {p1, p2, p3, p4};

  return output;
}

void sendGoal(nav_msgs::Path& p)
{
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = p.poses[p.poses.size()-1];
        //ROS_INFO("Sending goal for x:%lf / y:%lf",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y);
        ac->sendGoal(goal);
        //ac.waitForResult();
}

void laneCB(const lane_detector::Lane::ConstPtr& lane) {

  /*teb_local_planner::ObstacleMsg obstacle_msg;
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
  obstacles_pub.publish(obstacle_msg);*/

    //float w = std::atan2(y,x);
    nav_msgs::Path path;
    //float w = std::atan2(detectedPoints->polygon.points[0].y, detectedPoints->polygon.points[0].x);
    path.header.frame_id = "base_link";
    path.header.stamp = ros::Time::now();

    std::vector<geometry_msgs::PoseStamped> pVector;

    if(lane->guide_line.size() >= 4) {
      std::vector<geometry_msgs::Point32> sampled_guide_line = splineSampling(lane->guide_line);
      for(int i = 0; i< sampled_guide_line.size(); i++) {
              geometry_msgs::PoseStamped new_goal;

              new_goal.header = path.header;
              double yaw = std::atan2(sampled_guide_line[i].y, sampled_guide_line[i].x);
              //std::cout << 180/CV_PI*yaw << std::endl;
              tf::Quaternion goal_quat = tf::createQuaternionFromYaw(yaw);

              new_goal.pose.position.x = sampled_guide_line[i].x;
              new_goal.pose.position.y = sampled_guide_line[i].y;

              new_goal.pose.orientation.x = goal_quat.x();
              new_goal.pose.orientation.y = goal_quat.y();
              new_goal.pose.orientation.z = goal_quat.z();
              new_goal.pose.orientation.w = goal_quat.w();
              pVector.push_back(new_goal);
      }

      path.poses = pVector;
      path_pub.publish(path);
      ros::spinOnce();
      sendGoal(path);
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
     ros::Subscriber lane_sub = nh.subscribe<lane_detector::Lane>("/lane_detector/lane", 1, laneCB);
     obstacles_pub = nh.advertise<teb_local_planner::ObstacleMsg>("/obstacles", 10);
     path_pub = nh.advertise<nav_msgs::Path>("/pathtransformPlanner/path", 10);
     ac = new MoveBaseClient("move_base", true);

     //wait for the action server to come up
     while(!ac->waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting for the move_base action server to come up");
     }

     ros::spin();

     delete ac;


    return 0;
}
