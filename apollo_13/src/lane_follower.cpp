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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

cv_bridge::CvImageConstPtr currentFrame_ptr;
MoveBaseClient* ac;

void readPointCloud(const sensor_msgs::ImageConstPtr& pointCloud)
{

  try
  {
    currentFrame_ptr = cv_bridge::toCvShare(pointCloud, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

}

void vanishingPointCB(const geometry_msgs::PointStampedConstPtr& vp) {
  if(currentFrame_ptr->image.data != NULL) {
    float x = currentFrame_ptr->image.at<float>(vp->point.x, vp->point.y);
    float y = cvRound(currentFrame_ptr->image.cols/2 - vp->point.x);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = std::atan(x/y);
    ac->sendGoal(goal);
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
    image_transport::ImageTransport it(nh);


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

     image_transport::Subscriber pointCloud_sub = it.subscribe("camera/depth/image_raw", 1, readPointCloud);
     ros::Subscriber vanishingPoint_sub = nh.subscribe("lane_detector/vanishing_point", 1, vanishingPointCB);
     ac = new MoveBaseClient("move_base", true);

     //wait for the action server to come up
     while(!ac->waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting for the move_base action server to come up");
     }

     ros::spin();


    return 0;
}
