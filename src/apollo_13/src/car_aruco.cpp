/*
 * car_aruco.cpp
 *
 *      Authors: Sebastian Ehmes
 *         Nicolas Acero
 *         Huynh-Tan Truong
 *         Li Zhao
 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <list>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <exception>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//#include <costmap_2d/costmap_2d.h>
//#include <costmap_2d/costmap_2d_ros.h>

// used for sending navigation goals to sbpl motion planner
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// container for detected aruco marker
std::list<geometry_msgs::TransformStamped> discoveredMarkers;
std::list<geometry_msgs::TransformStamped> markerDump;
std::list<visualization_msgs::Marker> publishedMarkers;
// car position variable
geometry_msgs::Pose carTransform;
// algorithm on/off
int aruco_control = 0;

/*  Callback for the topic on which positions of aruco markers are published.
 *  Markers have to be unique, so duplictes are discarded.
 */
void cameraInfo(const geometry_msgs::TransformStamped::ConstPtr& marker)
{
    bool duplicate = false;
    for(auto current : discoveredMarkers){
      if (marker->child_frame_id.compare(current.child_frame_id)==0){
        duplicate = true;
      }
    }
    if(!duplicate){
      try{
        discoveredMarkers.push_back(*marker);
        markerDump.push_back(*marker);
      }catch(std::exception& e){
      }
    }
}
/*  Callback for the algorithm control signal, published by apollo_13 node
*/
void aruCTRL(const std_msgs::Int32::ConstPtr& ctrl)
{
  aruco_control = ctrl->data;
}
/*  This method sends a navigational goal, to the sbpl planner. It needs the
 *  position and the orientation of the goal and a reference to MoveBaseClient
 *  as parameters.
 */
void sendGoal(geometry_msgs::Point goal_position, geometry_msgs::Quaternion goal_orientation, MoveBaseClient& ac)
{
	move_base_msgs::MoveBaseGoal goal;
	//we'll send a goal to the robot to move to the next Aruco gate
  	goal.target_pose.header.frame_id = "/map";
  	goal.target_pose.header.stamp = ros::Time::now();
	  goal.target_pose.pose.position = goal_position;
  	goal.target_pose.pose.orientation = goal_orientation;

  	ROS_INFO("Sending goal");
  	ac.sendGoal(goal);
}
/*  This function calculates coordinates of the center between two aruco
 *  aruco markers. It needs the postion of both markers in world coordinates.
*/
geometry_msgs::Point calcGoalCenterPosition(geometry_msgs::Point& aruco1, geometry_msgs::Point& aruco2){
  geometry_msgs::Point goal;
  goal.x = 0.5*(aruco1.x+aruco2.x);
  goal.y = 0.5*(aruco1.y+aruco2.y);
  goal.z = 0;
  std::cout<<"Distance to Goal: "<<std::sqrt(goal.x*goal.x+goal.y*goal.y)<<std::endl;
  return goal;
}
/*  This function calculates the orientation as yaw angle of a possible goal.
 *  It needs the orientation of both aruco markers as quaternions.
 *  -> currently unsed
 */
double calcGoalOrientantionYaw(geometry_msgs::Quaternion& poseAruco1, geometry_msgs::Quaternion& poseAruco2){
  tf::Quaternion q;
  double roll1, pitch1, yaw1;
  double roll2, pitch2, yaw2;
  tf::quaternionMsgToTF(poseAruco1, q);
  tf::Matrix3x3(q).getRPY(roll1, pitch1, yaw1);
  tf::quaternionMsgToTF(poseAruco2, q);
  tf::Matrix3x3(q).getRPY(roll2, pitch2, yaw2);
  double yawAruco1 = yaw1;
  double yawAruco2 = yaw2;
  double midYaw = 0.5*(yawAruco1+yawAruco2);
  double result = (midYaw>=0)?midYaw-3.141:midYaw+3.141;
  std::cout<<"yawMarker1: "<<yawAruco1*180/3.141<<" yawMarker2: "<<yawAruco2*180/3.141<<" yawGoal: "<<result*180/3.141<<std::endl;
  return result;
}
/*  Calculates the distance between two aruco markers which may form a goal.
*/
double distanceBetweenGoals(geometry_msgs::Point& aruco1, geometry_msgs::Point& aruco2){
  geometry_msgs::Point aTob;
  aTob.x = aruco2.x-aruco1.x;
  aTob.y = aruco2.y-aruco1.y;
  return std::sqrt(aTob.x*aTob.x+aTob.y*aTob.y);
}
/*  Returns the angle two aruco markers have with respect to the car.
 *  If this angle is bigger than 180 degrees, the car already passed the aruco markers.
 */
double calcGoalAperture(geometry_msgs::Point& aruco1, geometry_msgs::Point& aruco2){
  geometry_msgs::Point a1, a2;
  a1.x = aruco1.x-carTransform.position.x;
  a1.y = aruco1.y-carTransform.position.y;
  a1.z = 0;
  a2.x = aruco2.x-carTransform.position.x;
  a2.y = aruco2.y-carTransform.position.y;
  a2.z = 0;

  double yaw1 = std::atan2(a1.y, a1.x);
  double yaw2 = std::atan2(a2.y, a2.x);
  return std::fabs(yaw1-yaw2)*180/3.141;
}
/*  This functor is needed to compare two markers in a list, with the distance
 *  to the car as criteria. If a list is sorted with this functor, it will be
 *  in ascending order.
*/
bool compareDistanceToCar(visualization_msgs::Marker& marker1, visualization_msgs::Marker& marker2){
  geometry_msgs::Point aruco1 = marker1.pose.position;
  geometry_msgs::Point aruco2 = marker2.pose.position;
  geometry_msgs::Point a1, a2;
  a1.x = aruco1.x-carTransform.position.x;
  a1.y = aruco1.y-carTransform.position.y;
  a1.z = 0;
  a2.x = aruco2.x-carTransform.position.x;
  a2.y = aruco2.y-carTransform.position.y;
  a2.z = 0;

  double a1_dist = std::sqrt(a1.x*a1.x+a1.y*a1.y);
  double a2_dist = std::sqrt(a2.x*a2.x+a2.y*a2.y);

  return (a1_dist<a2_dist)?true:false;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "car_aruco");

  ros::NodeHandle n;
  //  this object is needed to gain information about the current position of the car
  tf::TransformListener listener;
  //  subscribe to the aruco marker topic published by ar_sys
  ros::Subscriber camera_info = n.subscribe<geometry_msgs::TransformStamped>("ar_multi_boards/transform", 50, cameraInfo);
  //  subscribe to the control flag from the apollo_13 node
  ros::Subscriber aruco_active = n.subscribe<std_msgs::Int32>("apollo_13/aruco_active", 50, aruCTRL);
  //  publish discovered aruco marker to rviz
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("car_aruco/rviz_marker", 50);

  //  the spatial dimensions of a aruco marker displayed in rviz
  geometry_msgs::Vector3 scale;
  scale.x = 0.18;
  scale.y = 0.18;
  scale.z = 0.18;

  //  tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //  a flag to prevent the goal finding algorithm to spam solutions
  bool goal_flag = true;

  //  connect to the move_base server
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  //  The loop does not need to be 20hz, but this seemed to work fine
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    //  prevents useless calculations, when there is nothing to be done
    if(discoveredMarkers.size()>0){
      //  publish new markers only when it has not been discovered before
      while(markerDump.size()>0){

        geometry_msgs::TransformStamped currentMarker = markerDump.front();
        markerDump.pop_front();
        //  instantiate a new visualization marker for rviz
        visualization_msgs::Marker rvizMarker;
        rvizMarker.type = visualization_msgs::Marker::CUBE;
        rvizMarker.type = visualization_msgs::Marker::ADD;
        rvizMarker.id = std::stoi(currentMarker.child_frame_id);
        //  due to differences between kamera coordinates and base coordinates
        //  axis must be swapped. x_car = z_camera, y_car = - x_camera, z_car = - y_camera
        rvizMarker.pose.position.x= currentMarker.transform.translation.z;
        rvizMarker.pose.position.y= -currentMarker.transform.translation.x;
        rvizMarker.pose.position.z= -currentMarker.transform.translation.y;
        //  The next 5 lines only exist to fix a mismatch between two types
        //  which carry exactly the same information.
        tf::Quaternion q;
        double roll, pitch, yaw;
        tf::quaternionMsgToTF(currentMarker.transform.rotation, q);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double rollOffset, pitchOffset, yawOffset;
        //  As mentioned above, there is a coordinate mismatch, that means rotational
        //  axis also needs to be swapped. roll_car = yaw_camera, pitch_car = pitch_camera,
        //  yaw_car = roll_camera.
        rvizMarker.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(yaw, pitch, roll);
        rvizMarker.color.a = 1.0;
        rvizMarker.color.r = 0.0;
        rvizMarker.color.g = 1.0;
        rvizMarker.color.b = 0.0;
        rvizMarker.scale = scale;
        rvizMarker.header=currentMarker.header;
        //  The frame_id is very important, it tells TF how to transform position and orientation.
        rvizMarker.header.frame_id="base_laser";
        //  The next section transforms coordinates of a marker from car to world map
        //  otherwise calculations are not placed correctly.
        geometry_msgs::PoseStamped tmp_transform;
        geometry_msgs::PoseStamped marker_transform;
        tmp_transform.pose = rvizMarker.pose;
        tmp_transform.header = rvizMarker.header;
        listener.transformPose("map", ros::Time(0), tmp_transform, "base_laser", marker_transform);
        rvizMarker.pose = marker_transform.pose;
        rvizMarker.header = marker_transform.header;
        rvizMarker.header.frame_id = "map";
        //  Publish the marker and promt succes to console
        publishedMarkers.push_back(rvizMarker);
        marker_pub.publish(rvizMarker);
        std::cout << rvizMarker.header.frame_id << std::endl;
        std::cout<<"Marker x: "<<rvizMarker.pose.position.z<<"Marker y: "<<-rvizMarker.pose.position.x<<"Marker_amount: "<<publishedMarkers.size()<<std::endl;

      }

    }
    //  only try to find a path through goals if the control flag from apollo_13 is set
    if(aruco_control != 0){
      //  rest when only one marker is deteced
      if(publishedMarkers.size()>1){
        //  Get Position of Car!
        try{
          tf::StampedTransform stf;
          listener.lookupTransform("/base_link", "/map", ros::Time(0), stf);
          tf::Stamped<tf::Pose> tmp(stf, stf.stamp_, "/map");
          geometry_msgs::PoseStamped tmp2;
          tf::poseStampedTFToMsg(tmp,tmp2);
          carTransform = tmp2.pose;

        }catch(tf::TransformException ex){
          ROS_ERROR("%s", ex.what());
        }
        //  start if previous goal has been reached or not goal is set
        if(goal_flag) {
          //  sort all markers by distance to car in ascending order
          publishedMarkers.sort(compareDistanceToCar);
          //  this is precaution to prevent infinite loops
          int maxIterations  = publishedMarkers.size()*publishedMarkers.size();
          //  extract first marker and start
          visualization_msgs::Marker temp1 = publishedMarkers.front();
          publishedMarkers.pop_front();
          while(publishedMarkers.size()>0 && maxIterations > 0){
            //  extract second marker
            visualization_msgs::Marker temp2 = publishedMarkers.front();
            publishedMarkers.pop_front();

            double alpha = calcGoalAperture(temp1.pose.position, temp2.pose.position);
            double d = distanceBetweenGoals(temp1.pose.position, temp2.pose.position);
            std::cout<<"Markerabstand: "<<d<<" Apertur: "<<alpha<<std::endl;
            //  the goal needs to be in front of the car and the car has to fit between two markers
            if(alpha<=180 && d>=0.3){
              //  This marker is just used as a container for position and orientation information
              visualization_msgs::Marker rvizMarker;
              //  We assume the center of two appropriate aruco markers as goal position.
              rvizMarker.pose.position = calcGoalCenterPosition(temp1.pose.position, temp2.pose.position);
              //  rvizMarker.pose.orientation= tf::createQuaternionMsgFromYaw(calcGoalOrientantionYaw(temp1.pose.orientation, temp2.pose.orientation));
              //  The car drives through the goal with the same orientation it had upon discovery of the goal.
              rvizMarker.pose.orientation = carTransform.orientation;
              //  Broadcast the new found goal
              sendGoal(rvizMarker.pose.position, rvizMarker.pose.orientation, ac);
              //  prevent "spamming" of new goal solutions
              goal_flag=false;
              //  prevent segmentation fault
              if(publishedMarkers.size()>0){
                temp1 = publishedMarkers.front();
                publishedMarkers.pop_front();
              }
              //  we do not need more than one goal at once
              break;
          }else{
            //  insert the unfitting marker at the back
            publishedMarkers.push_back(temp2);
          }
          maxIterations--;
        }
      }
      //  check if goal has been reached
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the auto moved to the next Aruco gate");
        goal_flag=true;
      }

      }

      ros::spinOnce();
    }
    else{
      ros::spinOnce();
    }

    loop_rate.sleep();
  }


  return 0;
}
