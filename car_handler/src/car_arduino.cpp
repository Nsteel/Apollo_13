/*
 * car_arduino.cpp
 *
 *      Authors: Sebastian Ehmes
 *				 Nicolas Acero
 *				 Huynh-Tan Truong
 *				 Li Zhao
 */

#include <iostream>
#include <car_handler/cArduino.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <car_handler/Speedometer.h>
#include <car_handler/ForwardKinematics.h>
#include <sstream>
#include <vector>
#include <exception>

/* The following variables are used to determine the acceleration, speed, position,
 * and orientation of the car within the world map.
 */
float th = 0.0;
float vth = 0.0;
float dist = 0.0;
const double NSEC_PER_SEC = 1000000000;
geometry_msgs::Vector3 acc;
geometry_msgs::Vector3 angular_vel;
Speedometer distanceAndSpeed;

// This Object is used to communicate wih the arduino microcontroller
cArduino arduino(ArduinoBaundRate::B115200bps);

/* The following variables are car specific and used to calculate steering levels
 * from steering angles provided by the sbpl planner.
 */
std::vector<double> ctrlData;
double angleArray[101]={25.0, 24.6, 24.2, 23.8, 23.4, 23.0, 22.6, 22.2, 21.8, 21.4,
						21.0, 20.6, 20.2, 19.8, 19.4, 19.0, 18.6, 18.2, 17.8, 17.4,
						17.0, 16.5, 16.0, 15.5, 15.0, 14.5, 14.0, 13.5, 13.0, 12.5,
						12.0, 11.7, 11.4, 11.1, 10.8, 10.5, 10.2, 9.9, 9.6, 9.3,
						9.0, 8.7, 8.4, 8.1, 7.8, 7.5, 7.2, 6.9, 6.6, 6.3,
						6.0, 5.2, 4.4, 3.6, 2.8, 2.0, 1.2, 0.4, -0.4, -1.2,
						-2.0, -2.3, -2.6, -2.9, -3.2, -3.5, -3.8, -4.1, -4.4, -4.7,
						-5.0, -5.4, -5.8, -6.2, -6.6, -7.0, -7.4, -7.8, -8.2, -8.6,
						-9.0, -9.4, -9.8, -10.2, -10.6, -11.0, -11.4, -11.8, -12.2, -12.6,
						-13.0, -13.5, -14.0, -14.5, -15.0, -15.5, -16.0, -16.5, -17.0, -17.5, -18.0};

// The callback for the sbpl planner data, used to steer the car
void plannerCommands(const geometry_msgs::Twist::ConstPtr& plannerCMD)
{
  ctrlData.clear();
  ctrlData.push_back(plannerCMD->linear.x);
  ctrlData.push_back(plannerCMD->angular.z);
}
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
/* Gets a set of commands (sbpl motion planner) from the ctrlData container
 * and determines steering levels und motor levels.
 */
void setMotorAndSteering(std_msgs::Int32& motorLevel, std_msgs::Int32& steeringLevel){
	if(ctrlData.size()>1){

		if(ctrlData[0]>0){
			if(ctrlData[0] < 0.15) {
	    	motorLevel.data = 1;
			}
			else if(ctrlData[0] < 0.28) {
				motorLevel.data = 2;
			}
			// rotary encoder is broken, do not go higher than level 3 or the odometry will be off (Max speed: 0.35 m/s)
			else motorLevel.data = 3;
	  }
		else if(ctrlData[0] < 0){
			if(ctrlData[0] > -0.15)
				motorLevel.data = -3;
			// rotary encoder is broken, do not go lower than level -4 or the odometry will be off (Max speed: -0.35 m/s)
			else motorLevel.data = -4;
	  }
		else {
			motorLevel.data = 0;
		}
	  steeringLevel.data = angleToSteering(ctrlData[1]*180/3.1415);
	}else{
		motorLevel.data = 0;
		steeringLevel.data = 0;
	}

}
/* This method interpretes a data string from the arduino,
 * extracts the correct data fields and stores them in global variables.
 */
void readString(std::string input){

			std::vector<float> temp;
			std::string::iterator it = input.begin();
			std::string result;

			for(;it!=input.end(); ++it){
				// '*' is a speparator between data fields
				if((*it)!='*'){
					result.push_back(*it);
				}else{
					temp.push_back(std::atof(result.c_str()));
					result = "";
				}
			}
			if(temp[0]+temp[1]+temp[2]+temp[3]==temp[4]){
				dist = temp[0];
				vth = temp[1];
				th = temp[2];
			}

}
/* getData tries to fetch a data stream from the arduino,
 * calls readString with the string representation of the arduino data,
 * which then writes the extracted data fields in global variables.
 */
void getData() {
	//response
	std::string arduinoOutput;

	if(!arduino.read(arduinoOutput))//read which timeout!
	{
		//DO NOTHING

	}else{
		//response
		if(arduinoOutput.compare(std::string("\n")) != 0 && sizeof(arduinoOutput)!=0){
			readString(arduinoOutput);
			angular_vel.z = vth;
		}
	}
}
// create a timeStamp needed for the Speedometer object
double timeStamp(ros::Time T){
		return (double)T.sec + ((double)T.nsec)/NSEC_PER_SEC;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "car_arduino");

  ros::NodeHandle n;
	// Publishes the results of the odometry calculations to other ros nodes
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	// Publishes an interpretation of the sbpl planner commands,
	// which can be used by apollo_13 node to instruct car_handler what to do.
	ros::Publisher sbplMotor_pub = n.advertise<std_msgs::Int32>("car_arduino/motor", 50);
	ros::Publisher sbplSteering_pub = n.advertise<std_msgs::Int32>("car_arduino/steering", 50);
	// Here we subscribe to the sbpl planner to receive steering commands,
	// when the planner has found a solution for a certain goal.
	ros::Subscriber planner = n.subscribe<geometry_msgs::Twist>("cmd_vel", 50, plannerCommands);

	//This node needs to publish with at least 20hz, or the navigational stack will complain
	ros::Rate loop_rate(45);
	// object needed to send odometric information to the navigational stack
	tf::TransformBroadcaster odom_broadcaster;
	// car control values to be published
	std_msgs::Int32 motorValue, steeringValue;

	// set initial angular velocity to 0
	angular_vel.x = 0.0;
	angular_vel.y = 0.0;

	// timer variables
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	// objects needed for odometric calculations
	ForwardKinematics odometric(0.25);
	distanceAndSpeed = Speedometer(timeStamp(current_time));
	std::vector<double> metrics = std::vector<double>(5);
	std::vector<double> odometrics = std::vector<double>(3);

	// open connection to the arduino microcontroller
	if(!arduino.isOpen())
	{
		std::cerr<<"can't open arduino"<<std::endl;
	}
	std::cout<<"arduino open at "<<arduino.getDeviceName()<<std::endl;

	while(ros::ok()) {
		// get sensor information from the arduino
		getData();
		current_time = ros::Time::now();
		// update odometric information
		metrics = distanceAndSpeed.updateArduino(dist, timeStamp(current_time));
		odometrics = odometric.getUpdateWithGyro(th, metrics[3]);

    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

		odom_trans.transform.translation.x = odometrics[1];
    odom_trans.transform.translation.y = -odometrics[0];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);


    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // set the position
    odom.pose.pose.position.x = odometrics[1];
    odom.pose.pose.position.y = -odometrics[0];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = "base_footprint";
		odom.twist.twist.linear.x = metrics[2]*std::cos(th);
		odom.twist.twist.linear.y = metrics[2]*std::sin(th);
    odom.twist.twist.angular.z = vth;

		// publish the message
    odom_pub.publish(odom);

		// publish steering values
		setMotorAndSteering(motorValue, steeringValue);
		sbplMotor_pub.publish(motorValue);
    sbplSteering_pub.publish(steeringValue);

    last_time = current_time;
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
